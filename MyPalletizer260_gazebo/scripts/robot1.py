#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import List, Optional

import rospy
import serial.tools.list_ports

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

# ---------- 驱动导入 ----------
MyDriver = None
try:
    from pymycobot.mypalletizer260 import MyPalletizer260 as _DRV
    MyDriver = _DRV
except Exception:
    try:
        from pymycobot.mycobot import MyCobot as _DRV
        MyDriver = _DRV
    except Exception:
        MyDriver = None

# ---------- 全局 ----------
mc = None
pub_arm = None
pub_gripper = None

CTRL_ARM_NAMES: List[str] = []     # 控制器期望的关节名（顺序固定）
GRIPPER_NAME: str = "gripper_joint"
ANGLE_INDEX_MAP: List[int] = []    # 控制器序 -> 硬件序 的索引映射

last_sent_deg = None
_last_cmd_time = 0.0

ANGLE_THRESHOLD_DEG = 0.3  # 滑块模式的微小变化抑制
MAX_HZ = 30.0

# =============== 工具：打印与串口 ===============

def _log_names(prefix: str, names: List[str]):
    rospy.loginfo(f"{prefix} {names} (len={len(names)})")

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.loginfo("[port] 没有可用串口")
        return
    rospy.loginfo("[port] === 可用串口 ===")
    for i, p in enumerate(ports):
        rospy.loginfo(f"[port] {i+1}. {p.device} | {p.description} | {p.hwid}")

def pick_port() -> str:
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.logwarn("[port] 未找到串口，回退 /dev/ttyUSB0")
        return "/dev/ttyUSB0"
    prio = ["ACM", "USB", "CH340", "CP210", "FTDI", "Arduino"]
    for kw in prio:
        for p in ports:
            if kw in (p.device or "").upper() or kw in (p.description or "").upper() or kw in (str(p.hwid) or "").upper():
                rospy.loginfo(f"[port] 优先选择: {p.device} ({p.description})")
                return p.device
    rospy.loginfo(f"[port] 使用第一个: {ports[0].device} ({ports[0].description})")
    return ports[0].device

# =============== 获取控制器 joint_names ===============

_state_once = {"got": False, "names": []}
def _state_cb(msg: JointTrajectoryControllerState):
    if not _state_once["got"] and msg.joint_names:
        _state_once["names"] = list(msg.joint_names)
        _state_once["got"] = True

def get_arm_names_from_param(param_name: str) -> Optional[List[str]]:
    try:
        names = rospy.get_param(param_name)
        if isinstance(names, list) and names:
            return [str(x) for x in names]
    except Exception:
        pass
    return None

def get_arm_names_from_state(state_topic: str, timeout=2.0) -> Optional[List[str]]:
    sub = rospy.Subscriber(state_topic, JointTrajectoryControllerState, _state_cb, queue_size=1)
    start = time.time()
    while time.time() - start < timeout and not _state_once["got"]:
        rospy.sleep(0.05)
    sub.unregister()
    return _state_once["names"][:] if _state_once["got"] else None

def resolve_controller_names(arm_param, grip_param, arm_state_topic, override_arm: List[str], override_grip: Optional[str]):
    global CTRL_ARM_NAMES, GRIPPER_NAME
    if override_arm:
        CTRL_ARM_NAMES = [str(x) for x in override_arm]
        _log_names("[names] 使用 ~arm_joint_names:", CTRL_ARM_NAMES)
    else:
        names = get_arm_names_from_param(arm_param)
        if names:
            CTRL_ARM_NAMES = names
            _log_names(f"[names] 从 {arm_param} 获取:", CTRL_ARM_NAMES)
        else:
            names = get_arm_names_from_state(arm_state_topic, timeout=2.0)
            if names:
                CTRL_ARM_NAMES = names
                _log_names(f"[names] 从 {arm_state_topic} 获取:", CTRL_ARM_NAMES)
            else:
                rospy.logerr(f"[names] 无法获取控制器关节名，请配置 ~arm_joint_names 或 {arm_param}")
                raise RuntimeError("missing controller joint names")

    if override_grip:
        GRIPPER_NAME = str(override_grip)
    else:
        gnames = get_arm_names_from_param(grip_param)
        if gnames and len(gnames) >= 1:
            GRIPPER_NAME = str(gnames[0])
    rospy.loginfo(f"[names] 夹爪关节名: {GRIPPER_NAME}")

# =============== 夹爪映射 ===============

def map_gripper(hw_deg: Optional[float], hw_min, hw_max, sim_min, sim_max, prev_sim: Optional[float]) -> float:
    if hw_deg is None or hw_deg in (254, 255):
        return prev_sim if prev_sim is not None else sim_min
    try:
        t = (hw_deg - hw_min) / float(hw_max - hw_min)
        sim = sim_min + t * (sim_max - sim_min)
        return max(sim_min, min(sim_max, sim))
    except Exception:
        return prev_sim if prev_sim is not None else sim_min

# =============== 读角净化（修复 int 迭代错误） ===============

def _sanitize_seq(obj):
    """
    把驱动返回的角度值净化为 list[float] 或 None：
    - list/tuple -> list[float]
    - str/bytes -> 逗号分割为数字列表
    - 标量(int/float) -> 认为无效（返回 None）
    - 其他 -> None
    """
    if obj is None:
        return None
    if isinstance(obj, (list, tuple)):
        if len(obj) == 0:
            return None
        try:
            return [float(x) for x in obj]
        except Exception:
            return None
    if isinstance(obj, (str, bytes)):
        try:
            s = obj.decode() if isinstance(obj, bytes) else obj
            parts = [p.strip() for p in s.split(",") if p.strip() != ""]
            if not parts:
                return None
            return [float(p) for p in parts]
        except Exception:
            return None
    if isinstance(obj, (int, float)):
        return None
    return None

# =============== 硬件 I/O（稳健版） ===============

def hw_read_angles_deg() -> Optional[List[float]]:
    """
    稳健读取：优先度制，其次弧度；对返回类型做净化。
    某些固件在未就绪/错误时会返回 0/254/255 这类 int，需要忽略并重试。
    """
    try:
        ang = None
        if hasattr(mc, "get_angles"):
            raw = mc.get_angles()
            ang = _sanitize_seq(raw)
        if ang is None and hasattr(mc, "get_radians"):
            raw_r = mc.get_radians()
            rad_list = _sanitize_seq(raw_r)
            if rad_list:
                ang = [math.degrees(x) for x in rad_list]
        if not ang:
            return None
        return [float(a) for a in ang]
    except Exception as e:
        rospy.logerr_throttle(1.0, f"[hw] 读角失败: {e}")
        return None

def hw_read_gripper_deg() -> Optional[float]:
    """
    尝试多种 API，净化返回类型：
    - list/tuple -> 取第一个
    - 标量 -> float
    - 254/255 -> 视为无效
    """
    try:
        for api in ("get_gripper_value", "get_gripper", "get_finger_value"):
            if hasattr(mc, api):
                raw = getattr(mc, api)()
                seq = _sanitize_seq(raw)
                if seq:
                    val = float(seq[0])
                else:
                    if isinstance(raw, (int, float)):
                        val = float(raw)
                    else:
                        continue
                if val in (254.0, 255.0):
                    return None
                return val
    except Exception:
        pass
    return None

def hw_send_angles_deg(deg_list: List[float], speed=40) -> bool:
    try:
        if hasattr(mc, "send_angles"):
            mc.send_angles(deg_list, speed)
            return True
    except Exception as e1:
        try:
            if hasattr(mc, "send_radians"):
                mc.send_radians([math.radians(d) for d in deg_list], speed)
                return True
        except Exception as e2:
            rospy.logerr_throttle(1.0, f"[hw] 发送角失败: {e1} / {e2}")
    return False

def hw_power_on_if_possible():
    try:
        if hasattr(mc, "power_on"):
            mc.power_on()
        elif hasattr(mc, "focus_all_servos"):
            mc.focus_all_servos()
    except Exception:
        pass

def hw_release_all(joint_ids: Optional[List[int]] = None):
    """尽力释放全部关节（含夹爪）。接口因固件而异，这里做最大兼容尝试。"""
    try:
        if hasattr(mc, "release_all_servos"):
            mc.release_all_servos()
            mc.set_gripper_state(flag=10, speed=50)
            rospy.loginfo("[release] release_all_servos() 已调用")
            return
    except Exception:
        pass
    try:
        if hasattr(mc, "release_servo"):
            ids = joint_ids if (joint_ids and len(joint_ids) > 0) else list(range(1, 13))
            cnt = 0
            for sid in ids:
                try:
                    mc.release_servo(int(sid))
                    cnt += 1
                except Exception:
                    pass
            rospy.loginfo(f"[release] release_servo() 尝试 ID: {ids}, 成功≈{cnt}")
    except Exception:
        pass
    try:
        if hasattr(mc, "power_off"):
            mc.power_off()
            rospy.loginfo("[release] power_off() 已调用")
    except Exception:
        pass
def hw_release_gripper(speed: int = 50, type_1: Optional[int] = None):
    """
    独立的夹爪释放：flag=10。兼容可选的 type_1（1: 自适应，2: 五指，3: 平行，4: 柔性）。
    """
    if mc is None:
        return
    try:
        if hasattr(mc, "set_gripper_state"):
            if type_1 is None:
                mc.set_gripper_state(flag=10, speed=int(speed))
            else:
                mc.set_gripper_state(flag=10, speed=int(speed), _type_1=int(type_1))
            rospy.loginfo(f"[release] gripper release: flag=10, speed={speed}, type={type_1}")
            return
    except Exception as e:
        rospy.logwarn(f"[release] set_gripper_state 失败: {e}")
    # 兜底：若存在其它可能 API，可在此补充（不同固件可能叫法不同）
    for alt_api in ("set_finger_state", ):
        try:
            if hasattr(mc, alt_api):
                getattr(mc, alt_api)(10, int(speed))
                rospy.loginfo(f"[release] gripper release via {alt_api}")
                return
        except Exception:
            continue
# =============== Gazebo 发布 ===============

def send_arm_to_gazebo(deg_in_ctrl_order: List[float], joint_names: List[str], tfs: float, arm_topic: str):
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names[:]   # 必须与控制器期望一致
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(d) for d in deg_in_ctrl_order]
    pt.velocities = [0.0] * len(deg_in_ctrl_order)
    pt.accelerations = [0.0] * len(deg_in_ctrl_order)
    pt.time_from_start = rospy.Duration.from_sec(tfs)
    traj.points = [pt]
    pub_arm.publish(traj)

def send_gripper_to_gazebo(sim_pos: float, gripper_joint: str, tfs: float, grip_topic: str):
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = [gripper_joint]  # 必须是关节名
    pt = JointTrajectoryPoint()
    pt.positions = [sim_pos]
    pt.velocities = [0.0]
    pt.accelerations = [0.0]
    pt.time_from_start = rospy.Duration.from_sec(tfs)
    traj.points = [pt]
    pub_gripper.publish(traj)

# =============== 索引映射 ===============

def build_index_map(hardware_deg_len: int, opt_map_param: List[int]) -> List[int]:
    """
    构造 控制器序 -> 硬件序 映射。
    若未指定 angle_index_map，则默认顺序为 [0,1,2,4,3,...]（交换关节4与5）。
    """
    if opt_map_param and len(opt_map_param) == len(CTRL_ARM_NAMES):
        return [int(i) for i in opt_map_param]

    n = len(CTRL_ARM_NAMES)
    base = list(range(n))

    # 若存在至少5个关节，则交换索引3与4（即 joint4 和 joint5）
    if n >= 5:
        base[3], base[4] = base[4], base[3]

    return base


# =============== slider 源回调 ===============

def slider_cb(msg: JointState, cfg):
    global last_sent_deg, _last_cmd_time

    # 解析为度（优先弧度 -> 度）
    name2deg = {}
    for name, pos in zip(msg.name, msg.position):
        try:
            name2deg[name] = math.degrees(float(pos))
        except Exception:
            try:
                name2deg[name] = float(pos)
            except Exception:
                continue

    ctrl_deg = [float(name2deg.get(jn, 0.0)) for jn in CTRL_ARM_NAMES]

    now = time.time()
    if now - _last_cmd_time < 1.0 / cfg["max_hz"]:
        return
    if last_sent_deg is not None:
        diff = sum(abs(a - b) for a, b in zip(ctrl_deg, last_sent_deg))
        if diff < len(ctrl_deg) * cfg["arm_eps"]:
            return

    # 真机：仅在允许运动时才下发（否则保持释放）
    if cfg["allow_hw_motion"]:
        if ANGLE_INDEX_MAP and len(ANGLE_INDEX_MAP) >= len(ctrl_deg):
            # 控制器序 -> 硬件序
            hw_deg = [0.0] * len(ctrl_deg)
            for k, hw_i in enumerate(ANGLE_INDEX_MAP[:len(ctrl_deg)]):
                if hw_i < len(hw_deg):
                    hw_deg[hw_i] = ctrl_deg[k]
        else:
            hw_deg = ctrl_deg[:]
        hw_send_angles_deg(hw_deg, speed=cfg["hw_speed"])

    last_sent_deg = ctrl_deg[:]
    _last_cmd_time = now

    # Gazebo
    send_arm_to_gazebo(ctrl_deg, CTRL_ARM_NAMES, cfg["arm_tfs"], cfg["arm_topic"])

# =============== 初始化与主程序 ===============

def init_hardware(port: Optional[str], baud: int) -> bool:
    global mc
    if MyDriver is None:
        rospy.logerr("[init] 未找到 pymycobot，请先 pip install pymycobot")
        return False
    if port is None:
        list_ports()
        port = pick_port()
    rospy.loginfo(f"[init] 串口: {port} @ {baud}")
    try:
        mc = MyDriver(port, baud)
        time.sleep(1.0)
        rospy.loginfo("[init] 硬件连接成功")
        return True
    except Exception as e:
        rospy.logerr(f"[init] 硬件连接失败: {e}")
        return False

def main():
    rospy.init_node("sync_bridge_real_sim", anonymous=True)

    # 参数
    source = rospy.get_param("~source", "hardware")
    port = rospy.get_param("~port", None)
    baud = int(rospy.get_param("~baud", 115200))
    rate_hz = float(rospy.get_param("~rate_hz", 30.0))

    arm_topic = rospy.get_param("~arm_cmd_topic", "/arm_controller/command")
    grip_topic = rospy.get_param("~grip_cmd_topic", "/gripper_controller/command")
    arm_state_topic = rospy.get_param("~arm_state_topic", "/arm_controller/state")
    arm_param = rospy.get_param("~arm_param_joints", "/arm_controller/joints")
    grip_param = rospy.get_param("~grip_param_joints", "/gripper_controller/joints")

    override_arm = rospy.get_param("~arm_joint_names", [])
    override_grip = rospy.get_param("~gripper_joint_name", "")

    angle_index_map_param = rospy.get_param("~angle_index_map", [])

    arm_tfs = float(rospy.get_param("~arm_time_from_start", 0.1))
    grip_tfs = float(rospy.get_param("~grip_time_from_start", 0.1))

    g_hw_min = float(rospy.get_param("~gripper_hw_min", 3.0))
    g_hw_max = float(rospy.get_param("~gripper_hw_max", 91.0))
    g_sim_min = float(rospy.get_param("~gripper_sim_min", -0.68))
    g_sim_max = float(rospy.get_param("~gripper_sim_max", 0.15))

    # 释放相关
    release_on_start = bool(rospy.get_param("~release_on_start", True))
    release_joint_ids = rospy.get_param("~release_joint_ids", [])
    allow_hw_motion = bool(rospy.get_param("~allow_hw_motion", not release_on_start))

    # 发布器
    global pub_arm, pub_gripper
    pub_arm = rospy.Publisher(arm_topic, JointTrajectory, queue_size=10)
    pub_gripper = rospy.Publisher(grip_topic, JointTrajectory, queue_size=10)

    # 控制器关节名
    resolve_controller_names(
        arm_param=arm_param,
        grip_param=grip_param,
        arm_state_topic=arm_state_topic,
        override_arm=override_arm,
        override_grip=override_grip if override_grip else None,
    )

    # 连接硬件
    if not init_hardware(port, baud):
        return

    # 启动即释放 or 上力矩
    if release_on_start:
        try:
            hw_release_all(release_joint_ids if isinstance(release_joint_ids, list) else None)
            # 显式释放夹爪（flag=10）
            hw_release_gripper(speed=50)
            rospy.loginfo("[init] 启动后已释放所有关节，并显式释放夹爪")
        except Exception as e:
            rospy.logwarn(f"[init] 释放关节时发生异常: {e}")
    else:
        hw_power_on_if_possible()

    # 映射（控制器序 -> 硬件序）
    hw_len = len(hw_read_angles_deg() or [])
    global ANGLE_INDEX_MAP
    ANGLE_INDEX_MAP = build_index_map(hw_len, angle_index_map_param)
    rospy.loginfo(f"[map] angle_index_map (ctrl->hw): {ANGLE_INDEX_MAP}")

    if source.lower() == "slider":
        # 以滑块为主
        cfg = {
            "max_hz": max(5.0, rate_hz),
            "arm_eps": ANGLE_THRESHOLD_DEG,
            "allow_hw_motion": allow_hw_motion,
            "hw_speed": 40,
            "arm_tfs": arm_tfs,
            "grip_tfs": grip_tfs,
            "arm_topic": arm_topic,
            "grip_topic": grip_topic,
            "g_hw_min": g_hw_min,
            "g_hw_max": g_hw_max,
            "g_sim_min": g_sim_min,
            "g_sim_max": g_sim_max,
        }
        rospy.Subscriber("/joint_states", JointState, slider_cb, cfg, queue_size=1)
        rospy.loginfo(f"[mode] slider → sim，同步运行；向硬件下发：{allow_hw_motion}")
        rospy.spin()
        return

    # 默认：hardware → sim（硬件保持释放，不下发动作）
    rospy.loginfo("[mode] hardware → sim，同步运行")
    rate = rospy.Rate(rate_hz)
    last_sim_grip = None

    while not rospy.is_shutdown():
        deg_hw = hw_read_angles_deg()
        if deg_hw:
            n = min(len(CTRL_ARM_NAMES), len(ANGLE_INDEX_MAP))
            ctrl_deg = []
            for k in range(n):
                idx_in_hw = ANGLE_INDEX_MAP[k] if k < len(ANGLE_INDEX_MAP) else k
                val = deg_hw[idx_in_hw] if idx_in_hw < len(deg_hw) else 0.0
                ctrl_deg.append(float(val))
            send_arm_to_gazebo(ctrl_deg, CTRL_ARM_NAMES, arm_tfs, arm_topic)

        g_hw = hw_read_gripper_deg()
        sim_g = map_gripper(g_hw, g_hw_min, g_hw_max, g_sim_min, g_sim_max, last_sim_grip)
        last_sim_grip = sim_g
        send_gripper_to_gazebo(sim_g, GRIPPER_NAME, grip_tfs, grip_topic)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
