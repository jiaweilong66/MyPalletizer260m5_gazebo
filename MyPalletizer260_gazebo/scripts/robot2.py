#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, math, time, threading, queue, termios, tty, select
import rospy
import serial.tools.list_ports
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointTrajectoryControllerState

# ============== 260 驱动优先，失败回退 MyCobot ==============
DriverClass = None
try:
    from pymycobot.mypalletizer260 import MyPalletizer260
    DriverClass = MyPalletizer260
except Exception:
    try:
        from pymycobot.mycobot import MyCobot
        DriverClass = MyCobot
    except Exception:
        DriverClass = None

# ============== 全局 ==============
mc = None
connected_port = None
connected_baud = None

pub_arm_command = None
pub_gripper_command = None
pub_angles = None

command_queue = queue.Queue(maxsize=5)
stop_executor = False
last_cmd_time = 0.0
last_gripper_time = 0.0

MIN_CMD_INTERVAL = 0.05
MIN_GRIPPER_INTERVAL = 0.5

CTRL_ARM_NAMES = []            # 控制器期望的 joint_names（发布时就用这个，绝不写死）
GRIPPER_JOINT_NAME = "gripper_joint"  # 夹爪“关节名”，不是控制器名
ANGLE_INDEX_MAP = []           # 控制器序 -> 硬件序（默认交换4/5，可参数覆盖）

MYCOBOT_GRIP_MIN = 3.0
MYCOBOT_GRIP_MAX = 91.0
GAZEBO_GRIP_MIN = -0.68
GAZEBO_GRIP_MAX = 0.15

teleop_help = """\
Teleop (lowercase)
w/s e/d r/f t/g : joint1..4 ++/-- 
o / p           : gripper open / close (反转：Gazebo o=开夹爪，p=关夹爪)
1               : home (速度降低)
q               : quit
"""

# ============== 终端工具 ==============
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)

def get_key_non_blocking():
    try:
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
    except Exception:
        pass
    return None

# ============== 串口探测 ==============
def find_ports():
    return list(serial.tools.list_ports.comports())

def get_optimal_baudrate(port):
    for baud in [115200, 1000000, 500000, 256000, 230400, 57600, 38400, 19200, 9600]:
        try:
            test = DriverClass(port, baud)
            time.sleep(0.5)
            ok = False
            try:
                a = test.get_angles()
                ok = isinstance(a, (list,tuple)) and len(a) >= 3
            except Exception:
                pass
            try: test.close()
            except Exception: pass
            if ok: return baud
        except Exception:
            continue
    return 115200

def smart_connect():
    global connected_port, connected_baud
    port = rospy.get_param("~port", None)
    baud = int(rospy.get_param("~baud", 115200))
    if port:
        try:
            test = DriverClass(port, baud)
            time.sleep(0.8)
            a = None
            try: a = test.get_angles()
            except Exception: pass
            if isinstance(a, (list,tuple)) and len(a) >= 3:
                connected_port, connected_baud = port, baud
                return test
            test.close()
        except Exception as e:
            rospy.loginfo(f"[connect] 指定串口失败: {e}")
    ports = find_ports()
    if not ports:
        rospy.logerr("[connect] 没有可用串口")
        return None
    filtered = []
    for p in ports:
        name = (p.device or "").lower()
        desc = (p.description or "").lower()
        if "ama0" in name: continue
        if "usb" in name or "acm" in name or any(x in desc for x in ["ch340","ch341","cp210","ftdi","serial"]):
            filtered.append(p)
    for p in filtered or ports:
        try:
            ob = get_optimal_baudrate(p.device)
            test = DriverClass(p.device, ob)
            time.sleep(0.8)
            a = None
            try: a = test.get_angles()
            except Exception: pass
            if isinstance(a, (list,tuple)) and len(a) >= 3:
                connected_port, connected_baud = p.device, ob
                return test
            test.close()
        except Exception:
            continue
    return None

# ============== 控制器 joint_names 解析（关键） ==============
_state_once = {"got": False, "names": []}
def _state_cb(msg: JointTrajectoryControllerState):
    if not _state_once["got"] and msg.joint_names:
        _state_once["names"] = list(msg.joint_names)
        _state_once["got"] = True

def names_from_param(param_name):
    try:
        v = rospy.get_param(param_name)
        if isinstance(v, list) and v:
            return [str(x) for x in v]
    except Exception:
        pass
    return None

def get_ctrl_arm_names():
    # 1) rosparam: /arm_controller/joints
    names = names_from_param("/arm_controller/joints")
    if names: return names
    # 2) 从 /arm_controller/state 试
    sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, _state_cb, queue_size=1)
    start = time.time()
    while time.time() - start < 2.0 and not _state_once["got"]:
        rospy.sleep(0.05)
    sub.unregister()
    if _state_once["got"]:
        return _state_once["names"][:]
    # 3) 参数覆盖
    override = rospy.get_param("~arm_joint_names", [])
    if isinstance(override, list) and override:
        return [str(x) for x in override]
    # 4) 兜底（260常见4轴模板）
    rospy.logwarn("[names] 无法自动获取关节名，使用默认模板（如不动请用 ~arm_joint_names 指定）")
    return ["joint1_to_base","joint2_to_joint1","joint3_to_joint2","joint4_to_joint3"]

def get_gripper_joint_name():
    # 优先从 /gripper_controller/joints 读第一个
    g = names_from_param("/gripper_controller/joints")
    if g and len(g) >= 1:
        return str(g[0])
    # 参数覆盖
    ov = rospy.get_param("~gripper_joint_name", "")
    if ov:
        return str(ov)
    # 默认
    return "gripper_joint"

def build_index_map(ctrl_len, opt_map):
    if isinstance(opt_map, list) and len(opt_map) == ctrl_len:
        return [int(i) for i in opt_map]
    # 默认顺序，但如果 >=5 轴，把第4/5互换（你之前的需求）
    base = list(range(ctrl_len))
    if ctrl_len >= 5:
        # 默认交换关节4和关节5
        base[3], base[4] = base[4], base[3]  # 交换关节4和关节5
    return base

# ============== Gazebo 同步 ==============
def publish_arm_to_gazebo(ctrl_deg_list):
    """
    发布机械臂的角度到 Gazebo。
    控制器顺序的角度（度）需要转换为 Gazebo 顺序。
    """
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = CTRL_ARM_NAMES[:]  # 必须与控制器期望完全一致
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(a) for a in ctrl_deg_list]  # 转换为弧度
    pt.time_from_start = rospy.Duration(0.5)  # 回到 Home 位置速度降低
    traj.points = [pt]
    pub_arm_command.publish(traj)

def map_gripper_value(hw_value):
    t = (float(hw_value) - MYCOBOT_GRIP_MIN) / (MYCOBOT_GRIP_MAX - MYCOBOT_GRIP_MIN)
    return GAZEBO_GRIP_MAX - t * (GAZEBO_GRIP_MAX - GAZEBO_GRIP_MIN)

def publish_gripper_to_gazebo(hw_value):
    """
    优化的发布夹爪状态到Gazebo的函数。
    确保以合理的频率发送命令到Gazebo。
    hw_value 是硬件的夹爪角度，从 3（打开）到 91（关闭）。
    将硬件值映射为 Gazebo 的夹爪角度，然后发布。
    """
    mapped_grip = map_gripper_value(hw_value)  # 将硬件值映射到 Gazebo范围
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = [GRIPPER_JOINT_NAME]  # 夹爪关节名，通过控制器
    pt = JointTrajectoryPoint()
    pt.positions = [mapped_grip]  # 转换后的夹爪角度值
    pt.time_from_start = rospy.Duration(0.1)  # 控制间隔 100ms
    traj.points = [pt]
    pub_gripper_command.publish(traj)

# ============== ROS 发布器 ==============
def init_publishers():
    global pub_arm_command, pub_gripper_command, pub_angles
    arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
    grip_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")
    pub_arm_command = rospy.Publisher(arm_topic, JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher(grip_topic, JointTrajectory, queue_size=10)
    pub_angles = rospy.Publisher("/joint_command_angles", Float64MultiArray, queue_size=1)
    rospy.loginfo(f"[ros] publishers ready: {arm_topic}, {grip_topic}")
# ============== Gazebo 同步 ==============
def publish_arm_to_gazebo(ctrl_deg_list):
    """
    发布机械臂的角度到 Gazebo。
    控制器顺序的角度（度）需要转换为 Gazebo 顺序。
    """
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = CTRL_ARM_NAMES[:]  # 必须与控制器期望完全一致
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(a) for a in ctrl_deg_list]  # 转换为弧度
    pt.time_from_start = rospy.Duration(2.5)  # 回到 Home 位置速度降低
    traj.points = [pt]
    pub_arm_command.publish(traj)

# ============== 键盘执行（异步） ==============
# ============== 键盘执行（异步） ==============
def command_executor():
    global last_cmd_time, last_gripper_time, stop_executor
    while not rospy.is_shutdown() and not stop_executor:
        try:
            cmd = command_queue.get(timeout=0.5)
            try:
                now = time.time()
                if cmd["type"] == "angles":
                    if now - last_cmd_time < MIN_CMD_INTERVAL:
                        rospy.sleep(MIN_CMD_INTERVAL - (now - last_cmd_time))
                    deg_ctrl = cmd["data"]
                    # 控制器序 -> 硬件序
                    deg_hw = [0.0]*len(deg_ctrl)
                    for k, hw_i in enumerate(ANGLE_INDEX_MAP[:len(deg_ctrl)]):
                        if 0 <= hw_i < len(deg_ctrl):
                            deg_hw[hw_i] = deg_ctrl[k]
                    # 真机
                    try: mc.send_angles(deg_hw, 70)
                    except Exception as e: rospy.logwarn(f"[hw] 发送角度失败: {e}")
                    # Gazebo
                    publish_arm_to_gazebo(deg_ctrl)
                    try: pub_angles.publish(Float64MultiArray(data=deg_ctrl))
                    except Exception: pass
                    last_cmd_time = time.time()

                elif cmd["type"] == "gripper":
                    if now - last_gripper_time < MIN_GRIPPER_INTERVAL:
                        continue
                    action = int(cmd["data"])
                    try:
                        # control hardware gripper here
                        publish_gripper_to_gazebo(3.0 if action == 0 else 91.0)
                        last_gripper_time = time.time()
                    except Exception as e:
                        rospy.logwarn(f"[hw] 夹爪失败: {e}")
            finally:
                command_queue.task_done()
        except queue.Empty:
            continue



def add_command(cmd_type, data):
    if cmd_type == "gripper":
        try:
            command_queue.put_nowait({"type":"gripper","data":int(data)})
        except queue.Full:
            rospy.logwarn("[queue] 夹爪队列已满，忽略")
    else:
        try:
            command_queue.put_nowait({"type":"angles","data":list(data)})
        except queue.Full:
            try:
                old = command_queue.get_nowait()
                if old["type"]=="angles":
                    command_queue.put_nowait({"type":"angles","data":list(data)})
            except queue.Empty: pass


# ============== 主键盘循环 ==============
def teleop():
    print(teleop_help)
    step = float(rospy.get_param("~step_deg", 1.0))
    jmin = float(rospy.get_param("~joint_min_deg", -180))
    jmax = float(rospy.get_param("~joint_max_deg", 180))
    # 角度数组（控制器顺序！）
    deg_ctrl = [0.0] * len(CTRL_ARM_NAMES)

    t = threading.Thread(target=command_executor, daemon=True)
    t.start()

    keymap = {'w':(0,+1),'s':(0,-1),
              'e':(1,+1),'d':(1,-1),
              'r':(2,+1),'f':(2,-1),
              't':(4,+1),'g':(4,-1)}  # 修改这里，t和g控制第五关节（关节4 -> 硬件的第五关节）

    with RawTerminal():
        while not rospy.is_shutdown():
            k = get_key_non_blocking()
            if k is None:
                time.sleep(0.005)
                continue

            if k == 'q':
                break

            elif k == '1':
                # 返回Home位置
                deg_ctrl = [0.0] * len(CTRL_ARM_NAMES)
                add_command("angles", deg_ctrl)
                rospy.loginfo("home")
                continue

            elif k in ('o', 'p'):  # 改进按键逻辑
                action = 0 if k == 'o' else 1  # o=打开夹爪，p=关闭夹爪
                # 直接设置目标值：硬件夹爪 3 (open) 或 91 (close)
                gval = MYCOBOT_GRIP_MIN if action == 0 else MYCOBOT_GRIP_MAX
                try:
                    # 确保硬件夹爪同步
                    mc.set_gripper_state(action, 100)
                    time.sleep(0.25)  # 硬件动作完成的短暂延时
                except Exception as e:
                    rospy.logwarn(f"[hw] 夹爪失败: {e}")
                # 发布到 Gazebo
                publish_gripper_to_gazebo(gval)
                rospy.loginfo(f"Gazebo夹爪 {'打开' if action == 0 else '关闭'}")
                continue

            elif k not in keymap:
                continue

            # 控制臂角度的逻辑
            idx, sgn = keymap[k]
            if idx >= len(deg_ctrl):
                continue

            nv = deg_ctrl[idx] + sgn * step
            if nv < jmin or nv > jmax:
                rospy.logwarn(f"joint{idx+1} 超限: {nv:.1f}°")
                continue

            deg_ctrl[idx] = nv
            add_command("angles", deg_ctrl)
            rospy.loginfo(f"joint{idx+1} -> {nv:.1f}°")


# ============== 初始化入口 ==============
def main():
    global mc, CTRL_ARM_NAMES, GRIPPER_JOINT_NAME, ANGLE_INDEX_MAP
    rospy.init_node("mypalletizer260_keyboard_sync", anonymous=True)

    if DriverClass is None:
        rospy.logerr("未找到 pymycobot，请 pip install pymycobot"); return

    # 解析控制器关节名（关键）
    CTRL_ARM_NAMES = get_ctrl_arm_names()
    rospy.loginfo(f"[names] arm joints: {CTRL_ARM_NAMES}")
    GRIPPER_JOINT_NAME = get_gripper_joint_name()
    rospy.loginfo(f"[names] gripper joint: {GRIPPER_JOINT_NAME}")

    # 默认映射：交换第4/5；可用 ~angle_index_map 覆盖
    opt_map = rospy.get_param("~angle_index_map", [])
    ANGLE_INDEX_MAP = build_index_map(len(CTRL_ARM_NAMES), opt_map)
    rospy.loginfo(f"[map] ctrl->hw index map: {ANGLE_INDEX_MAP}")

    # 连接硬件
    mc_local = smart_connect()
    if mc_local is None:
        rospy.logerr("无法连接设备"); return
    mc = mc_local
    rospy.loginfo(f"[connect] {connected_port} @ {connected_baud}")

    # 启动即释放（含夹爪）
    try:
        if hasattr(mc, "release_all_servos"): mc.release_all_servos()
        else:
            if hasattr(mc, "release_servo"):
                for sid in range(1, 8):
                    try: mc.release_servo(sid)
                    except Exception: pass
    except Exception as e:
        rospy.logwarn(f"[init] 释放失败: {e}")

    # 发布器
    init_publishers()

    print("\n已启动。若 Gazebo 仍不动，执行：")
    print("  rosparam get /arm_controller/joints")
    print("  rostopic echo -n1 /arm_controller/state | grep -A1 joint_names")
    print("并把结果填进 ~arm_joint_names 或修正 URDF/controller 配置。")

    try:
        teleop()
    finally:
        try:
            if hasattr(mc, "release_all_servos"): mc.release_all_servos()
        except Exception:
            pass
        try: mc.close()
        except Exception: pass
        rospy.loginfo("退出。")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass