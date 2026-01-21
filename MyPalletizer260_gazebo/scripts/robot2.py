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

teleop_help = """
╔══════════════════════════════════════════════════════════════╗
║            MyPalletizer 260 键盘遥控                         ║
╠══════════════════════════════════════════════════════════════╣
║  关节控制 (小写字母)                                         ║
║  ┌─────────┬─────────┬─────────┬─────────┐                   ║
║  │ Joint 1 │ Joint 2 │ Joint 3 │ Joint 4 │                   ║
║  │  底座   │  大臂   │  小臂   │  末端   │                   ║
║  ├─────────┼─────────┼─────────┼─────────┤                   ║
║  │  w / s  │  e / d  │  r / f  │  t / g  │                   ║
║  │  +   -  │  +   -  │  +   -  │  +   -  │                   ║
║  └─────────┴─────────┴─────────┴─────────┘                   ║
║                                                              ║
║  夹爪控制                                                    ║
║  ┌─────────────────────────────┐                             ║
║  │  o = 打开夹爪   p = 关闭夹爪│                             ║
║  └─────────────────────────────┘                             ║
║                                                              ║
║  其他功能                                                    ║
║  ┌─────────────────────────────┐                             ║
║  │  1 = 回到Home    q = 退出   │                             ║
║  └─────────────────────────────┘                             ║
╚══════════════════════════════════════════════════════════════╝
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
def publish_arm_to_gazebo(ctrl_deg_list, duration=0.1):
    """
    发布机械臂的角度到 Gazebo。
    duration: 轨迹执行时间（秒），默认0.1秒实现快速响应
    """
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = CTRL_ARM_NAMES[:]
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(a) for a in ctrl_deg_list]
    pt.time_from_start = rospy.Duration(duration)
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
def clear_screen():
    """清屏"""
    print("\033[2J\033[H", end="")

def print_status(deg_ctrl, hw_angles, gripper_state, joint_limits):
    """打印当前状态面板"""
    clear_screen()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║            MyPalletizer 260 键盘遥控                         ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  关节控制 (小写字母)                                         ║")
    print("║  ┌─────────┬─────────┬─────────┬─────────┐                   ║")
    print("║  │ Joint 1 │ Joint 2 │ Joint 3 │ Joint 4 │                   ║")
    print("║  │  底座   │  大臂   │  小臂   │  末端   │                   ║")
    print("║  ├─────────┼─────────┼─────────┼─────────┤                   ║")
    print("║  │  w / s  │  e / d  │  r / f  │  t / g  │                   ║")
    print("║  │  +   -  │  +   -  │  +   -  │  +   -  │                   ║")
    print("║  └─────────┴─────────┴─────────┴─────────┘                   ║")
    print("║                                                              ║")
    print("║  目标角度 (Gazebo)                                           ║")
    angles_str = "  ".join([f"J{i+1}:{deg_ctrl[i]:+7.1f}°" for i in range(len(deg_ctrl))])
    print(f"║  {angles_str:<58} ║")
    print("║                                                              ║")
    print("║  实际角度 (硬件)                                             ║")
    if hw_angles and len(hw_angles) >= len(deg_ctrl):
        hw_str = "  ".join([f"J{i+1}:{hw_angles[i]:+7.1f}°" for i in range(len(deg_ctrl))])
    else:
        hw_str = "读取中..."
    print(f"║  {hw_str:<58} ║")
    print("║                                                              ║")
    print("║  限位范围                                                    ║")
    limits_str = "  ".join([f"[{joint_limits[i][0]:+4.0f},{joint_limits[i][1]:+4.0f}]" for i in range(min(len(joint_limits), len(deg_ctrl)))])
    print(f"║  {limits_str:<58} ║")
    print("║                                                              ║")
    print("║  夹爪控制                                                    ║")
    print("║  ┌─────────────────────────────────────────┐                 ║")
    grip_status = "打开" if gripper_state == 0 else "关闭"
    print(f"║  │  o = 打开    p = 关闭    当前: {grip_status:<6}  │                 ║")
    print("║  └─────────────────────────────────────────┘                 ║")
    print("║                                                              ║")
    print("║  其他: 1 = Home    q = 退出                                  ║")
    print("╚══════════════════════════════════════════════════════════════╝")

def get_hw_angles():
    """从硬件读取当前关节角度"""
    try:
        angles = mc.get_angles()
        if isinstance(angles, (list, tuple)) and len(angles) >= 4:
            return list(angles)
    except Exception:
        pass
    return None

def teleop():
    step = float(rospy.get_param("~step_deg", 1.0))
    refresh_interval = float(rospy.get_param("~refresh_interval", 0.5))  # 刷新间隔（秒）
    
    # 每个关节的独立限位（度），与实际硬件保持一致
    joint_limits = [
        (-162, 162),   # joint1_to_base
        (-2, 90),      # joint2_to_joint1
        (-55, 55),     # joint3_to_joint2 (实际硬件限位约±55°)
        (-145, 145),   # joint5_to_joint4
    ]
    # 角度数组（控制器顺序！）
    deg_ctrl = [0.0] * len(CTRL_ARM_NAMES)
    gripper_state = 0  # 0=打开, 1=关闭

    t = threading.Thread(target=command_executor, daemon=True)
    t.start()

    keymap = {'w':(0,+1),'s':(0,-1),
              'e':(1,+1),'d':(1,-1),
              'r':(2,+1),'f':(2,-1),
              't':(3,+1),'g':(3,-1)}

    last_refresh = 0.0
    need_refresh = True
    hw_angles = None

    with RawTerminal():
        while not rospy.is_shutdown():
            # 定时刷新显示
            now = time.time()
            if need_refresh or (now - last_refresh >= refresh_interval):
                hw_angles = get_hw_angles()  # 读取硬件角度
                print_status(deg_ctrl, hw_angles, gripper_state, joint_limits)
                last_refresh = now
                need_refresh = False

            k = get_key_non_blocking()
            if k is None:
                time.sleep(0.01)
                continue

            if k == 'q':
                break

            elif k == '1':
                deg_ctrl = [0.0] * len(CTRL_ARM_NAMES)
                add_command("angles", deg_ctrl)
                need_refresh = True
                continue

            elif k in ('o', 'p'):
                action = 0 if k == 'o' else 1
                gval = MYCOBOT_GRIP_MIN if action == 0 else MYCOBOT_GRIP_MAX
                try:
                    mc.set_gripper_state(action, 100)
                    time.sleep(0.25)
                except Exception as e:
                    pass
                publish_gripper_to_gazebo(gval)
                gripper_state = action
                need_refresh = True
                continue

            elif k not in keymap:
                continue

            idx, sgn = keymap[k]
            if idx >= len(deg_ctrl):
                continue

            jmin, jmax = joint_limits[idx] if idx < len(joint_limits) else (-180, 180)
            nv = deg_ctrl[idx] + sgn * step
            if nv < jmin or nv > jmax:
                continue

            deg_ctrl[idx] = nv
            add_command("angles", deg_ctrl)
            need_refresh = True


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

    # 启动信息（简短，因为teleop会显示完整面板）
    print("\n" + "═" * 62)
    print("  ✓ 硬件连接: {} @ {}".format(connected_port, connected_baud))
    print("  ✓ 启动中...")
    print("═" * 62)

    try:
        teleop()
    finally:
        try:
            if hasattr(mc, "release_all_servos"): mc.release_all_servos()
        except Exception:
            pass
        try: mc.close()
        except Exception: pass
        print("\n" + "═" * 62)
        print("  程序已退出，舵机已释放")
        print("═" * 62 + "\n")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
