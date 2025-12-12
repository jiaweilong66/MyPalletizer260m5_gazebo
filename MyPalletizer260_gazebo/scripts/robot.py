#!/usr/bin/env python3

# -- coding: utf-8 --
"""
slider_control_compatible.py
双模式滑块控制脚本（兼容 MyPalletizer260）
1: 滑块 -> Gazebo 控制器
2: 滑块 -> 真实 MyPalletizer260 机械臂 (自动检测关节数，兼容 3/4 轴)
"""
import time
import math
import threading
import queue
from collections import deque
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import serial.tools.list_ports

# 全局变量
mc = None
mode = 2
pub_arm = None
pub_gripper = None

# 优化参数
ANGLE_THRESHOLD = 3.0 # 角度变化阈值(度)
GRIPPER_THRESHOLD = 10.0 # 夹爪角度变化阈值(度)
MAX_COMMAND_RATE = 10.0 # 最大命令频率(Hz)
COMMAND_QUEUE_SIZE = 5 # 命令队列大小

# 动态关节设置（NUM_JOINTS 在连接后或者从第一个 joint_states 推断）
NUM_JOINTS = None

# 默认关节名称模板（若你的 joint_states 名称不同，会在运行时自动替换）
DEFAULT_ARM_JOINTS = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint5_to_joint4",
    "joint4_to_joint3",
]
ARM_JOINTS = DEFAULT_ARM_JOINTS.copy()

# 初始 JOINT_LIMITS 可先为空，连接后会按 NUM_JOINTS 填充
JOINT_LIMITS = []
GRIPPER_JOINT = "gripper_controller"
GRIPPER_LIMITS = (-40, 40)

# 状态记录
last_angles = None
last_gripper_angle = None
last_command_time = 0
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
is_executing = False

# 超限警告控制
last_warning_time = {} # 每个关节的最后警告时间
WARNING_INTERVAL = 3.0 # 警告间隔(秒)

# 统计信息
stats = {
    'total_messages': 0,
    'commands_sent': 0,
    'commands_skipped': 0,
    'limit_violations': 0,
    'errors': 0
}

def slider_to_gripper_value(slider_value):
    """
    将滑块值 [0, 100] 映射到夹爪控制值 [-0.570, 0.128]
    """
    gripper_value = -0.570 + (slider_value / 100.0) * (0.128 - (-0.570))
    return gripper_value

def find_available_port():
    ports = serial.tools.list_ports.comports()
    priority_keywords = ['ACM', 'USB', 'Arduino', 'CH340', 'CP210', 'FTDI']
    for keyword in priority_keywords:
        for port in ports:
            if (keyword in port.device.upper() or keyword in port.description.upper() or keyword in str(port.hwid).upper()):
                rospy.loginfo(f"[串口检测] 找到优先串口: {port.device} ({port.description})")
                return port.device
    if ports:
        selected_port = ports[0].device
        rospy.loginfo(f"[串口检测] 使用第一个可用串口: {selected_port} ({ports[0].description})")
        return selected_port
    rospy.logwarn("[串口检测] 未找到任何可用串口，使用默认值")
    return "/dev/ttyUSB0"

def list_available_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.loginfo("[串口检测] 没有找到可用串口")
        return
    rospy.loginfo("[串口检测] === 可用串口列表 ===")
    for i, port in enumerate(ports):
        rospy.loginfo(f"[串口检测] {i+1}. 设备: {port.device}")
        rospy.loginfo(f"[串口检测] 描述: {port.description}")
        rospy.loginfo(f"[串口检测] 硬件ID: {port.hwid}")
        rospy.loginfo("[串口检测] ---")

def test_port_connectivity(port, baud):
    try:
        try:
            from pymycobot.mypalletizer260 import MyPalletizer260
            test_mc = MyPalletizer260(port, baud)
        except Exception:
            from pymycobot.mycobot import MyCobot
            test_mc = MyCobot(port, baud)
        time.sleep(1.0)
        angles = None
        try:
            angles = test_mc.get_angles()
        except Exception:
            try:
                angles = test_mc.get_radians()
            except Exception:
                angles = None
        if hasattr(test_mc, "close"):
            try:
                test_mc.close()
            except Exception:
                pass
        rospy.loginfo(f"[串口测试] ✅ 端口 {port} 连接成功，当前角度: {angles}")
        return True
    except Exception as e:
        rospy.logwarn(f"[串口测试] ❌ 端口 {port} 连接失败: {e}")
        return False

def smart_port_selection():
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.logwarn("[智能选择] 未找到任何串口设备")
        return "/dev/ttyUSB0"
    rospy.loginfo("[智能选择] 开始智能串口选择...")
    priority_keywords = ['ACM', 'USB', 'Arduino', 'CH340', 'CP210', 'FTDI']
    for keyword in priority_keywords:
        for port in ports:
            if (keyword in port.device.upper() or port.description and keyword in port.description.upper() or keyword in str(port.hwid).upper()):
                rospy.loginfo(f"[智能选择] 正在测试高优先级端口: {port.device}")
                if test_port_connectivity(port.device, 115200):
                    return port.device
    rospy.loginfo("[智能选择] 高优先级端口测试失败，尝试所有可用端口...")
    for port in ports:
        rospy.loginfo(f"[智能选择] 正在测试端口: {port.device}")
        if test_port_connectivity(port.device, 115200):
            return port.device
    rospy.logerr("[智能选择] 所有端口测试失败，使用默认端口")
    return "/dev/ttyUSB0"

class RobotCommand:
    def __init__(self, cmd_type, data, timestamp=None):
        self.type = cmd_type  # 'angles' or 'gripper'
        self.data = data
        self.timestamp = timestamp or time.time()

def is_mycobot_connected():
    global mc
    try:
        if mc is None:
            return False
        _ = mc.get_angles()
        return True
    except Exception:
        return False

def check_angle_limits(angles, gripper_angle):
    global last_warning_time, stats, NUM_JOINTS
    current_time = time.time()
    violations = []
    if NUM_JOINTS is None:
        for i, angle in enumerate(angles):
            if i < len(JOINT_LIMITS):
                min_limit, max_limit = JOINT_LIMITS[i]
                if angle < min_limit or angle > max_limit:
                    joint_key = f"joint{i+1}"
                    if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                        violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°{max_limit}°)")
                    last_warning_time[joint_key] = current_time
                    stats['limit_violations'] += 1
    else:
        for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
            if angle < min_limit or angle > max_limit:
                joint_key = f"joint{i+1}"
                if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                    violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
                last_warning_time[joint_key] = current_time
                stats['limit_violations'] += 1
    min_grip, max_grip = GRIPPER_LIMITS
    if gripper_angle is not None and (gripper_angle < min_grip or gripper_angle > max_grip):
        gripper_key = "gripper"
        if gripper_key not in last_warning_time or current_time - last_warning_time[gripper_key] > WARNING_INTERVAL:
            violations.append(f"夹爪: {gripper_angle:.1f}° (限制: {min_grip}°~{max_grip}°)")
        last_warning_time[gripper_key] = current_time
        stats['limit_violations'] += 1
    if violations:
        rospy.logwarn(f"[slider_control] ⚠️ 角度超限:")
        for v in violations:
            rospy.logwarn(f"[slider_control] {v}")
    return len(violations) == 0

def clamp_angles(angles, gripper_angle):
    clamped_angles = []
    limits = JOINT_LIMITS if JOINT_LIMITS else [(-180, 180)] * len(angles)
    for i, angle in enumerate(angles):
        if i < len(limits):
            min_limit, max_limit = limits[i]
        else:
            min_limit, max_limit = -180, 180
        clamped_angles.append(max(min_limit, min(max_limit, angle)))
    min_grip, max_grip = GRIPPER_LIMITS
    clamped_gripper = None
    if gripper_angle is not None:
        clamped_gripper = max(min_grip, min(max_grip, gripper_angle))
    return clamped_angles, clamped_gripper

def calculate_angle_difference(angles1, angles2):
    if angles1 is None or angles2 is None:
        return float('inf')
    return sum(abs(a - b) for a, b in zip(angles1, angles2))

def should_send_command(new_angles, new_gripper_angle):
    global last_angles, last_gripper_angle, last_command_time
    current_time = time.time()
    if current_time - last_command_time < 1.0 / MAX_COMMAND_RATE:
        return False, "频率限制"
    angle_diff = calculate_angle_difference(new_angles, last_angles)
    gripper_diff = abs(new_gripper_angle - last_gripper_angle) if (last_gripper_angle is not None and new_gripper_angle is not None) else float('inf')
    if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
        return False, f"角度变化太小 (臂:{angle_diff:.1f}°, 夹爪:{gripper_diff:.1f}°)"
    return True, "允许发送"

def add_command_to_queue(command):
    try:
        command_queue.put_nowait(command)
        return True
    except queue.Full:
        try:
            _ = command_queue.get_nowait()  # 弹出最旧
            command_queue.put_nowait(command)
            return True
        except queue.Empty:
            return False

def command_executor():
    global is_executing, last_angles, last_gripper_angle, last_command_time, stats, NUM_JOINTS, mc
    while not rospy.is_shutdown():
        try:
            command = command_queue.get(timeout=1.0)
            if not is_mycobot_connected():
                rospy.logwarn("[slider_control] MyCobot未连接，跳过命令")
                stats['errors'] += 1
                command_queue.task_done()
                continue
            is_executing = True
            try:
                if command.type == 'angles':
                    angles_to_send = command.data
                    if NUM_JOINTS is not None:
                        angles_to_send = angles_to_send[:NUM_JOINTS]
                    if len(angles_to_send) < NUM_JOINTS:
                        if last_angles and len(last_angles) >= NUM_JOINTS:
                            angles_to_send = (angles_to_send + last_angles)[:NUM_JOINTS]
                        else:
                            angles_to_send = angles_to_send + [0.0] * (NUM_JOINTS - len(angles_to_send))
                    try:
                        mc.send_angles(angles_to_send, 40)
                    except Exception as e:
                        try:
                            mc.send_angles(angles_to_send, 40)
                        except Exception as e2:
                            rospy.logerr(f"[slider_control] 发送角度失败: {e} / {e2}")
                            raise
                    last_angles = angles_to_send.copy()
                    rospy.logdebug(f"[slider_control] 发送角度: {angles_to_send}")
                elif command.type == 'gripper':
                    # 使用 set_gripper_state 释放夹爪
                    try:
                        mc.set_gripper_state(10, 80)  # 释放夹爪，参数10代表夹爪的状态
                        rospy.logdebug(f"[slider_control] 夹爪释放")
                        stats['commands_sent'] += 1
                    except Exception as e:
                        rospy.logwarn(f"[slider_control] 无法释放夹爪: {e}")
                    last_gripper_angle = None
                    stats['commands_sent'] += 1
                    last_command_time = time.time()
            except Exception as e:
                rospy.logerr(f"[slider_control] 命令执行失败: {e}")
                stats['errors'] += 1
            finally:
                is_executing = False
            command_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"[slider_control] 命令执行器错误: {e}")
            is_executing = False


def callback(msg: JointState):
    global stats, NUM_JOINTS, ARM_JOINTS, last_angles, last_gripper_angle
    stats['total_messages'] += 1
    name_to_deg = {}
    for name, pos in zip(msg.name, msg.position):
        try:
            name_to_deg[name] = round(math.degrees(pos), 1)
        except Exception:
            try:
                name_to_deg[name] = round(float(pos), 1)
            except Exception:
                continue
    if NUM_JOINTS is None:
        candidates = [n for n in msg.name if n != GRIPPER_JOINT]
        matched = [n for n in DEFAULT_ARM_JOINTS if n in name_to_deg]
        if matched:
            ARM_JOINTS = matched.copy()
            NUM_JOINTS = len(ARM_JOINTS)
        else:
            take = min(len(candidates), 4) if len(candidates) > 0 else min(len(msg.name), 4)
            if take == 0:
                ARM_JOINTS = DEFAULT_ARM_JOINTS[:4]
                NUM_JOINTS = 4
            else:
                ARM_JOINTS = candidates[:take]
                NUM_JOINTS = take
    JOINT_LIMITS.clear()
    JOINT_LIMITS.extend([(-180, 180)] * NUM_JOINTS)
    rospy.loginfo(f"[slider_control] 推断到关节数: {NUM_JOINTS}, ARM_JOINTS: {ARM_JOINTS}")
    arm_deg = [0.0] * NUM_JOINTS
    for i, joint_name in enumerate(ARM_JOINTS):
        if joint_name in name_to_deg:
            arm_deg[i] = name_to_deg[joint_name]
    grip_deg = name_to_deg.get(GRIPPER_JOINT, None)
    check_angle_limits(arm_deg, grip_deg)
    if mode == 1:
        clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
        publish_to_gazebo(clamped_arm, clamped_grip if clamped_grip is not None else 0.0)
    elif mode == 2:
        should_send, reason = should_send_command(arm_deg, grip_deg if grip_deg is not None else 0.0)
        if should_send:
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg if grip_deg is not None else 0.0)
            send_arm = clamped_arm[:NUM_JOINTS]
            arm_command = RobotCommand('angles', send_arm)
            if add_command_to_queue(arm_command):
                gripper_diff = abs(clamped_grip - last_gripper_angle) if (last_gripper_angle is not None and clamped_grip is not None) else float('inf')
                if clamped_grip is not None and gripper_diff >= GRIPPER_THRESHOLD:
                    add_command_to_queue(RobotCommand('gripper', clamped_grip))
                else:
                    stats['commands_skipped'] += 1
            else:
                stats['commands_skipped'] += 1
        rospy.logdebug(f"[slider_control] 跳过命令: {reason}")

def publish_to_gazebo(arm_deg, grip_deg):
    global pub_arm, pub_gripper
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ARM_JOINTS[:len(arm_deg)]
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(d) for d in arm_deg]
    pt.time_from_start = rospy.Duration(0.1)
    traj.points = [pt]
    pub_arm.publish(traj)
    traj_g = JointTrajectory()
    traj_g.header.stamp = rospy.Time.now()
    traj_g.joint_names = [GRIPPER_JOINT]
    ptg = JointTrajectoryPoint()
    ptg.positions = [math.radians(grip_deg)]
    ptg.time_from_start = rospy.Duration(0.1)
    traj_g.points = [ptg]
    pub_gripper.publish(traj_g)

def initialize_mycobot():
    global mc, NUM_JOINTS, ARM_JOINTS, last_angles, last_gripper_angle, JOINT_LIMITS
    list_available_ports()
    port = rospy.get_param("~port", None)
    baud = rospy.get_param("~baud", 115200)
    if port is None:
        rospy.loginfo("[slider_control] 未指定串口，启动智能串口选择...")
        port = smart_port_selection()
    else:
        rospy.loginfo(f"[slider_control] 使用指定串口: {port}")
    if not test_port_connectivity(port, baud):
        rospy.logwarn(f"[slider_control] 指定串口 {port} 连接失败，尝试自动检测...")
        port = smart_port_selection()
    rospy.loginfo(f"[slider_control] 最终选择串口: {port} @ {baud}")
    try:
        try:
            from pymycobot.mypalletizer260 import MyPalletizer260
            mc = MyPalletizer260(port, baud)
            rospy.loginfo("[slider_control] 使用 MyPalletizer260 驱动")
        except Exception as e:
            from pymycobot.mycobot import MyCobot
            mc = MyCobot(port, baud)
            rospy.loginfo("[slider_control] 使用 MyCobot 驱动 (回退)")
        time.sleep(1.5)
        try:
            current_angles = mc.get_angles()
        except Exception:
            try:
                current_angles = mc.get_radians()
                current_angles = [math.degrees(a) for a in current_angles]
            except Exception:
                current_angles = None
        if current_angles is None:
            rospy.logwarn("[slider_control] 无法从设备读取初始角度，仍然继续，但无法确定关节数")
            NUM_JOINTS = None
        else:
            NUM_JOINTS = len(current_angles)
        ARM_JOINTS = DEFAULT_ARM_JOINTS[:NUM_JOINTS]
        JOINT_LIMITS = [(-180, 180)] * NUM_JOINTS
        last_angles = [round(a, 1) for a in current_angles]
        try:
            g = None
            if hasattr(mc, "get_gripper_value"):
                g = mc.get_gripper_value()
            if isinstance(g, list):
                last_gripper_angle = float(g[0])
            elif g is not None:
                last_gripper_angle = float(g)
            else:
                last_gripper_angle = None
        except Exception:
            last_gripper_angle = None
        rospy.loginfo(f"[slider_control] ✅ MyCobot 连接成功! 关节数: {NUM_JOINTS}, 当前角度: {current_angles}")
        try:
            if hasattr(mc, "release_all_servos"):
                mc.release_all_servos()
                mc.set_gripper_state(10,80)
            elif hasattr(mc, "release_servo"):
                if NUM_JOINTS is not None:
                    for sid in range(1, NUM_JOINTS + 1):
                        try:
                            mc.release_servo(sid)
                        except Exception:
                            pass
        except Exception:
            pass
        time.sleep(0.5)
        return True
    except Exception as e:
        rospy.logerr(f"[slider_control] ❌ MyCobot初始化失败: {e}")
        rospy.logerr("[slider_control] 请检查: 1. 机械臂连接 2. 串口权限 3. 其他程序占用串口 4. 机械臂电源")
        return False

def print_stats():
    if stats['total_messages'] > 0:
        efficiency = (stats['commands_sent'] / stats['total_messages']) * 100
        rospy.loginfo(f"[slider_control] 统计: 消息:{stats['total_messages']}, 发送:{stats['commands_sent']}, 跳过:{stats['commands_skipped']}, 超限:{stats['limit_violations']}, 错误:{stats['errors']}, 效率:{efficiency:.1f}%")

def main():
    global mc, mode, pub_arm, pub_gripper, NUM_JOINTS
    rospy.init_node("slider_control_optimized", anonymous=True)
    print("\nSelect control mode:")
    print(" 1: Slider → Gazebo")
    print(" 2: Slider → Real MyPalletizer260 (Optimized Auto Port Detection)")
    inp = input("Enter 1 or 2 (default 2): ").strip()
    mode = 1 if inp == "1" else 2
    rospy.loginfo(f"[slider_control] 模式: {'Gazebo' if mode==1 else 'Real Robot (优化版+自动串口检测)'}")
    rospy.loginfo(f"[slider_control] 配置: 角度阈值={ANGLE_THRESHOLD}°, 最大频率={MAX_COMMAND_RATE}Hz, 队列大小={COMMAND_QUEUE_SIZE}")
    rospy.loginfo(f"[slider_control] 安全限制: 关节默认±180°, 夹爪{GRIPPER_LIMITS[0]}°~{GRIPPER_LIMITS[1]}°")
    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
    if mode == 2:
        if not initialize_mycobot():
            rospy.logerr("[slider_control] MyCobot初始化失败，退出")
            return
    executor_thread = threading.Thread(target=command_executor, daemon=True)
    executor_thread.start()
    rospy.loginfo("[slider_control] 异步命令执行器已启动")
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    rospy.loginfo("[slider_control] 🚀 节点启动成功，等待滑块输入...")
    rospy.loginfo("[slider_control] 💡 超限时会自动限制角度并显示警告")
    rospy.loginfo("[slider_control] 🔌 支持自动串口检测和智能端口选择")
    
    def stats_timer():
        while not rospy.is_shutdown():
            time.sleep(10.0)
            print_stats()
            
    stats_thread = threading.Thread(target=stats_timer, daemon=True)
    stats_thread.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 收到中断信号，正在关闭...")
    finally:
        print_stats()
        if mc is not None:
            try:
                if hasattr(mc, "release_all_servos"):
                    mc.release_all_servos()
                elif hasattr(mc, "release_servo") and NUM_JOINTS is not None:
                    for sid in range(1, NUM_JOINTS + 1):
                        try:
                            mc.release_servo(sid)
                        except Exception:
                            pass
                rospy.loginfo("[slider_control] 已释放所有舵机（如接口支持）")
            except Exception:
                pass

if __name__ == "__main__":
    main()
