#!/usr/bin/env python3

import time
import rospy
from pymycobot.mycobot import MyCobot
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 全局变量
mc = None
pub_arm = None
pub_gripper = None

# 默认参数
GRIPPER_JOINT = "gripper_controller"
GRIPPER_LIMITS = (-40, 40)

def initialize_mycobot():
    """初始化MyCobot"""
    global mc
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    try:
        mc = MyCobot(port, baud)
        rospy.loginfo("[slider_control] MyCobot初始化成功")
        return True
    except Exception as e:
        rospy.logerr(f"[slider_control] MyCobot初始化失败: {e}")
        return False

def release_gripper():
    """释放夹爪"""
    try:
        mc.set_gripper_state(1, 80, 1)  # 10 - release, 80 - speed, 1 - 自适应夹爪
        rospy.loginfo("[slider_control] 夹爪释放成功")
    except Exception as e:
        rospy.logerr(f"[slider_control] 夹爪释放失败: {e}")


def publish_to_gazebo(arm_deg, grip_deg):
    """发布控制命令到 Gazebo"""
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ["joint1_to_base", "joint2_to_joint1"]  # 示例，实际需要根据关节数目调整
    pt = JointTrajectoryPoint()
    pt.positions = [arm_deg]  # 假设这是关节1的角度，实际上你可能需要多个角度
    pt.time_from_start = rospy.Duration(0.1)
    traj.points = [pt]
    pub_arm.publish(traj)

    # 发布夹爪命令
    traj_g = JointTrajectory()
    traj_g.header.stamp = rospy.Time.now()
    traj_g.joint_names = [GRIPPER_JOINT]
    ptg = JointTrajectoryPoint()
    ptg.positions = [grip_deg]
    ptg.time_from_start = rospy.Duration(0.1)
    traj_g.points = [ptg]
    pub_gripper.publish(traj_g)

def callback(msg: JointState):
    """接收关节状态消息"""
    arm_deg = 0.0  # 假设的一个角度值，实际应从msg中读取
    grip_deg = 0.0  # 假设的夹爪角度
    publish_to_gazebo(arm_deg, grip_deg)

def main():
    global pub_arm, pub_gripper, mc
    rospy.init_node("slider_control_optimized", anonymous=True)

    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)

    if not initialize_mycobot():
        rospy.logerr("[slider_control] MyCobot初始化失败，退出")
        return

    # 释放夹爪
    release_gripper()

    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 收到中断信号，正在关闭...")

if __name__ == "__main__":
    main()
