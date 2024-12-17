#!/usr/bin/env python
from math import cos, sin
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion


def send_goal_to_move_base(x, y, theta):
    """
    发送目标位置到 move_base，机器人将朝指定的目标移动。
    :param x: 目标位置的 x 坐标
    :param y: 目标位置的 y 坐标
    :param theta: 目标朝向角度（以弧度为单位）
    """
    # 创建一个动作客户端，用于连接 move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 等待服务器启动
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # 创建一个目标 (Goal)
    goal = MoveBaseGoal()

    # 设置目标位置的坐标系 (参考坐标系是 'map')
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置的坐标 (x, y, theta)
    goal.target_pose.pose.position = Point(x, y, 0.0)  # 设置目标点的x, y坐标
    goal.target_pose.pose.orientation = Quaternion(
        0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0)
    )  # 将目标朝向转换为四元数，theta为朝向角度（以弧度为单位）

    # 发送目标并等待结果
    rospy.loginfo(f"Sending goal to position: ({x}, {y}, {theta})")
    client.send_goal(goal)

    # 等待目标完成
    client.wait_for_result()

    # 检查结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot has reached the goal!")
    else:
        rospy.loginfo("The robot failed to reach the goal.")


def move_to_multiple_goals():
    """
    发送多个目标位置，演示如何多次调用目标发布函数。
    """
    goals = [
        (1.0, 1.0, 0.0),  # 第一个目标
        (2.0, 2.0, 0.0),  # 第二个目标
        (3.0, 1.0, 1.57), # 第三个目标，朝向改变（90度）
    ]

    for goal in goals:
        x, y, theta = goal
        send_goal_to_move_base(x, y, theta)  # 调用发送目标函数


if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client')
        move_to_multiple_goals()  # 调用多次发布目标位置
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test interrupted.")
