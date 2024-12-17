#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion



def move_to_goal():
    # 初始化节点
    rospy.init_node('move_base_client')

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
    goal.target_pose.pose.position = Point(1.0, 1.0, 0.0)  # 设置目标点的x, y坐标
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # 设置目标点的朝向 (无旋转)

    # 发送目标并等待结果
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)

    # 等待目标完成
    client.wait_for_result()

    # 检查结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot has reached the goal!")
    else:
        rospy.loginfo("The robot failed to reach the goal.")

if __name__ == '__main__':
    try:
        move_to_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test interrupted.")
