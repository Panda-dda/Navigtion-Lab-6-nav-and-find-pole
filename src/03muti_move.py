#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
import math  # 导入 math 库
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np

# def extract_and_find_closest(data):
#     indices = []
#     values = []

#     # 提取大于3的值及其索引
#     for i in range(len(data)):
#         if data[i] > 0.3:
#             values.append(data[i])
#             indices.append(i)
#     # print(data)
#     # 如果没有找到大于3的值，返回适当的值
#     if len(values) < 2:
#         return None, [], []  # 或者返回 -1

#     # 找到索引最接近的两个值
#     min_distance = float('inf')  # 初始化为正无穷大
#     closest_pair = (None, None)  # 存储索引对
#     closest_values = (None, None)  # 存储值对

#     for i in range(len(indices) - 1):
#         distance = indices[i + 1] - indices[i]  # 计算相邻索引之间的距离
#         if distance < min_distance:
#             min_distance = distance  # 更新最小距离
#             closest_pair = (indices[i], indices[i + 1])  # 更新最小距离的索引
#             closest_values = (values[i], values[i + 1])  # 更新值对

#     return closest_pair, closest_values, indices



# def detect_outliers_change_rate(data, threshold=1.5):
#     outliers = []
#     for i in range(1, len(data)):
#         # 计算变化率
#         change_rate = abs(data[i] - data[i-1]) / max(abs(data[i]), abs(data[i-1]), 1e-10)  # 避免除以零
#         if change_rate > threshold:
#             outliers.append(data[i])
#     return outliers



# def calculate_differences(data):
#     # 确保输入是 NumPy 数组
#     data = np.array(data)
    
#     # 计算一阶差分
#     first_diff = np.diff(data)
#     first_diff=abs(first_diff)
#     end=data[-1]
#     start=data[0]
    
#     absolute_difference = abs(start - end)
    
#     # 将绝对差和一阶差分组合在一起
#     first_diff_combined = np.array([absolute_difference] + list(first_diff))

#     return first_diff_combined

# class PoleGo:
#     def __init__(self, k_rho, k_alpha, k_p, k_d, k_i, allow_reverse_parking):
#         self.k_rho = k_rho
#         self.k_alpha = k_alpha
        
#         self.k_p = k_p
#         self.k_d = k_d
#         self.k_i = k_i
        
#         self.allow_reverse_parking = allow_reverse_parking
        
#         self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    

#     def get_ranges_filtered(self):
#         scan = rospy.wait_for_message('/scan', LaserScan)

        
#         ranges_filtered = np.array(scan.ranges)

#         ranges_diff=calculate_differences(ranges_filtered)
#         print(ranges_diff)

        
#         ranges_filtered[ranges_filtered < 0.14] = 10.0
        
#         return ranges_filtered.tolist(),ranges_diff.tolist()
    
#     def detect_pole(self): 
#         ranges,diff_range= self.get_ranges_filtered()
        
#         max_diff_range=np.argmax(diff_range)
        
#         nearest_idx2 = np.argmin(ranges)
        
        
#         index,_,_=extract_and_find_closest(diff_range)
        

#         nearest_idx=int((index[0]+index[1])/2)

        
        
#         alpha = (nearest_idx - 360 if nearest_idx >= 180 else nearest_idx) / 180.0 * np.pi
#         return alpha, ranges[nearest_idx],ranges[nearest_idx2]
    
#     def stop(self):
#         self.pub_cmd_vel.publish(Twist())
    
#     def run(self):
#         v = 0
#         theta = 0
        
#         integral_v = 0
#         integral_theta = 0
        
#         delta_v = 0
#         delta_theta = 0
#         # print("OK")
#         sttop=0
#         while not rospy.is_shutdown():
#             try:
#                 alpha, rho,minggg = self.detect_pole()
#                 # print()
#                 print(minggg)
#                 if minggg < 0.15:
#                     self.stop()
#                     sttop=1
#                     break   
#                 if self.allow_reverse_parking and abs(alpha) > np.pi / 2:
#                     alpha = alpha + np.pi if alpha < 0 else alpha - np.pi
#                     rho = -rho
#                 #using PID 
#                 target_v = np.clip(self.k_rho * rho * (1 if abs(alpha) < np.pi / 2 else -1), -0.22, 0.22)
#                 target_theta = np.clip(self.k_alpha * alpha, -2.84, 2.84)
                
#                 error_v = target_v - v
#                 error_theta = target_theta - theta
                
#                 integral_v += error_v
#                 integral_theta += error_theta
                
#                 delta_v = self.k_p * error_v - self.k_d * delta_v + self.k_i * integral_v
#                 delta_theta = self.k_p * error_theta - self.k_d * delta_theta + self.k_i * integral_theta
                
#                 v = np.clip(v + delta_v, -0.22, 0.22)
#                 theta = np.clip(theta + delta_theta, -2.84, 2.84)
                
#                 twist = Twist()
#                 twist.linear.x = v
#                 twist.angular.z = theta

                
#                 self.pub_cmd_vel.publish(twist)
                
#                 if sttop==1:
#                     break # break the while loop
                    
#                 # print(">>>>>>")
#             except KeyboardInterrupt:
#                 self.stop()
                
#                 break


#     #! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

def extract_and_find_closest(data, ranges):
    indices = []
    values = []

    # 提取大于 0.3 且小于 1 的突变值及其索引
    for i in range(len(data)):
        if data[i] > 0.3 and data[i] < 2:
            values.append(data[i])
            indices.append(i)

    # 如果没有找到足够的突变值，返回 None
    if len(values) < 2:
        return None, [], []  # 或者返回 -1

    # 存储所有的索引对
    index_pairs = []
    min_distance = float('inf')  # 初始化最小距离为无穷大
    best_pair = None  # 存储最优的索引对
    best_values = None  # 存储最优的突变值

    # 遍历所有索引的两两组合
    for i in range(len(indices) - 1):
        for j in range(i + 1, len(indices)):
            # 获取这两个索引对应的ranges值
            idx1, idx2 = indices[i], indices[j]
            range_value1, range_value2 = ranges[idx1], ranges[idx2]

            # 计算这两个索引在 ranges 中的距离
            # pair_distance = abs(range_value1 - range_value2)
            pair_distance= ranges[int((idx1+idx2)/2)]
            # 如果这个距离小于当前最小距离，更新最优值
            if pair_distance < min_distance:
                min_distance = pair_distance
                best_pair = (idx1, idx2)
                best_values = (range_value1, range_value2)

            # 将当前索引对加入索引组合列表
            index_pairs.append((idx1, idx2))
    print(best_pair)
    # 如果没有找到最优的索引对，返回 None
    if best_pair is None:
        return None, [], []

    # 返回最优的索引对，突变值，和所有的索引组合
    return best_pair, best_values, index_pairs


def detect_outliers_change_rate(data, threshold=1.5):
    outliers = []
    for i in range(1, len(data)):
        # 计算变化率
        change_rate = abs(data[i] - data[i-1]) / max(abs(data[i]), abs(data[i-1]), 1e-10)  # 避免除以零
        if change_rate > threshold:
            outliers.append(data[i])
    return outliers


def calculate_differences(data):
    # 确保输入是 NumPy 数组
    data = np.array(data)
    
    # 计算一阶差分
    first_diff = np.diff(data)
    first_diff = abs(first_diff)
    end = data[-1]
    start = data[0]
    
    absolute_difference = abs(start - end)
    
    # 将绝对差和一阶差分组合在一起
    first_diff_combined = np.array([absolute_difference] + list(first_diff))

    return first_diff_combined

class PoleGo:
    def __init__(self, k_rho, k_alpha, k_p, k_d, k_i, allow_reverse_parking):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        
        self.allow_reverse_parking = allow_reverse_parking
        
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    def get_ranges_filtered(self):
        scan = rospy.wait_for_message('/scan', LaserScan)
        
        ranges_filtered = np.array(scan.ranges)

        ranges_diff = calculate_differences(ranges_filtered)

        # 过滤掉距离过近的障碍物（例如小于0.14米的点）
        ranges_filtered[ranges_filtered < 0.14] = 10.0

        return ranges_filtered.tolist(), ranges_diff.tolist()

    def detect_pole(self): 
        ranges, diff_range = self.get_ranges_filtered()
        
        # 找到最大的变化差异点
        max_diff_range = np.argmax(diff_range)
        
        # 找到最近的点（距离最小的）
        nearest_idx2 = np.argmin(ranges)
        
        index, _, _ = extract_and_find_closest(diff_range,ranges)

        # 如果没有找到合适的柱子，则返回 None
        if not index:
            return None, None, None
        
        nearest_idx = int((index[0] + index[1]) / 2)
        
        # 计算柱子的方向
        alpha = (nearest_idx - 360 if nearest_idx >= 180 else nearest_idx) / 180.0 * np.pi
        return alpha, ranges[nearest_idx], ranges[nearest_idx2]
    
    def stop(self):
        self.pub_cmd_vel.publish(Twist())
    
    def run(self):
        v = 0
        theta = 0
        
        integral_v = 0
        integral_theta = 0
        
        delta_v = 0
        delta_theta = 0
        sttp=0
        while not rospy.is_shutdown():
            try:
                alpha, rho, minggg = self.detect_pole()
                if minggg is None:
                    rospy.logwarn("minggg is None, skipping this iteration")
                    continue  # 跳过这一轮循环
                if minggg < 0.15:
                    # 如果距离过近，则停止
                    sttp=1
                    self.stop()
                    break   
                
                if self.allow_reverse_parking and abs(alpha) > np.pi / 2:
                    # 如果允许倒车泊车，且角度大于90度，调整方向
                    alpha = alpha + np.pi if alpha < 0 else alpha - np.pi
                    rho = -rho
                
                # 使用PID算法控制速度和角度
                target_v = np.clip(self.k_rho * rho * (1 if abs(alpha) < np.pi / 2 else -1), -0.22, 0.22)
                target_theta = np.clip(self.k_alpha * alpha, -2.84, 2.84)
                
                error_v = target_v - v
                error_theta = target_theta - theta
                
                integral_v += error_v
                integral_theta += error_theta
                
                delta_v = self.k_p * error_v - self.k_d * delta_v + self.k_i * integral_v
                delta_theta = self.k_p * error_theta - self.k_d * delta_theta + self.k_i * integral_theta
                
                v = np.clip(v + delta_v, -0.22, 0.22)
                theta = np.clip(theta + delta_theta, -2.84, 2.84)
                
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = theta
                
                self.pub_cmd_vel.publish(twist)
                
                if sttp==1:
                    break
            except KeyboardInterrupt:
                self.stop()
                break
pole_go = PoleGo(0.6, 0.8, 0.25, 0.02, 0.01, False) 

def send_goal_to_move_base(client, x, y, z,w,en):
    """
    发送目标位置到 move_base，机器人将朝指定的目标移动。
    :param client: 动作客户端
    :param x: 目标位置的 x 坐标
    :param y: 目标位置的 y 坐标
    :param theta: 目标朝向角度（以弧度为单位）
    """
    # 创建一个目标 (Goal)
    goal = MoveBaseGoal()

    # 设置目标位置的坐标系 (参考坐标系是 'map')
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置的坐标 (x, y, theta)
    goal.target_pose.pose.position = Point(x, y, 0.0)  # 设置目标点的x, y坐标
    goal.target_pose.pose.orientation = Quaternion(
        0.0, 0.0, z, w)

    # 发送目标并等待结果
    rospy.loginfo(f"Sending goal to position: ({x}, {y}, {z},{w})")
    client.send_goal(goal)

    # 等待目标完成
    client.wait_for_result()

    # 检查结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot has reached the goal!")
        if en==1:
            pole_go.run()
            rospy.sleep(2)
    else:
        rospy.loginfo("The robot failed to reach the goal.")

    
        
def move_to_multiple_goals():
    """
    发送多个目标位置，演示如何多次调用目标发布函数。
    """
    # 创建一个动作客户端，用于连接 move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 等待服务器启动
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # 目标列表，每个目标包含 (x, y, theta)
    
    # goals map2
    # goals = [
    #     (3.8242601782038723,2.8502384636700375, -0.9251241995776339, 0.3796646090378219),
    #     (3.3003350540667302,  -0.2268991066470268,  0.9025180789054508, 0.4306519676592858),
    #     (6.618969369731982, 0.3471057075206631, 0.24822855770112875, 0.9687014933103063),
    # ]
    
    goals = [
        # (-0.83660159558687, 0.05761642685661404,  0.28734129806413994, 0.9578282614471214,0),
        
        (1.0299096492675384,1.2756173295620776, 0.2487481943322448, 0.9685681885218241,1),
        
        
        (0.17033036118311987,  4.431261090089401,  -0.10264784681743906, 0.9947177587354835,1),
        (-2.5850586366313983, 2.896818178212208, -0.9471766685919096, 0.32071226742226727,1),
        
             (-1.3907483295257783,
     -0.23079873853957641
     
    ,-0.8731747412628421
      , 0.4874072950013865,0),

        
        

        
        
    ]
    # 遍历目标列表并发送目标
    for goal in goals:
        x, y, z,w,en = goal
        send_goal_to_move_base(client, x, y, z , w,en)  # 调用发送目标函数


if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client')
        move_to_multiple_goals()  # 调用多次发布目标位置
        
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test interrupted.")














