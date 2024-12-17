    #! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

def extract_and_find_closest(data):
    indices = []
    values = []

    # 提取大于3的值及其索引
    for i in range(len(data)):
        if data[i] > 0.3 and data[i] < 1:
            values.append(data[i])
            indices.append(i)

    # 如果没有找到大于3的值，返回适当的值
    if len(values) < 2:
        return None, [], []  # 或者返回 -1

    # 找到索引最接近的两个值
    min_distance = float('inf')  # 初始化为正无穷大
    closest_pair = (None, None)  # 存储索引对
    closest_values = (None, None)  # 存储值对

    for i in range(len(indices) - 1):
        distance = indices[i + 1] - indices[i]  # 计算相邻索引之间的距离
        if distance < min_distance:
            min_distance = distance  # 更新最小距离
            closest_pair = (indices[i], indices[i + 1])  # 更新最小距离的索引
            closest_values = (values[i], values[i + 1])  # 更新值对
    return closest_pair, closest_values, indices


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
        
        index, _, _ = extract_and_find_closest(diff_range)

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

        while not rospy.is_shutdown():
            try:
                alpha, rho, minggg = self.detect_pole()
                
                if minggg < 0.15:
                    # 如果距离过近，则停止
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

            except KeyboardInterrupt:
                self.stop()
                break


def main():
    rospy.init_node('pole_Go', anonymous=True)
    
    # 初始化PoleGo对象
    pole_go = PoleGo(0.6, 0.8, 0.25, 0.02, 0.01, False)
    pole_go.run()

    rospy.spin()

if __name__ == '__main__':
    main()
