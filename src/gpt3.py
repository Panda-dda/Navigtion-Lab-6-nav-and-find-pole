import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

def get_ranges_filtered():
    # 获取雷达扫描数据
    scan = rospy.wait_for_message('/scan', LaserScan)
    
    # 将激光雷达数据转换为 numpy 数组
    ranges = np.array(scan.ranges)
    
    # 过滤掉无效数据，距离小于 0.3m 或大于 10m
    ranges[ranges < 0.15] = 10.0  # 设置小于 0.3 的距离为 10m（无效数据）
    ranges[ranges > 2] = 10.0  # 设置大于 10m 的距离为 10m（无效数据）
    print(ranges)
    return ranges

def find_pillar(ranges):
    # 使用 DBSCAN 聚类来找出柱子所在区域
    # 首先，设置 DBSCAN 的参数
    eps = 0.15  # 聚类半径
    min_samples = 3  # 聚类的最小点数
    
    # 将雷达数据转为二维坐标，这里我们将极坐标转为直角坐标 (x, y)
    angles = np.linspace(0, 2 * np.pi, len(ranges), endpoint=False)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    
    # 将 (x, y) 转为特征矩阵
    points = np.column_stack((x, y))
    
    # 进行 DBSCAN 聚类
    db = DBSCAN(eps=eps, min_samples=min_samples)
    labels = db.fit_predict(points)
    
    # 过滤掉噪声点，噪声点的标签为 -1
    pillar_label = 0  # 假设柱子属于标签 0
    print(labels)
    pillar_indices = np.where(labels == pillar_label)[0]
    
    return pillar_indices, labels

def detect_pillar():
    # 获取过滤后的雷达数据
    ranges = get_ranges_filtered()
    
    # 使用 DBSCAN 查找柱子所在的索引
    pillar_indices, labels = find_pillar(ranges)
    
    # 输出柱子的索引
    if len(pillar_indices) > 0:
        print(f"检测到柱子，柱子的索引：{pillar_indices}")
    else:
        print("未检测到柱子")

    # 可视化聚类结果
    angles = np.linspace(0, 2 * np.pi, len(ranges), endpoint=False)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    
    plt.scatter(x, y, c=labels, cmap='jet', s=10)
    plt.title("Laser Scan with DBSCAN Clustering")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.colorbar()
    plt.show()

def main():
    rospy.init_node('pillar_detection', anonymous=True)
    
    detect_pillar()

if __name__ == '__main__':
    main()
