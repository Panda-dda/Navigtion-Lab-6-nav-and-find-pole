import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 生成一个假设的柱子数据（例如沿Z轴方向的柱子）
n_points = 1000
radius = 0.05  # 柱子的半径
height = 1.0  # 柱子的高度
theta = np.linspace(0, 2 * np.pi, 100)  # 柱子的圆周分布
z = np.linspace(0, height, 100)  # 高度分布
x = radius * np.cos(theta)  # X坐标
y = radius * np.sin(theta)  # Y坐标
x = np.concatenate([x + np.random.normal(0, 0.01, 100) for _ in range(10)])
y = np.concatenate([y + np.random.normal(0, 0.01, 100) for _ in range(10)])
z = np.concatenate([z + np.random.normal(0, 0.01, 100) for _ in range(10)])

# 假设这1000个点代表柱子的形状
point_cloud = np.column_stack((x, y, z))

# 使用DBSCAN进行聚类
eps = 0.1  # 设置邻域半径
min_samples = 50  # 最小样本数
dbscan = DBSCAN(eps=eps, min_samples=min_samples)

# 聚类并标记
labels = dbscan.fit_predict(point_cloud)

# 输出聚类结果
print(f"聚类结果：\n{labels}")

# 可视化聚类结果
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制不同簇的点
for label in set(labels):
    if label != -1:  # 排除噪声点
        cluster_points = point_cloud[labels == label]
        ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], label=f"Cluster {label}")
    else:
        noise_points = point_cloud[labels == label]
        ax.scatter(noise_points[:, 0], noise_points[:, 1], noise_points[:, 2], color='k', label="Noise")

ax.set_title("DBSCAN Clustering Result - Finding a Pillar")
ax.legend()
plt.show()
