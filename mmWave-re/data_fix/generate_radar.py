import open3d as o3d
import numpy as np
import numpy as np

def get_random_sensor_position(distance=3.0):
    # 在半径为 distance 的球面上随机取一个点
    theta = np.random.uniform(0, 2 * np.pi) # 方位角
    phi = np.random.uniform(0, np.pi)       # 极角
    
    x = distance * np.sin(phi) * np.cos(theta)
    y = distance * np.sin(phi) * np.sin(theta)
    z = distance * np.cos(phi)
    
    return np.array([x, y, z])

def get_sensor_array(center, direction_to_origin, aperture_length=0.2, num_points=20):
    # 线性阵列模拟：在中心点左右展开
    
    # 1. 计算切线方向，作为我们线性阵列的线性延展方向 (传入一个视线参数，然后构建一个与该视线垂直且尽量位于水平面的单位切向量)
    tangent = np.cross(direction_to_origin, np.array([0, 0, 1]))
    if np.linalg.norm(tangent) < 1e-6: # 如果视线平行于Z轴，直接取一个水平切向量
        tangent = np.array([1, 0, 0])
    tangent = tangent / np.linalg.norm(tangent)
    
    # 2. 生成 N 个点（这里我们先假定线性排列，实际要根据我们的采集情况来定）
    sensor_positions = []
    for i in range(num_points):
        offset = (i / (num_points - 1) - 0.5) * aperture_length
        pos = center + tangent * offset
        sensor_positions.append(pos)
        
    return np.array(sensor_positions)

sensor_center = get_random_sensor_position(distance=3) # 假设雷达距离物体 2 米（1米内的球体内是归一化后的三维点云）
# 视线方向是从传感器指向原点
view_dir = -sensor_center / np.linalg.norm(sensor_center) #计算欧几里得距离
# view_dir是一个方向向量
P = get_sensor_array(sensor_center, view_dir) 
#P就是我们的线性雷达阵列，传入一个传感器点的位置和方向信息进行线性延展