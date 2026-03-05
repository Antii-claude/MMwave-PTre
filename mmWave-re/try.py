import open3d as o3d
import numpy as np
from generate_radar import get_random_sensor_position
from generate_radar import get_sensor_array



def specularity_aware_filter(pcd, sensor_positions, tau_degrees=15):
    """
    实现 Wave-Former 论文公式 (2): 镜面反射感知归纳偏置
    
    参数:
        pcd: open3d.geometry.PointCloud, 完整的输入点云 (必须包含法向量)
        sensor_positions: numpy array (N, 3), 传感器阵列的位置集合 P
        tau_degrees: float, 阈值角度 (论文中的 tau)，单位是度
    
    返回:
        filtered_pcd: open3d.geometry.PointCloud, 筛选后的稀疏点云
    """
    
    # --- 1. 可见性检查 V(si) ---
    # 论文注3提到使用 hidden_point_removal。
    # 由于 HPR 需要一个单一视点，我们使用传感器阵列的几何中心作为视点。
    # radius 设置得很大(比如 100倍物体尺寸)，模拟近似平行投影或远距离观测。

    #某个传感器的位置
    center_pos = sensor_positions
    _, visible_indices = pcd.hidden_point_removal(camera_location=center_pos, radius=100)
    pcd_visible = pcd.select_by_index(visible_indices)
    
    # 点坐标和法向量
    points = np.asarray(pcd_visible.points)
    normals = np.asarray(pcd_visible.normals)
    
    # 如果点云没有法向量，需要先计算
    if len(normals) == 0:
        pcd_visible.estimate_normals()
        normals = np.asarray(pcd_visible.normals)

    o3d.visualization.draw_geometries([pcd_visible])
    pass

pcd = o3d.io.read_point_cloud(r"normalization_dataset\1024_ply\1024\battery\battery_002\pcd_1024.ply")
sensor_center = get_random_sensor_position(distance=3) # 假设雷达距离物体 2 米（1米内的球体内是归一化后的三维点云）
# 视线方向是从传感器指向原点
view_dir = -sensor_center / np.linalg.norm(sensor_center) #计算欧几里得距离
# view_dir是一个方向向量
P = get_sensor_array(sensor_center, view_dir) 
#P就是我们的线性雷达阵列，传入一个传感器点的位置和方向信息进行线性延展
specularity_aware_filter(pcd,P[2])