import open3d as o3d
import numpy as np

from generate_radar import get_random_sensor_position
from generate_radar import get_sensor_array
from generate_radar import get_180_degree_scan_trajectory

def specularity_aware_filter(pcd, sensor_array, tau_degrees=25,theta_l=75,theta_v=75):
    """
    模拟毫米波雷达的镜面反射特性，整合阵列中所有传感器的观测结果。
        命名：
        pcd: 原始完整点云 (Open3D PointCloud)
        sensor_array: 传感器位置数组 (N x 3 numpy array)
        tau_degrees: 镜面反射阈值角度，用于模拟去除点云中无法通过镜面反射获得的点 (论文中的 tau)
        theta_l:镜面反射点的视线与水平方向的夹角阈值（用于模拟各向异性，可以调整）
        theta_v:镜面反射点的视线与垂直方向的夹角阈值（用于模拟各向异性，可以调整）
    """
    #构造法线（从原点指向center）
    if not pcd.has_normals():
        pcd.estimate_normals()
        pcd.orient_normals_towards_camera_location(pcd.get_center())
    all_points = np.asarray(pcd.points)
    all_normals = np.asarray(pcd.normals)
    
    #去重
    collected_indices = set()
    
    #做两件事：去除不可见点和不符合镜面反射的点

    #1.去掉不可见点
    #遍历阵列中的每一个传感器位置
    for sensor_pos in sensor_array:
        # radius先假定模拟平视（届时根据实际采集数据调整）
        _, visible_indices = pcd.hidden_point_removal(camera_location=sensor_pos, radius=1000)
        
        if len(visible_indices) == 0:
            continue
            
        # 提取当前传感器可见的点和法线
        visible_indices = np.array(visible_indices)
        points_vis = all_points[visible_indices]
        normals_vis = all_normals[visible_indices]
        
    #2.镜面反射检查 (论文公式2)和考虑各向异性（论文公式3）
        # 计算视线向量 (从点指向传感器) u = (p_k - s_i)
        view_vectors = sensor_pos - points_vis
        # 归一化视线向量
        norms = np.linalg.norm(view_vectors, axis=1, keepdims=True)
        # 避免除以0
        norms[norms == 0] = 1e-6
        view_vectors_normalized = view_vectors / norms

        # 计算点积
        # axis=1 表示对每一行（每个点）做点积
        dot_products = np.sum(normals_vis * view_vectors_normalized, axis=1)
        
        # 计算夹角
        # 使用 abs 获得夹角
        angles_rad = np.arccos(np.clip(np.abs(dot_products), -1.0, 1.0))
        angles_deg = np.degrees(angles_rad)
        
        level_dot_products = np.sum([1,0,0] * view_vectors_normalized, axis=1)
        level_angles_rad = np.arccos(np.clip(np.abs(level_dot_products), -1.0, 1.0))
        level_angles_deg = np.degrees(level_angles_rad)

        vertical_dot_products = np.sum([0,0,1] * view_vectors_normalized, axis=1)
        vertical_angles_rad = np.arccos(np.clip(np.abs(vertical_dot_products ), -1.0, 1.0))
        vertical_angles_deg = np.degrees(vertical_angles_rad)

        # 筛选满足镜面反射阈值 (theta < tau)且（水平与垂直夹角符合要求）的点
        is_specular = (angles_deg < tau_degrees) & (level_angles_deg < theta_l) & (vertical_angles_deg < theta_v)
        # 获取满足条件的原始索引并加入总集合
        valid_indices_for_this_sensor = visible_indices[is_specular]
        collected_indices.update(valid_indices_for_this_sensor)

        
    #根据收集到的索引构建最终点云
    if len(collected_indices) == 0:
        print("Warning: No points collected. Check normals or threshold.")
        return o3d.geometry.PointCloud()
    print("已构建最终点云，正在绘图... ")
    final_indices = list(collected_indices)
    pcd_filtered = pcd.select_by_index(final_indices)
    
    # 可视化结果
    o3d.visualization.draw_geometries([pcd_filtered], window_name="Combined Radar Observation")
    return pcd_filtered

# 【测试程序】 

# 读取点云，这里取一个恐龙玩具的点云作为例子
pcd_path = r"normalization_dataset\1024_ply\1024\dinosaur\dinosaur_001\pcd_1024.ply"
pcd = o3d.io.read_point_cloud(pcd_path)
print(f"Loaded point cloud with {len(pcd.points)} points.")


# 1. 生成所有位置
P_all = get_180_degree_scan_trajectory(distance=3.0)

# 2. 传入滤波器
filtered_pcd = specularity_aware_filter(pcd, sensor_array=P_all, tau_degrees=25)



