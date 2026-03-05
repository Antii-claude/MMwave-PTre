import open3d as o3d
import numpy as np
import generate_radar



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
    # --- 2. 计算最小角度失配 theta_P(si) ---
    # 公式: theta = min | acos( n_i . u_k,i ) |
    # 我们需要计算每个点到每个传感器位置的角度，然后取最小值。
    
    num_points = len(points)
    # 初始化最小角度为无穷大
    min_angles = np.full(num_points, np.inf)
    
    # 遍历每一个传感器位置 p_k (模拟 min_k∈P)
    for sensor_pos in sensor_positions:
        # 计算视线向量: p_k - s_i
        # view_vectors shape: (Num_points, 3)
        view_vectors = sensor_pos - points 
        
        # 归一化视线向量得到 u_k,i
        # np.linalg.norm(..., axis=1) 计算每个向量的长度
        norms = np.linalg.norm(view_vectors, axis=1, keepdims=True)
        u_ki = view_vectors / (norms + 1e-8) # 加小量防止除以0
        
        # 计算点积: n_i . u_k,i
        # 对应元素相乘后求和
        dot_products = np.sum(normals * u_ki, axis=1)
        
        # 截断数值范围到 [-1, 1] 防止 arccos 出现 NaN (浮点误差)
        dot_products = np.clip(dot_products, -1.0, 1.0)
        
        # 计算角度 (弧度)
        # 注意：这里假设法向量朝外。如果点积为负，说明法线背对传感器，角度会 > 90度。
        # 毫米波只反射正对的面，所以我们关注接近 0 度的角。
        angles = np.arccos(dot_products)
        
        # 更新每个点的最小角度
        min_angles = np.minimum(min_angles, angles)

    # --- 3. 阈值筛选 (Inductive Bias) ---
    # 将阈值转换为弧度
    tau_radians = np.radians(tau_degrees)
    
    # 生成掩码: theta < tau
    mask = min_angles < tau_radians
    
    # 提取符合条件的点
    final_points = points[mask]
    
    # 创建输出点云
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(final_points)
    
    # 可选：如果你想保留颜色
    if pcd_visible.has_colors():
        colors = np.asarray(pcd_visible.colors)
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

    return filtered_pcd

# ==========================================
# 测试代码 (生成一个球体来测试)
# ==========================================
if __name__ == "__main__":
    # 1. 生成一个球体 (模拟完整物体 F)
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    mesh.compute_vertex_normals()
    # 采样成点云
    pcd = mesh.sample_points_poisson_disk(5000)
    
    # 2. 定义传感器阵列 P (模拟雷达扫描线)
    # 假设传感器在 Z 轴正方向 2.0 处，形成一条横向的线段
    sensor_center = np.array([0, 0, 3.0])
    sensor_array = []
    # 生成 10 个传感器位置，沿 X 轴分布
    for x in np.linspace(-0.5, 0.5, 10):
        sensor_array.append(sensor_center + np.array([x, 0, 0]))
    sensor_array = np.array(sensor_array)
    
    # 3. 运行算法
    # 阈值设为 10 度，模拟非常窄的镜面反射
    result_pcd = specularity_aware_filter(pcd, sensor_array, tau_degrees=10)
    
    # 4. 可视化对比
    # 把结果涂成红色，原图涂成灰色
    pcd.paint_uniform_color([0.8, 0.8, 0.8])
    result_pcd.paint_uniform_color([1, 0, 0]) # 红色是模拟的雷达回波
    
    # 为了看清楚，把结果稍微移出来一点
    result_pcd.translate([2.2, 0, 0])
    
    print(f"原始点数: {len(pcd.points)}")
    print(f"模拟雷达点数: {len(result_pcd.points)}")
    
    o3d.visualization.draw_geometries([pcd, result_pcd], window_name="左: 原始 | 右: 论文算法模拟")
