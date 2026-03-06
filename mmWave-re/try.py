import open3d as o3d
import numpy as np

from generate_radar import get_sensor_array
from generate_radar import get_180_degree_scan_trajectory
from pointcloud_noise_add import Noise_Add
from specularity_aware_filter import specularity_aware_filter

# 【测试程序】 

# 读取点云，这里取一个恐龙玩具的点云作为例子
pcd_path = r"normalization_dataset\1024_ply\1024\dinosaur\dinosaur_001\pcd_1024.ply"
pcd = o3d.io.read_point_cloud(pcd_path)
print(f"Loaded point cloud with {len(pcd.points)} points.")


# 1. 生成所有位置
P_all = get_180_degree_scan_trajectory(distance=3.0)

# 2. 传入滤波器
filtered_pcd = specularity_aware_filter(pcd, sensor_array=P_all, tau_degrees=25)

# 3.添加噪点
final_pcd = Noise_Add(filtered_pcd)

# 可视化结果
print("已构建最终点云，绘图完成✅ ")
o3d.visualization.draw_geometries([final_pcd], window_name="Combined Radar Observation")

