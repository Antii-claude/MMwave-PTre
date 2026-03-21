import open3d as o3d
import numpy as np

pcd_path = r"normalization_dataset\1024_ply\1024\cheese\cheese_004\pcd_1024.ply"
pcd = o3d.io.read_point_cloud(pcd_path)
o3d.visualization.draw_geometries([pcd], window_name="原始点云")