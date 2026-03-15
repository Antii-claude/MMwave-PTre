import open3d as o3d
import numpy as np

from generate_radar import get_sensor_array
from generate_radar import get_180_degree_scan_trajectory

pcd_path = r"normalization_dataset/1024_ply/1024/dinosaur/dinosaur_001/pcd_1024.ply"
pcd = o3d.io.read_point_cloud(pcd_path)
o3d.visualization.draw_geometries([pcd], window_name="Combined Radar Observation")