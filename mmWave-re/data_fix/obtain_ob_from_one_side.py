import open3d as o3d
import numpy as np
from generate_radar import get_random_sensor_position
from generate_radar import get_sensor_array
 
def specularity_aware_filter(pcd, sensor_positions, tau_degrees=15):

    center_pos = sensor_positions
    _, visible_indices = pcd.hidden_point_removal(camera_location=center_pos, radius=100)
    pcd_visible = pcd.select_by_index(visible_indices)

    points = np.asarray(pcd_visible.points)
    normals = np.asarray(pcd_visible.normals)

    if len(normals) == 0:
        pcd_visible.estimate_normals()
        normals = np.asarray(pcd_visible.normals)

    
    pass

pcd = o3d.io.read_point_cloud(r"normalization_dataset\1024_ply\1024\battery\battery_002\pcd_1024.ply")
sensor_center = get_random_sensor_position(distance=3)
view_dir = -sensor_center / np.linalg.norm(sensor_center) 
P = get_sensor_array(sensor_center, view_dir) 
specularity_aware_filter(pcd,P[2])#随便取线性阵列里的第二点作为例子

o3d.visualization.draw_geometries([pcd_visible])