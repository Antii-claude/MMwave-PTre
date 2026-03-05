import open3d as o3d
import open3d.t as o3dt
import open3d.core as o3c
import numpy as np
from pathlib import Path

folder = Path("your/folder/path")

for file_path in folder.iterdir():
    print(file_path)



def get_random_sensor_position(distance=1.0):
    # 在球面上随机取一个点
    theta = np.random.uniform(0, 2 * np.pi) # 方位角
    phi = np.random.uniform(0, np.pi)       # 极角
    
    x = distance * np.sin(phi) * np.cos(theta)
    y = distance * np.sin(phi) * np.sin(theta)
    z = distance * np.cos(phi)
    
    return np.array([x, y, z])

sensor_center = get_random_sensor_position(distance=3) # 假设雷达距离物体 2米（）
