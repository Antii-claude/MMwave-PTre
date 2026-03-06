import open3d as o3d
import numpy as np
from pathlib import Path

from generate_radar import get_sensor_array
from generate_radar import get_180_degree_scan_trajectory
from pointcloud_noise_add import Noise_Add
from specularity_aware_filter import specularity_aware_filter

in_root = Path("normalization_dataset")
out_root = Path("physicalRE_dataset")

for ply_path in in_root.rglob("*.ply"):
    rel_path = ply_path.relative_to(in_root)
    out_path = out_root / rel_path
    out_path.parent.mkdir(parents=True, exist_ok=True)
    pcd = o3d.io.read_point_cloud(str(ply_path))
    points = np.asarray(pcd.points)

    if points.shape[0] == 0:
        print(f"跳过空点云: {rel_path}")
        continue
   
    pcd.points = specularity_aware_filter(points)
    pcd.points = Noise_Add(pcd.points)

    o3d.io.write_point_cloud(str(out_path), pcd)

    print(f"已处理: {rel_path}")

print("点云处理结束，全部子文件夹处理完成 ✅")