import numpy as np
import open3d as o3d
from pathlib import Path

def normalize_point_cloud(points):
    center = points.mean(axis=0)
    points_centered = points - center

    scale = np.max(np.linalg.norm(points_centered, axis=1))
    if scale < 1e-8:
        return o3d.utility.Vector3dVector(points)

    points_normalized = points_centered / scale
    return o3d.utility.Vector3dVector(points_normalized)

in_root = Path("dataset")
out_root = Path("normalization_dataset")

for ply_path in in_root.rglob("*.ply"):
    rel_path = ply_path.relative_to(in_root)
    out_path = out_root / rel_path
    out_path.parent.mkdir(parents=True, exist_ok=True)
    pcd = o3d.io.read_point_cloud(str(ply_path))
    points = np.asarray(pcd.points)

    if points.shape[0] == 0:
        print(f"跳过空点云: {rel_path}")
        continue
   
    pcd.points = normalize_point_cloud(points)
    
    o3d.io.write_point_cloud(str(out_path), pcd)

    print(f"已处理: {rel_path}")

print("全部子文件夹处理完成 ✅")