import open3d as o3d
import numpy as np

def get_sensor_array(center, direction_to_origin, width=0.2, height=0.2, num_width=20, num_height=20):
    """
    生成矩形平面阵列传感器位置。
    
    Args:
        center: 阵列中心点 (x, y, z)
        direction_to_origin: 从中心指向原点(物体)的方向向量，用于确定阵列的朝向
        width: 阵列的水平宽度 (米)
        height: 阵列的垂直高度 (米)
        num_width: 水平方向的传感器数量
        num_height: 垂直方向的传感器数量
        
    Returns:
        numpy array: 形状为 (num_width * num_height, 3) 的传感器坐标数组
    """
    
    #归一化方向向量 (Forward)
    forward = direction_to_origin / np.linalg.norm(direction_to_origin)
    
    # 计算水平方向向量 (Right / Tangent)
    # 通过与全局 Z 轴叉乘得到水平向量
    right = np.cross(forward, np.array([0, 0, 1]))
    if np.linalg.norm(right) < 1e-6: 
        # 如果视线平行于 Z 轴（例如从正上方看），强制指定一个 X 轴方向
        right = np.array([1, 0, 0])
    right = right / np.linalg.norm(right)
    
    # 构建一个正交坐标系
    up = np.cross(right, forward)
    up = up / np.linalg.norm(up)
    
    # 4. 生成网格点
    sensor_positions = []
    
    # 生成水平和垂直方向的偏移量 linspace
    x_offsets = np.linspace(-width / 2, width / 2, num_width)
    y_offsets = np.linspace(-height / 2, height / 2, num_height)
    
    # 双重循环生成网格
    for y in y_offsets:
        for x in x_offsets:
            pos = center + (right * x) + (up * y)
            sensor_positions.append(pos)
            
    return np.array(sensor_positions)


def get_180_degree_scan_trajectory(
    distance=3.0,
    num_views=60,
    array_width=0.2,
    array_height=0.2,
    pts_per_row=20,
    pts_per_col=20,
    seed=None
):
    """
    在半径为distance的球面上生成一个【随机方向的大圆】，并沿该大圆扫描 180 度，这是我们模拟的雷达扫描轨迹
    """

    if seed is not None:
        np.random.seed(seed)

    all_sensor_positions = []

    # 1. 随机生成一个大圆所在平面的法向量 n
    n = np.random.randn(3)
    n /= np.linalg.norm(n)

    # 2. 在该平面内构造两个正交基 u, v
    # 先随便找一个不平行的向量
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(tmp, n)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])

    u = np.cross(n, tmp)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)  # 已自动单位化

    # 3. 180° 扫描角
    angles = np.linspace(-np.pi / 2, np.pi / 2, num_views)

    print("开始生成随机大圆扫描轨迹")
    print(f"圆平面法向量 n = {n}")

    for theta in angles:
        center_pos = distance * (np.cos(theta) * u + np.sin(theta) * v)

        # 始终指向原点
        direction_to_origin = -center_pos

        current_view_sensors = get_sensor_array(
            center=center_pos,
            direction_to_origin=direction_to_origin,
            width=array_width,
            height=array_height,
            num_width=pts_per_row,
            num_height=pts_per_col
        )

        all_sensor_positions.append(current_view_sensors)

    return np.vstack(all_sensor_positions)
