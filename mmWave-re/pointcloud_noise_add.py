import open3d as o3d
import numpy as np

def Noise_Add(group,sigma_gaussian=0.05,sigma_outliers=0.1,keep_ratio_dropout=0.88):
    '''
    本函数用于添加噪点，有以下三个功能：
    1.添加高斯噪声(gaussian)
    2.添加随机离群点(outliers)
    3.随机删除点(dropout)

    这一部分的参数调整需要负责PointTR的同学尝试修正，这里我的取值仅作测试
    '''
    points = np.asarray(group.points)
    #添加高斯噪点
    noise = np.random.normal(0, sigma_gaussian, points.shape)
    points_noisy = points + noise

    #添加离群点
    num_outliers = int(sigma_outliers * len(points))
    outliers = np.random.uniform(-1, 1, (num_outliers, 3))
    points_noisy = np.concatenate([points_noisy, outliers], axis=0)

    #随机删除点
    indices = np.random.choice(len(points), int(len(points_noisy)*keep_ratio_dropout), replace=False)
    points_sparse = points_noisy[indices]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_sparse)
    
    print("已添加噪点✅")
    return pcd