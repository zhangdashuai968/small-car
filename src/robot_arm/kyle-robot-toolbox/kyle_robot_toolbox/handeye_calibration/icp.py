import numpy as np
import cv2

def icp_solver_svd(a_points3d, b_points3d):
    '''计算的是从点集A到点集B的变换矩阵 ^{A}_{B}T 
    P_A = ^{A}_{B}P * P_B 
    把B中的点，映射为A中的点.
    '''
    # 分别计算两组点集的质心
    a_points3d_mean = np.mean(a_points3d, axis=0)
    b_points3d_mean = np.mean(b_points3d, axis=0)
    # 点集去除质心
    a_points3d_center = a_points3d - a_points3d_mean
    b_points3d_center = b_points3d - b_points3d_mean
    # 构造W矩阵
    W = np.zeros((3, 3), dtype="float32")
    for i in range(9):
        # V1 列向量
        v1 = a_points3d_center[i].reshape(-1, 1)
        # V2 行向量
        v2 = b_points3d_center[i].reshape(1, -1)
        # W矩阵累加
        W += v1.dot(v2)
    # SVD分解
    U,Sigma,V = np.linalg.svd(W)
    # 求解旋转矩阵
    R = U.dot(V.T)
    # 将旋转矩阵规范化，满足正交特性
    rvec = cv2.Rodrigues(R)[0]
    R = cv2.Rodrigues(rvec)[0]
    # 求解平移向量
    t = a_points3d_mean.reshape(-1,1) - R.dot(b_points3d_mean.reshape(-1, 1))
    return R, t
