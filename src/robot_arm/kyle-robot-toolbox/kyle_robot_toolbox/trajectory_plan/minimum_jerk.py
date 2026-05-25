'''
Minimum Jerk轨迹规划
----------------------------------------------
@作者: 阿凯爱玩机器人
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import numpy as np

def minimum_jerk_plan(theta_s, theta_e, w_s, w_e, a_s, a_e, T):
    '''Minimum Jerk 规划'''
    T_pw = [1, 1, 1, 1, 1, 1]
    for i in range(1, 6):
        T_pw[i] = T_pw[i-1] * T

    A = np.float32([
        [0, 0, 0, 0, 0, 1],
        [T_pw[5], T_pw[4], T_pw[3], T_pw[2], T_pw[1], 1],
        [0, 0, 0, 0, 1, 0], 
        [5*T_pw[4], 4*T_pw[3], 3*T_pw[2], 2*T_pw[1], 1, 0],
        [0, 0, 0, 2, 0, 0],
        [20*T_pw[3], 12*T_pw[2], 6*T_pw[1], 2, 0, 0]])

    b = np.float32([theta_s, theta_e, w_s, w_e, a_s, a_e]).reshape(-1, 1)
    c = (np.linalg.inv(A)).dot(b)
    return c

def minimum_jerk_seq(T, c, delta_t=0.001):
    '''通过minimum jerk 生成序列'''
    c5, c4, c3, c2, c1, c0 = c.reshape(-1)
    t_arr = np.arange(0, T+delta_t, delta_t).reshape(-1, 1) # 时间序列
    t_arr_2 = t_arr * t_arr
    t_arr_3 = t_arr * t_arr_2
    t_arr_4 = t_arr * t_arr_3
    t_arr_5 = t_arr * t_arr_4
    
    theta_arr = c5*t_arr_5 + c4*t_arr_4 + c3*t_arr_3 + c2*t_arr_2 + c1*t_arr + c0
    t_arr = t_arr.reshape(-1)
    theta_arr = theta_arr.reshape(-1)
    return t_arr, theta_arr
