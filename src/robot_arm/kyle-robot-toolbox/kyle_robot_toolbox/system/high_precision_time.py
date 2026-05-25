import time

TIME_NS2S = 1.0 / (10**9)

def get_cur_time_s():
    '''获取高精度时间 单位s'''
    #return time.time_ns() * TIME_NS2S
    return time.time()

def sleep_s(delay_s):
	'''高精度延时'''
	t_target =  get_cur_time_s() + delay_s
	while get_cur_time_s() < t_target:
		pass
