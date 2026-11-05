#!/usr/bin/env python
# -*- coding: utf-8 -*-
# grab_place.py - 抓放一条龙（依赖: ~/boot_all.sh 已跑 + ~/start_grasp.sh 已跑）
# 用法:
#   python ~/grab_place.py 苹果 香蕉      # 抓苹果, 放到香蕉标记的位置
#   python ~/grab_place.py apple banana   # 英文名也行, vlm认什么用什么
#   python ~/grab_place.py apple          # 只给一个参数 = 抓完原地松爪
# 注意: 抓取和放置的间隔默认12秒(臂完成动作), 慢了可加第三个参数加大, 如:
#   python ~/grab_place.py apple banana 15
import rospy, sys, time
from vl_locate.srv import GetObject

def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    obj = sys.argv[1]
    marker = sys.argv[2] if len(sys.argv) > 2 else obj
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0

    rospy.init_node('grab_place')
    rospy.wait_for_service('/vlm_detection', timeout=10)
    vlm = rospy.ServiceProxy('/vlm_detection', GetObject)

    rospy.loginfo('[1/2] 抓取: %s ...', obj)
    rospy.set_param('/grab', 1)
    r = vlm.call(obj)
    rospy.loginfo('    识别: %s', getattr(r, 'message', r))
    rospy.loginfo('    等待机械臂完成抓取 %.0f 秒...', hold)
    time.sleep(hold)

    rospy.loginfo('[2/2] 放置: 到 %s 标记位置松爪 ...', marker)
    rospy.set_param('/grab', 0)
    r = vlm.call(marker)
    rospy.loginfo('    识别: %s', getattr(r, 'message', r))
    time.sleep(hold)
    rospy.loginfo('=== 抓放循环完成 ===')

if __name__ == '__main__':
    main()
