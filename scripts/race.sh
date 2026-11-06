#!/bin/bash
# ============================================================
# race.sh v3 — 比赛日总发车脚本(Ctrl-C=全停 + 命令全展示)
# 用法: ./race.sh apple pear watermelon  (或 ./race.sh 用默认值)
# 中文对照: 苹果=apple 榴莲=durian 西瓜=watermelon 葡萄=grape
#           红枣=jujube 山竹=mangosteen 梨(雪梨)=pear
# ============================================================
WS=/home/abot/catkin_ws
FRUITS_DEFAULT="apple,pear,watermelon"    # ← 当前轮:1号苹果 2号雪梨 3号西瓜

if [ $# -ge 1 ]; then FRUITS=$(echo "$@" | tr ' ' ','); else FRUITS="$FRUITS_DEFAULT"; fi

echo "=========================================================="
echo "[race] 工作空间: $WS"
echo "[race] 任务脚本: $WS/src/abot_project/abot/script/abot_race_task.py"
echo "[race] 本轮水果: $FRUITS"
echo "=========================================================="

run_show () { echo "+ $*"; eval "$@"; }
run_show "source /opt/ros/melodic/setup.bash"
run_show "source $WS/devel/setup.bash"
run_show "export ABOTMODEL=x4 CMM_MODE=rosserial ABOTBASE=omni ABOTIMU=mpu6050 ABOTLIDAR=rplidar NAV_PATH=dwa"

zero_stream () {
    timeout $1 python2 -c "
import rospy
from geometry_msgs.msg import Twist
try:
    rospy.init_node('estop_race', anonymous=True, disable_signals=True)
    p = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(10)
    t = rospy.Time.now()
    while (rospy.Time.now() - t).to_sec() < $1 - 1 and not rospy.is_shutdown():
        p.publish(Twist()); r.sleep()
except Exception:
    pass" 2>/dev/null
}

full_stop () {
    echo "[race] ===== 全停开始(先零速流) ====="
    zero_stream 6
    pkill -9 -f "[a]bot_race_task" 2>/dev/null
    pkill -9 -f "[e]stop_race"     2>/dev/null
    for p in "[m]ove_base" "[a]mcl" "[m]ap_server" "[g]emini" "[i]mage_zoomer" \
             "[v]lm_detection" "[b]asic_xarm_grasp" "[g]g.py" "[r]plidar" \
             "[a]rduino_serial" "[e]kf_localization" "[l]aser_filter" \
             "[i]mu_filter" "[a]bot_base_node" "[s]tatic_transform" \
             "[a]pply_calib" "[t]f_echo" "[n]avigate.launch" "[v]l_locate.launch" \
             "[g]rasp.launch" "[b]ringup.launch" "[r]oslaunch" "[r]osmaster"; do
        pkill -9 -f "$p" 2>/dev/null
    done
    sleep 2
    zero_stream 3
    N=$(pgrep -c -f "[r]os" 2>/dev/null || true)
    echo "[race] ===== 全停完成,残留ROS进程: ${N:-0} ====="
}
trap 'echo "[race] 收到Ctrl-C"; full_stop; exit 0' INT TERM

start_stack () {
    PAT="$1"; NAME="$2"; shift 2
    if pgrep -f "$PAT" >/dev/null; then
        echo "[race] $NAME 已在跑,跳过"
    else
        run_show "setsid $@ >/tmp/race_$NAME.log 2>&1 &"
        sleep 4
    fi
}
start_stack "[b]ringup.launch"   bringup  roslaunch abot bringup.launch
start_stack "[n]avigate.launch"  navigate  roslaunch abot navigate.launch
start_stack "[v]l_locate.launch" vision    roslaunch vl_locate vl_locate.launch
start_stack "[b]asic_xarm_grasp" grasp     roslaunch ZachLab_grasp grasp.launch

echo "[race] 等 move_base 和 vlm_detection 上线(最多90s)..."
MB=0; VL=0
for i in $(seq 1 90); do
    MB=$(rostopic list 2>/dev/null | grep -c "/move_base/goal")
    VL=$(rosservice list 2>/dev/null | grep -c "vlm_detection")
    if [ "$MB" -ge 1 ] && [ "$VL" -ge 1 ]; then break; fi
    sleep 1
done
if [ "$MB" -lt 1 ] || [ "$VL" -lt 1 ]; then
    echo "[race] !! 栈没起齐(move_base=$MB vlm=$VL),查 /tmp/race_*.log"; exit 1
fi
run_show "rostopic pub -1 /servo riki_msgs/Servo \"{Servo1: 88, Servo2: 20}\""
echo "[race] 相机角度已置88/20,3秒后发车...(Ctrl-C 反悔)"
sleep 3

run_show "rosrun abot abot_race_task.py _fruits:=$FRUITS _rest_time:=3"
RC=$?
echo "[race] 任务结束(退出码$RC),收尾全停..."
trap - INT TERM
full_stop
