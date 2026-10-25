#!/bin/bash
# TF卡重刷/系统重建后的一键工作区恢复（在车上执行）
# 用法：bash restore_car.sh <car_bundle_*.bundle>
# 效果：备份旧 ~/catkin_ws → 从 bundle 克隆新工作区 → 5个GPU依赖包CATKIN_IGNORE → catkin_make -j1
# 前提：车上已装好 ROS Melodic desktop + libopencv-dev 3.2 + python-serial 等基础依赖
set -euo pipefail

BUNDLE=${1:?用法: bash restore_car.sh <car_bundle_*.bundle>}
WS=$HOME/catkin_ws
STAMP=$(date +%Y%m%d-%H%M%S)
REMOTE_URL="https://github.com/zhangdashuai968/small-car.git"

# 这5个包依赖 jetson-inference/TensorRT/astra SDK（新系统未装），先跳过编译
IGNORE_PKGS=(
  src/abot_project/abot_deep_learning
  src/abot_project/abot_yolo_detection
  src/abot_project/abot_object_trtdetect
  src/abot_project/abot_traffic_trtdetect
  src/depth_camera/ros_astra_camera
)

[ -f "$BUNDLE" ] || { echo "恢复包不存在: $BUNDLE"; exit 1; }

if [ -d "$WS" ]; then
  echo "[1/5] 备份现有工作区: $WS -> $WS.bak-$STAMP（含旧 build/devel，确认无用后可删）"
  mv "$WS" "$WS.bak-$STAMP"
fi

echo "[2/5] 恢复工作区到 $WS ..."
case "$BUNDLE" in
  *.bundle)
    git clone --quiet "$BUNDLE" "$WS"
    git -C "$WS" remote set-url origin "$REMOTE_URL" ;;
  *.tar.gz|*.tgz)
    mkdir -p "$WS" && tar -xzf "$BUNDLE" -C "$WS"
    echo "    （tar 快照模式：无 .git，网络好时可 git clone 重取完整历史）" ;;
  *) echo "不支持的包类型（需 .bundle 或 .tar.gz）"; exit 1 ;;
esac

echo "[3/5] 给 5 个 GPU 依赖包打 CATKIN_IGNORE ..."
for p in "${IGNORE_PKGS[@]}"; do touch "$WS/$p/CATKIN_IGNORE"; done

echo "[4/5] catkin_make -j1（必须单线程，-j2 会撞消息生成竞态；约10-30分钟）..."
cd "$WS"
source /opt/ros/melodic/setup.bash
catkin_make -j1

echo "[5/5] 恢复完成。接下来按顺序验证："
cat <<'EOF'
a) 插上底盘/雷达USB，看串口映射：ls -l /dev/abotbase /dev/abotlidar /dev/ttyACM* /dev/ttyUSB*
   （怀疑映射反了：裸读两口，谁流数据谁是底盘 rosserial；反了改 /etc/udev/rules.d/99-abot.rules
    的两行 SYMLINK，再 sudo udevadm control --reload && sudo udevadm trigger）
b) 启动：bash -ic 'roslaunch abot bringup.launch'
   另开终端：bash -ic 'roslaunch abot localize.launch'
c) 最小验证：rosnode list; rostopic hz /scan /odom; rosrun tf tf_echo map base_footprint
d) 本脚本不覆盖的遗留项：vl_locate 的 conda py39 环境、server.py 的 DashScope API key 换新、
   串口映射实车确认（见 WORKLOG 第四节）
EOF
