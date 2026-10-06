#!/bin/bash

# 硬编码比赛启动脚本：bringup/localize/vision/grasp 常驻，任务 runner 只发 cmd_vel。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

ACTION="start"
RUNNER_SCRIPT="$SCRIPT_DIR/hardcoded_race.py"
RUNNER_ARGS=""
DRY_RUN="0"
SERVO_HOME_1="88"
SERVO_HOME_2="20"

usage() {
    cat <<'EOF'
Usage:
  hardcoded_race_start.sh [start|brake|grasp-idle|rerun|status] [options]

start       启动 bringup/localize/VL locate/grasp，并启动硬编码 runner
brake       刹停底盘，不重启进程
grasp-idle  重置抓放状态到空闲态
rerun       只重跑 hardcoded_race.py
status      打印关键 ros 节点状态

Options:
  --nav-only                 跳过抓/放动作，只跑固定距离
  --dry-run                  只打印硬编码路线，不发 /cmd_vel
  --from-step <n>            从第 n 个任务继续
  --speed <mps>              线速度，默认由 hardcoded_race.py 决定
  --x-scale <k>              x 方向时间补偿
  --y-scale <k>              y 方向时间补偿
  --settle-wait <seconds>    每段刹停后的等待
  --action-wait <seconds>    抓/放服务调用后的等待
  --runner-script <path>     指定 hardcoded_race.py 路径
  --servo1 <value>           回零舵机 Servo1（默认 88）
  --servo2 <value>           回零舵机 Servo2（默认 20）
  -h, --help                 显示帮助

Examples:
  hardcoded_race_start.sh
  hardcoded_race_start.sh --nav-only
  hardcoded_race_start.sh rerun --from-step 6
  hardcoded_race_start.sh brake
EOF
}

log() { echo "[hardcoded_race_start] $*"; }

append_runner_arg() {
    RUNNER_ARGS="${RUNNER_ARGS} $1"
}

run_cmd_in_tab() {
    local title="$1"
    local cmd="$2"
    local safe_title
    safe_title="$(echo "$title" | tr -c 'A-Za-z0-9._-' '_')"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --tab --title="$title" -- bash -c "$cmd; exec bash"
        return 0
    fi

    local log_file="/tmp/hardcoded_race_${safe_title}.log"
    nohup bash -lc "$cmd" >"$log_file" 2>&1 < /dev/null &
    log "no gnome-terminal, start in bg: $title -> $log_file"
}

publish_servo_home() {
    rostopic pub -1 servo riki_msgs/Servo "Servo1: ${SERVO_HOME_1} Servo2: ${SERVO_HOME_2}" || true
}

send_cmd_vel_zero() {
    local msg='{
      linear: {x: 0.0, y: 0.0, z: 0.0},
      angular: {x: 0.0, y: 0.0, z: 0.0}
    }'
    for _ in 1 2 3; do
        rostopic pub -1 /cmd_vel geometry_msgs/Twist "$msg" || true
    done
}

emergency_brake() {
    log "执行底盘刹车"
    send_cmd_vel_zero
    rosparam set /grab false || true
    rosparam set /success false || true
    publish_servo_home
    log "刹车动作完成"
}

run_launches() {
    log "启动 bringup（底盘/IMU/EKF）"
    run_cmd_in_tab "Bringup" "roslaunch abot bringup.launch"
    sleep 5

    log "启动 localize（只做定位观察，不依赖 move_base）"
    run_cmd_in_tab "Localize" "roslaunch abot localize.launch"
    sleep 4

    log "启动 VL locate（低优先：nice 10）"
    run_cmd_in_tab "VL_Locate" "source ~/.bashrc; conda activate 39; nice -n 10 roslaunch vl_locate vl_locate.launch"
    sleep 2

    log "启动 grasp（低优先：nice 10）"
    run_cmd_in_tab "Grasp" "nice -n 10 roslaunch ZachLab_grasp grasp.launch"
    sleep 5

    log "发送舵机初始位置"
    publish_servo_home
}

stop_mission_runner() {
    local pids
    pids="$(pgrep -f "hardcoded_race.py" || true)"
    if [ -n "$pids" ]; then
        log "停止旧 hardcoded runner: $pids"
        kill $pids || true
        sleep 1
        pids="$(pgrep -f "hardcoded_race.py" || true)"
        if [ -n "$pids" ]; then
            kill -9 $pids || true
        fi
    fi
}

run_mission_runner() {
    if [ ! -f "$RUNNER_SCRIPT" ]; then
        log "runner script not found: $RUNNER_SCRIPT"
        return 1
    fi
    if [ "$DRY_RUN" = "1" ]; then
        log "dry-run：只打印硬编码路线，不启动后台 runner"
        python2 "$RUNNER_SCRIPT" ${RUNNER_ARGS}
        return 0
    fi
    stop_mission_runner
    log "启动硬编码 runner：python2 ${RUNNER_SCRIPT}${RUNNER_ARGS}"
    run_cmd_in_tab "HardcodedRace" "python2 '$RUNNER_SCRIPT' ${RUNNER_ARGS}"
}

reset_grasp_state() {
    log "执行 Grasp 空闲重置"
    send_cmd_vel_zero
    publish_servo_home
    rosparam set /grab false || true
    rosparam set /success false || true
    rosparam set /grasp_reset true || true
}

show_status() {
    log "rosnode 简况"
    if rosnode list 2>/dev/null | grep -q "/hardcoded_race"; then
        log "hardcoded_race: running"
    else
        log "hardcoded_race: not running"
    fi
    if rosnode list 2>/dev/null | grep -q "/basic_xarm_grasp"; then
        log "basic_xarm_grasp: running"
    else
        log "basic_xarm_grasp: not running"
    fi
    if rosnode list 2>/dev/null | grep -q "/vl_locate"; then
        log "vl_locate: running"
    else
        log "vl_locate: unknown"
    fi
}

if [[ "$#" -gt 0 ]]; then
    case "$1" in
        start|brake|grasp-idle|rerun|status)
            ACTION="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --*)
            :
            ;;
        *)
            log "unknown action: $1"
            usage
            exit 1
            ;;
    esac
fi

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --dry-run)
            DRY_RUN="1"
            append_runner_arg "$1"
            shift
            ;;
        --nav-only|--skip-waits)
            append_runner_arg "$1"
            shift
            ;;
        --from-step|--speed|--x-scale|--y-scale|--settle-wait|--action-wait)
            append_runner_arg "$1 ${2:?$1 needs value}"
            shift 2
            ;;
        --runner-script)
            RUNNER_SCRIPT="${2:?--runner-script needs value}"
            shift 2
            ;;
        --servo1)
            SERVO_HOME_1="${2:?--servo1 needs value}"
            shift 2
            ;;
        --servo2)
            SERVO_HOME_2="${2:?--servo2 needs value}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log "unknown arg: $1"
            usage
            exit 1
            ;;
    esac
done

case "$ACTION" in
    start)
        if [ "$DRY_RUN" != "1" ]; then
            run_launches
        fi
        run_mission_runner
        ;;
    brake)
        emergency_brake
        ;;
    grasp-idle)
        reset_grasp_state
        ;;
    rerun)
        run_mission_runner
        ;;
    status)
        show_status
        ;;
    *)
        usage
        exit 1
        ;;
esac
