#!/bin/bash
# NoMachine 小车连接稳定性测试：N 轮 连接→保持→主动断开→验证清理
# 用法: bash nomachine_stability_test.sh [轮数]   (默认 3 轮, 每轮保持 300s)
# 可用环境变量覆盖: CAR_IP / NXS_FILE / HOLD_SECS / OUT_BASE / REMOTE_PASSWORD
# 依赖: scripts/ssh-car.py (车端取样), ss/awk/ping (本机监控)
# 经验教训(2026-10-17实测):
#   1) ss -tn 列序 State/Recv-Q/Send-Q/本地/对端 —— 对端 :4000 在 $5, 别写 $4
#   2) pgrep/pkill -f 'nxplayer' 会自匹配监控命令, 必须 pgrep -x nxplayer.bin
#   3) 车端 nxserver.log 混有二进制字节, grep 需加 -a
CYCLES=${1:-3}
CAR_IP=${CAR_IP:-192.168.43.8}
LOCAL_IP=${LOCAL_IP:-$(ip route get "$CAR_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')}
NXS_FILE=${NXS_FILE:-$HOME/桌面/小车.nxs}
HOLD_SECS=${HOLD_SECS:-300}
OUT_BASE=${OUT_BASE:-/tmp/nxtest}
REMOTE_PASSWORD=${REMOTE_PASSWORD:-123456}
REPO=$(cd "$(dirname "$0")/.." && pwd)

mkdir -p "$OUT_BASE"
log(){ echo "[$(date '+%H:%M:%S')] $*" | tee -a "$OUT_BASE/master.log"; }

car(){ (cd "$REPO" && CAR_HOST=$CAR_IP CAR_PASSWORD=$REMOTE_PASSWORD timeout 25 python3 scripts/ssh-car.py "$1" 2>/dev/null); }

nx_conns(){ ss -tn "dst $CAR_IP" 2>/dev/null | awk '$1=="ESTAB" && $5 ~ /:4000$/' | wc -l; }

kill_player(){
  pkill -TERM -x nxplayer.bin 2>/dev/null
  for _ in $(seq 1 15); do pgrep -x nxplayer.bin >/dev/null || return 0; sleep 2; done
  pkill -KILL -x nxplayer.bin 2>/dev/null
  return 1
}

one_cycle(){
  local C=$1
  local OUT="$OUT_BASE/cycle${C}"
  mkdir -p "$OUT"
  clog(){ echo "[$(date '+%H:%M:%S')] $*" | tee -a "$OUT/cycle.log"; }

  clog "=== 第${C}轮开始 (目标${CAR_IP}, 保持${HOLD_SECS}s) ==="
  car "echo $REMOTE_PASSWORD | sudo -S /usr/NX/bin/nxserver --list 2>/dev/null" > "$OUT/car_list_before.txt"
  clog "车端基线会话数: $(grep -c "$LOCAL_IP" "$OUT/car_list_before.txt" || true)"

  ping -i 1 "$CAR_IP" > "$OUT/ping.log" 2>&1 &
  local PING_PID=$!

  local T0=$(date +%s.%N)
  clog "启动 nxplayer ($NXS_FILE)"
  DISPLAY=:0 setsid /usr/NX/bin/nxplayer --session "$NXS_FILE" > "$OUT/player.out" 2>&1 &
  local PGID=$!

  local CONNECTED=0 i C
  for i in $(seq 1 60); do
    sleep 2
    if [ "$(nx_conns)" -gt 0 ]; then CONNECTED=1; break; fi
  done
  local T1=$(date +%s.%N)
  if [ $CONNECTED -eq 0 ]; then
    clog "失败: 120s 内未建立 TCP4000 连接"
    command -v import >/dev/null && DISPLAY=:0 import -window root "$OUT/fail_screenshot.png" 2>/dev/null
    kill -- -$PGID 2>/dev/null; kill_player
    kill -INT $PING_PID 2>/dev/null; sleep 1; kill -9 $PING_PID 2>/dev/null
    clog "=== 第${C}轮结果: FAIL(未连接) ==="
    return 1
  fi
  clog "TCP4000 已建立, 耗时 $(echo "$T1 $T0" | awk '{printf "%.1f", $1-$2}')s"

  local SESSION_OK=0
  for i in $(seq 1 12); do
    if car "echo $REMOTE_PASSWORD | sudo -S /usr/NX/bin/nxserver --list 2>/dev/null" | grep -q "$LOCAL_IP"; then SESSION_OK=1; break; fi
    sleep 5
  done
  if [ $SESSION_OK -eq 0 ]; then
    clog "警告: TCP 已通但车端 nxserver --list 60s 内未见会话"
  else
    clog "车端会话已注册, 累计耗时 $(echo "$(date +%s.%N) $T0" | awk '{printf "%.1f", $1-$2}')s"
  fi
  ps -C nxplayer.bin -o pid,etime,args > "$OUT/player_procs.txt" 2>/dev/null

  clog "开始保持 ${HOLD_SECS}s ..."
  local HOLD_START=$(date +%s) NEXT_CAR_CHECK=$(( $(date +%s) + 60 )) n=0
  while [ $(( $(date +%s) - HOLD_START )) -lt $HOLD_SECS ]; do
    sleep 30
    n=$((n+1))
    local ELAPSED=$(( $(date +%s) - HOLD_START )) CN=$(nx_conns)
    clog "采样#${n} t+${ELAPSED}s TCP4000连接数=$CN"
    if [ "$CN" -eq 0 ]; then
      clog "异常: 保持期间 t+${ELAPSED}s 连接断开!"
      break
    fi
    if [ "$(date +%s)" -ge $NEXT_CAR_CHECK ]; then
      car "echo $REMOTE_PASSWORD | sudo -S /usr/NX/bin/nxserver --list 2>/dev/null" | grep -E "$LOCAL_IP|^NX" > "$OUT/car_list_hold_${n}.txt" 2>/dev/null
      grep -q "$LOCAL_IP" "$OUT/car_list_hold_${n}.txt" 2>/dev/null || clog "警告: 车端列表此时刻无本机会话(t+${ELAPSED}s)"
      NEXT_CAR_CHECK=$(( NEXT_CAR_CHECK + 60 ))
    fi
  done
  local HELD=$(( $(date +%s) - HOLD_START ))

  clog "主动断开: 向 nxplayer 发 TERM"
  kill -- -$PGID 2>/dev/null
  sleep 2
  kill_player || clog "警告: TERM 后仍存活, 已 KILL"

  for i in $(seq 1 15); do [ "$(nx_conns)" -eq 0 ] && break; sleep 2; done
  local LOCAL_CLOSED=$(nx_conns)
  sleep 3
  car "echo $REMOTE_PASSWORD | sudo -S /usr/NX/bin/nxserver --list 2>/dev/null" > "$OUT/car_list_after.txt"
  local CAR_LEFT=$(grep -c "$LOCAL_IP" "$OUT/car_list_after.txt" || true)

  kill -INT $PING_PID 2>/dev/null; sleep 1; kill -9 $PING_PID 2>/dev/null; wait $PING_PID 2>/dev/null
  local PING_SUMMARY=$(tail -3 "$OUT/ping.log" | tr '\n' ' ')

  clog "保持时长: ${HELD}s (目标${HOLD_SECS}s) / 断开后本地TCP残留: $LOCAL_CLOSED / 车端残留会话: $CAR_LEFT"
  clog "ping: $PING_SUMMARY"

  if [ "$HELD" -ge "$HOLD_SECS" ] && [ "$LOCAL_CLOSED" -eq 0 ] && [ "$CAR_LEFT" -eq 0 ]; then
    clog "=== 第${C}轮结果: PASS ==="
    return 0
  fi
  clog "=== 第${C}轮结果: INCOMPLETE ==="
  return 2
}

PASS_N=0
for c in $(seq 1 "$CYCLES"); do
  log "########## 准备第${c}/${CYCLES}轮 ##########"
  one_cycle "$c"
  rc=$?
  [ $rc -eq 0 ] && PASS_N=$((PASS_N+1))
  [ $rc -eq 1 ] && { log "第${c}轮无法连接, 终止"; break; }
  [ $c -lt "$CYCLES" ] && { log "轮间隙 30s"; sleep 30; }
done
log "总结: ${PASS_N}/${CYCLES} 轮 PASS"
exit $(( CYCLES - PASS_N ))
