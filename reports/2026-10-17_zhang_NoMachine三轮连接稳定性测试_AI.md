# 2026-10-17 zhang NoMachine 三轮连接/断开稳定性测试（AI 辅助）

## 最终状态：3/3 轮 PASS，NoMachine↔小车链路稳定性验证通过，无需任何配置改动

## 任务与结论（TL;DR）
用户要求：调试好 NoMachine 确保能稳定连接小车，并做 3 次"连接保持 5 分钟后断开"测试。
结果：**预检全绿（无故障可修），三轮测试全部通过**。客户端 2s 建连、5.9-7.0s 完成登录注册会话；每轮保持 323-329s（服务端口径 5m28s-5m43s），30s 间隔采样共 30 次无一断开；主动断开后本机 TCP/进程零残留、车端会话零残留；车端日志三轮零 ERROR、零 crash，logout 均在断开后 0-10s 内干净落账。

## 环境
- 本机：Ubuntu, NoMachine 客户端 9.6.3-1 amd64, IP 192.168.43.12（热点网段 192.168.43.0/24, 无线网卡 wlxe865d42703ac）
- 小车：Jetson Nano, Ubuntu 18.04.6, NoMachine 服务端 8.19.2, IP **192.168.43.8**（MAC b4:6b:fc:32:4c:95 扫描定位；今日 10:36:48 开机, nxserver 随启动 active, 4000 端口监听）
- 连接方式：桌面 `小车.nxs` → `robot.local`(mDNS 解析正常) → NX 协议 TCP4000, abot + NX-password（密码已记住）, 证书已预信任不弹确认
- 本机与小车时钟偏差 < 1s，双端日志时间戳可直接对齐

## 预检（全部通过，未做任何修复）
robot.local 解析 → 192.168.43.8；双端 nxserver active；车端无残留会话、无卡死 `nxserver --login` 进程（10-15 的故障模式未复发）；本机 hosts.crt 含 robot.local 与 192.168.43.8 条目。

## 测试方法
自动化脚本（已存 `scripts/nomachine_stability_test.sh`）：每轮 = 基线车端会话快照 → 启动 `nxplayer --session 小车.nxs`(DISPLAY=:0) → 等 TCP4000 ESTAB + 车端 `nxserver --list` 注册 → 保持 300s（本机 30s 采样 TCP 连接数 + 1Hz ping；车端 60s 采样会话列表）→ 对 nxplayer 进程组发 TERM 断开 → 验证本机 TCP/进程与车端会话均清理干净。

## 三轮结果汇总

| 轮次 | 车端登录→登出(服务端日志) | TCP建连 | 保持采样 | 断开后残留(本机TCP/车端会话) | 丢包/延迟(1Hz ping) |
|---|---|---|---|---|---|
| 1 | 11:06:30 → 11:11:58 (5m28s) | 2.0s | 10/10 恒=1 | 0 / 0 | 6.8%, avg 161ms, 峰 1263ms |
| 2 | 11:12:43 → 11:18:11 (5m28s) | 2.0s | 10/10 恒=1 | 0 / 0 | 0.59%, avg 105ms, 峰 996ms |
| 3 | 11:18:57 → 11:24:40 (5m43s) | 2.0s | 10/10 恒=1 | 0 / 0 | 7.1%, avg 151ms, 峰 1558ms |

服务端交叉验证（`grep -a '2026-10-17 11:' nxserver.log`）：三轮各一组干净的 Connected→logged in(NX-password)→logged out→disconnected，窗口内**零 ERROR / 零 Encryptor 错误 / 零 crash**。注：11:04:44 有一条 `Encryptor context not received`，是调试 ss 语法时的裸 TCP 探针（未发客户端 hello 被服务端超时关闭，NX 协议预期行为），与真实客户端无关。

## 过程中踩的坑（已修，供复用）
1. **ss 列序**：`ss -tn` 输出为 State/Recv-Q/Send-Q/本地/对端，检测"到车端 4000 的连接"应匹配 `$5`（对端）而非 `$4`（本地临时端口）——首轮因此误报 120s 超时并误杀一条好连接（车端日志证实当时已登录成功）。
2. **pgrep 自匹配**：`pgrep -f 'nxplayer'` 会匹配到含该字样的监控命令自身，必须 `pgrep -x nxplayer.bin` 按进程名精确匹配。
3. **车端日志二进制字节**：`nxserver.log` 混有二进制数据，grep 默认判 binary 吞输出，需 `grep -a`。
4. 时钟偏差 <1s 使双端日志可直接对齐； abruptly kill 的客户端服务端约 42s 后才落账 logout，TERM 干净退出则 0-10s 内落账。

## 稳定性评估
- 可靠性：3/3 全通过、30 次采样零断开、双端零残留、服务端零错误 → **连接与断开行为可靠**。
- 链路质量：丢包 0.59%-7.1%、平均延迟 105-161ms、峰值到 1.5s（2.4G 热点与校园网 AP 同频干扰，10-15 报告已定性）——NX 会话在 7% 丢包下仍持续存活不掉线，但远程桌面操作流畅度受此限制。改善方向不变：热点换 5GHz/换位、画质调低。

## 交付物
- `scripts/nomachine_stability_test.sh`：可复用的稳定性回归测试脚本（支持 CAR_IP/NXS_FILE/HOLD_SECS 环境变量覆盖；`bash nomachine_stability_test.sh [轮数]`）。
- `logs/小车调节日志.txt`：追加 10:44-11:26 五条记录。
- 测试原始数据：`/tmp/nxtest/cycle{1,2,3}/`（cycle.log / ping.log / 车端会话快照）。

## 下一步交接
1. 日常使用：桌面双击 `小车.nxs` 即连（robot.local 免改 IP、免弹密钥确认）。
2. 复发"卡连接中"时：先查车端 `sudo grep -a 'Encryptor\|ERROR' /usr/NX/var/log/nxserver.log | tail`，再按 10-15 报告的顺序重启车端/本机 nxserver。
3. 链路卡顿（非掉线）属 WiFi 侧问题，见 10-15 报告结论，无需动 NoMachine。
4. 本次新增报告+脚本，建议 commit + push（有意义改动）。

## 环境/状态记录
- git SHA（本次开始时）：30b1680（工作区另有导航调试未提交改动，属另一条线，未动）
- 车端状态：nxserver active / 4000 监听 / 无会话残留；nxnode.pid 7827 为开机物理显示节点（正常）
- 本次未发运动命令、未改 ROS 参数、未改双端 NoMachine 任何配置；小车急停按钮状态未涉及
