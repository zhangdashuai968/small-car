# 2026-10-15 NoMachine 连接故障排查（AI 辅助）

## 最终状态：21:09 连接成功，问题完整闭环
robot.local 方式连接：21:09:09 握手成功（无 Encryptor 错误）、21:09:11 abot NX-password 认证通过，会话存活（车端 nxnode 持续运行、零错误）。桌面"小车.nxs"双击即连，目标为 mDNS 主机名 robot.local，换热点换 IP 均无需改配置；小车证书已预信任（robot.local 条目），不会再弹密钥确认。客户端侧已关闭 WebRTC 与 UDP 多媒体传输以适配热点网络。

## 背景
用户反馈本机 NoMachine 客户端连不上小车（Jetson Nano, 当前 IP 10.194.233.46，WiFi 网段 10.194.233.0/24）。SSH 正常。

## 结论（TL;DR）
不是网络/防火墙问题。车上 NoMachine 服务端（8.19.2）握手状态卡死，本机 4 次连接全部死在握手阶段（服务端日志 `Encryptor context not received` 120s 超时 ×3、`Invalid client identification '(empty)'` ×1）；其中一次密钥确认点晚了，客户端挂在已被服务端断开的死连接上不再重试。**已在小车重启 nxserver（19:46 完成），服务端已恢复；本机 kill 了卡死的会话进程。重新连接即可。**

## 排查过程
1. 本机 `10.194.233.247`（wlxe865d42703ac），扫 /24 找到小车 `10.194.233.46`（MAC b4:6b:fc:32:4c:95，与 hosts.crt 中记录一致）。
2. TCP 探测：22、4000 端口均可达 → 排除网络层。
3. 车上 `systemctl status nxserver`：active（19:28 随开机启动），nxd 监听 0.0.0.0:4000；但存在 99% CPU 的残留 `nxserver --login` 进程。
4. `/usr/NX/var/log/nxserver.log` + `nxd.log`：
   - 19:36:02 / 19:39:53 / 19:40:52 三次来自本机的连接，全部握手失败（120s 超时或空标识），昨晚 22:01 同样症状。
   - nxd.log 19:42:52（最后一次连接被服务端超时关闭）→ 19:46:01（重启）之间零新连接，而本机 hosts.crt 于 19:43:23 才写入 10.194.233.46 证书、player.cfg 19:43:27 更新"Last connection time" → 用户点密钥确认时连接已死，客户端挂死不重连。
5. 版本：本机客户端 9.6.3（amd64，含本机 nxserver 服务），车上服务端 8.19.2（2026-06-26 升级）。官方 KB 声明 v9 客户端兼容 v8 服务端（见参考），暂判非根因，但列为复发时的升级候选。

## 已执行变更
- 小车：`sudo /usr/NX/bin/nxserver --restart`（经 scripts/ssh-car.py，19:46:09 完成，`systemctl is-active` = active）。未动 ROS、未动车其它配置。
- 本机：kill 卡死的 `nxplayer --session` 进程（pid 5817）；保留主窗口。未改本机任何配置。

## 验证
- 重启后车端 nxd 正常监听 4000，Redis 正常启动，无残留告警。
- 本机 TCP 连 4000 成功（服务端静默等待客户端 hello，符合 NX 协议预期）。
- **修复已实锤**：车端 19:52-19:54 仍有一次失败（坐实本机侧问题）→ 本机 nxserver --restart（20:07，用户授权 sudo）→ **20:10:32 用户实连成功：握手通过、abot NX-password 认证通过，会话保持 2分46秒（20:13:19 客户端正常登出，服务端无崩溃）**。
- 稳定性测试（20:25）：丢包 0%（100+ 包）、TCP 4000 建连 15/15、SSH 持续流 30s 零停顿 → 可靠性 OK。延迟质量差：ping 车 avg 31-101ms / 峰值 346-805ms / 有重复包；ping 热点 avg 35ms / 峰 446ms → 瓶颈在本机↔热点段。根因：手机热点（Redmi Note 14 Pro，2.4G ch2472）紧邻 90-92% 信号校园网 AP（ch2462）同频干扰。车端 wlan0 power_save 已临时关闭（重启不持久），A/B 效果边际，非主因。

## 突发：20:25 小车整机掉 WiFi —— 已定论（20:45）：热点被切换，非小车故障
- 时间线：20:24 本机最后一次成功 ping/SSH；20:25:33 起小车在旧热点(Redmi 2.4G 10.194.233.x)完全失联；本机随后也被切到新热点（172.20.10.0/28，iOS 风格）。
- 定论证据：小车 `uptime -s` = 19:28:35（从未重启）；NetworkManager 日志显示 20:40:19 自动加入新热点（该 SSID 为小车已保存网络，无需新密码）拿到 `172.20.10.4`。即旧热点整体消失导致断开，**20:23 关闭 power_save 的嫌疑解除**。
- 已处理：20:44 在新 IP 重启 nxserver（active、4000 监听）。车端信号 -38dBm；power_save 随重关联回到默认 on，保持不动。
- 注意事项：NoMachine 客户端保存的连接仍指向旧 IP `10.194.233.46`，需改/新建为 `172.20.10.4`；新 IP 首连会再弹主机密钥确认，需尽快点确认（服务端握手 120s 超时的坑已踩过一次）。IP 查找以 MAC `b4:6b:fc:32:4c:95` 扫描为准（记忆库已更新）。

## 下一步交接
1. 用户重开 NoMachine → 连 `10.194.233.46`（abot/123456），密钥已信任不会再弹确认。
2. 若再次卡"连接中"：查车端 `nxserver.log` 尾部是否再现 `Encryptor context not received`；是则重启**本机** nxserver：`sudo /usr/NX/bin/nxserver --restart` 后重试。
3. 仍复发则考虑版本对齐：车上装 NoMachine 9 arm64 deb（Ubuntu 18.04），或本机降 8.19.2 客户端。
4. 若连上但黑屏/会话崩：关注车端每次开机都有的 `Session ... crashed on display 0`（Jetson 无显示器时的物理显示会话问题），届时再查。

## 补充：赴官网获取低版本（8.19.2）结论——官方已无旧版包
- 已穷尽验证：下载中心仅提供 10.2.3（包名改为 nomachine-personal-edition）；旧版直链（8.19.2/9.6.3/8.14.2 × amd64/arm64）一律 302 回主页；无 older/archive 归档页（404）；WP REST API 401；web9001 镜像同策略，web7551/web01 已下线；无 FTP；Wayback 从本网络不可达。
- 可行替代：① 小车 bash 历史显示当时手动装过 `nomachine_8.14.2_1_arm64.deb`（装后删除）——团队 Windows 电脑/U盘可能有 amd64 版，找到即可降级（`~/下载/nomachine_9.6.3_1_amd64.deb` 保留可回退）；② 联系 NoMachine 支持索取旧版。
- 工程判断：当前 9.6.3 客户端 ↔ 8.19.2 服务端连接正常（21:45:47 经 robot.local 重连成功），卡顿/掉线主因在 WiFi 侧，版本对齐预期收益有限。

## 补充：本机旧版安装包搜索（用户要求不上官网、只找本地包倒退版本）
- 结论：**本机不存在旧版安装包，也没有可退的旧版本**。
- dpkg 全部历史日志（含 gz 轮转）：NoMachine 唯一一次安装是 `2026-06-02 16:50:08 install nomachine:amd64 <none> 9.6.3-1`——首次安装即 9.6.3，此前无任何 8.x。
- 全盘 find（排除 /usr/NX、/var/NX、~/.nx）、回收站、apt 缓存：唯一安装包为 `~/下载/nomachine_9.6.3_1_amd64.deb`（即当前版本）。
- `/media/nomachine` 为 6月2日安装时段的 U 盘挂载点残留（空目录，盘未插）。未挂载分区 sda1（卷标 Z，ext4）经 udisksctl 挂载检查后只有 lost+found，已卸载恢复原状。
- 车上也没有安装包残留，且车为 arm64、本机 amd64，架构不通用。
- 旁证：本机 hosts.crt 含旧网段车端证书（192.168.43.211 / 10.66.73.46 / 192.168.43.8），说明本机 9.6.3 客户端历史上与车端 8.x 握手成功过——版本组合本身可用，回退版本非必要路径。

## 参考
- NoMachine KB: client v9 ↔ server v8/7 兼容 — https://kb.nomachine.com/DT04U00280 、https://kb.nomachine.com/SU05W00266
- 论坛同症状帖 — https://forum.nomachine.com/archive/topic/encryptor-context-not-received-error

## 环境/状态记录
- git SHA（本次排查开始时）：30b1680（工作区有未提交的导航调试改动，属另一条线，未动）
- 小车：Ubuntu 18.04.6 / ROS Melodic，IP 10.194.233.46，NoMachine server 8.19.2
- 本机：Ubuntu（Linux），NoMachine 9.6.3-1 amd64
- 本次未发运动命令、未改 ROS 参数；小车急停按钮状态未涉及
