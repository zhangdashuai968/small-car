# 2026-10-15 small_car 工程备份到新 Windows 电脑（AI 辅助）

## 背景
用户新接入一台 Windows 电脑到同一 WiFi（10.194.233.0/24），要求把 small_car 的主要工程文件和系统文件复制到该电脑**桌面**（参照旧 Windows 机 `C:/Users/zhang666/Desktop/small_car` 的布局，见 `scripts/sync-once.bat`）。

## 结论（TL;DR）
新 Windows 电脑为 `10.194.233.82`（MAC 10:91:d1:6b:cc:a9，Intel 无线网卡）。其防火墙拦截了 SMB(445)/SSH(22)/RDP(3389)/ICMP，仅开放 2 个私有协议高位端口（27036/55436，疑似 IM/网盘类软件监听），**没有任何可供本机直接推送的通道**。因此改为"打包 + 局域网 HTTP 下载"方案：

- 备份包：`~/桌面/small_car_backup_20261015/small_car_backup_20261015.zip`（272MB，2396 个文件，`unzip -t` 通过）
- 下载服务：本机 `python3 -m http.server 8000`（后台，随本会话存活），目录内附 `Windows下载说明.txt`
- 新电脑操作（二选一）：
  1. 浏览器打开 `http://10.194.233.247:8000/` 点 zip 下载，解压到桌面；
  2. PowerShell 一条命令直接下载到桌面并解压：
     `$d=[Environment]::GetFolderPath('Desktop'); curl.exe -o "$d\small_car_backup_20261015.zip" http://10.194.233.247:8000/small_car_backup_20261015.zip; Expand-Archive "$d\small_car_backup_20261015.zip" "$d" -Force`

## 过程
1. 本机 `10.194.233.247`（wlxe865d42703ac），网关 `.33`（本地管理 MAC，疑似手机热点），小车 `.46`。
2. TCP 扫描 /24 常见端口：仅小车（22/4000）与本机；ping 清扫后 ARP 表发现唯一未知主机 `.82`。
3. `.82` 全 65535 端口扫描：仅 27036/55436 开放；HTTP/TLS/被动 banner 探测均为私有二进制协议；NBSTAT(UDP137) 与 mDNS 无响应；二次 ping 无响应（防火墙拦 ICMP，符合 Windows 公用网络默认）。OUI 查询确认 10:91:D1 = Intel Corporate → 判定为新 Windows 电脑。
4. 打包：`zip -r` 整个工作区，排除 `.git/`（274MB 历史，GitHub origin 已有全量提交记录）、`__pycache__`、`*.pyc`、`src/out/`（VSCode CMake 构建产物）。含未提交的导航调试改动（工作区当前状态）与未跟踪的 `abot_arm_learning/`、`AGENTS.md`、`reports/` 两份新报告。

## 完整性
- SHA256：`18f92a106f0f94cf1d8bc162e97380570524382648993bd5662085220625ff48`
- `unzip -t` 通过；HTTP HEAD 返回 200，Content-Length 285272905。

## 已执行变更
- 新增（工作区外，不影响 git status）：`~/桌面/small_car_backup_20261015/`（zip + 说明 txt）。
- 新增本报告（仓库内唯一改动）。
- 后台进程：http.server 8000。未动 ROS、未动车、未改仓库任何源文件。

## 验证
- 本机 curl 经 127.0.0.1 与 10.194.233.247 访问 8000 端口均 200（ufw 不活动，无入站拦截）。
- **新电脑实际下载验证：待办**（需用户在新电脑上执行上述任一方式后确认，可选 Get-FileHash 对比 SHA256）。

## 下一步交接
1. 用户在新 Windows 电脑按方式一/二下载并解压到桌面；完成后可让我关掉 8000 服务并清理临时目录。
2. 若 8000 链接失效（本会话结束/服务被杀），叫我重新起服务即可（一条命令）。
3. 若希望以后能从本机**自动推送**文件到该 Windows 电脑：在其上启用 OpenSSH Server（设置→应用→可选功能）或开启 SMB 共享并告知账号，即可用 scp/smbclient 直传。
4. git 历史未包含在备份内，需要时在新电脑 `git clone https://github.com/zhangdashuai968/small-car.git`，再用本备份覆盖未提交改动。

## 环境/状态记录
- git SHA（本次开始时）：30b1680；工作区仍有未提交导航调试改动（M move_base_waypoint_runner.py / waypoints.yaml / dwa 参数 / 调节日志），原样打包，未提交。
- 小车 `.46` 未触碰；未发运动命令；急停按钮状态未涉及。
