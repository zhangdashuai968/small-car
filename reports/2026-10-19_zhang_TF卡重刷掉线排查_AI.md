# 2026-10-19 zhang TF卡重刷"卡住"排查（AI 会话报告）

## 背景与目标
原小车系统 SD 卡硬件损坏（见 2026-10-18 日志 [17:32]），走恢复路线 B：新 64G TF 卡在 Windows 上重刷 Jetson Nano 官方镜像（Ubuntu 18.04.6）。刷机过程中工具"卡住"，TF 卡拿到本 Linux 桌面机（USB 读卡器）排查。

## 结论（TL;DR）
1. **刷写其实已经写完了，卡上镜像完好可用**——直接上机开机即可（用户已执行）。
2. **"卡住"的根因：廉价 USB2.0 读卡器（Generic 14cd:125d）撑不住长时间连续传输**。整卡读取测试在连续传输 17.7 分钟 / 20.4GB 后，读卡器整体从 USB 总线掉线。刷镜像 + 校验需要 25 分钟以上连续传输，必然中途掉线，表现为刷机工具卡死。
3. 卡体 20.4GB 之后区域未验证（读卡器掉线，无法区分是卡有坏区还是读卡器崩溃）。镜像只占前 13.8GB，不影响本次开机使用；若后续要复刷或校验，换 USB3.0 品质读卡器。

## 诊断过程与证据
- 设备：`/dev/sdc` 64G（59.5GiB），读卡器 `usb 1-5`（idVendor 14cd:125d，USB 2.0 high-speed）。
- 分区表：Jetson Nano 标准 14 分区（sdc1=APP 12.9G ext4 + 13 个 128K~768K Tegra 分区），Tegra 分区数据非零 → 镜像尾部已写入。
- sdc1 挂载成功（ext4 journal recovery complete），内容为全新未开机官方镜像：Ubuntu 18.04.6、hostname 空、/home 空、根目录带 NVIDIA "sample filesystem" README.txt。
- 本机无刷写进程/无刷写历史 → 刷机在 Windows 侧完成，卡仅拿来检查。
- 整卡读取测试（`dd if=/dev/sdc of=/dev/null bs=4M`，19.3MB/s）：
  - 0 ~ 13.8GB（镜像区域）：完整读通，零错误 → **镜像数据完好**。
  - 20.4GB 处（连续 1060s 后）：`I/O error, dev sdc, sector 39933632` → `usb 1-5: USB disconnect` → `hostbyte=DID_ERROR` / `device offline` → 读卡器从 lsusb 消失，须物理重插。
  - 掉线时挂载的 ext4 被强制卸载、journal superblock 写丢失——下次挂载会再做一次日志恢复，无害。
- 全程 dmesg 在掉线前无任何 CRC / reset / timeout：不是线缆瞬断，是持续负载下设备崩溃。

## 硬件状态
- 原 SD 卡：物理损坏，已退役。
- 新 64G TF 卡：镜像区（前 13.8GB）验证完好；20.4GB 后未验证。已装入小车。
- 读卡器：廉价 Generic USB2.0，长时间连续传输必掉线，淘汰出刷机流程。

## 下一步（恢复路线 B）
1. 小车首次开机过 OEM 配置：显示器 + USB 键盘设用户/密码/hostname，连 WiFi。
2. SSH 通后由本机接手：装 ROS Melodic、按仓库恢复 catkin_ws（src/abot_project 及依赖）、WiFi/自启动项重配。
3. 最小验证：`roslaunch abot bringup.launch`、`rostopic hz /scan`、`tf echo map base_footprint`。

## Git
- 诊断基于 main @ 30b1680；本次仅追加日志与本报告，无代码/参数变更。
