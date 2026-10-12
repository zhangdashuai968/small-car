# vendor_official/ — 万创鑫诚官方标准包（勿混入真机代码）

## 这是什么

`vl_locate` + `ZachLab_grasp` 两个包的**官方干净底版**（21 文件），出自 2026 万创鑫诚第二次线上培训资料 `arm/视觉抓取完整版.zip`，2026-10-12 入库。同目录附带与真机的逐文件比对报告。

## ⚠️ 和真机的关系（重点）

**真机在用的加强版在 `src/abot_project/` 下（main / car-snapshot 分支），本目录只是官方基线存档，两边勿混、真机勿回退。**

2026-10-12 逐文件比对结论（vs GitHub `car-snapshot-20261006` 分支）：

- 7 个文件逐字节一致：client.py、GetObject.srv、gg.py、xarm_driver.py、grasp.launch、ZachLab 两个清单、SimHei.ttf
- 10 个文件真机是增强版：server.py（JSON 容错提取、bbox 0-1000 归一化、空结果降级）、resize.py（py3 重写 + camera_info 同步 + 内参 K/P 变换）、Detection.msg（加 bbox 字段）、vl_locate.launch（runner 移出、include rgbd_localizer）、CMakeLists/package.xml（rgbd 依赖）、ZachLab_params.yaml（自标定值，官方是教材示例值）

## 用途

1. **新比赛 runner 的蓝本**：`vl_locate/scripts/client.py` 是官方"输入 1 启动"原型——goto 带 60 秒超时+取消、`/grab` 参数 + 轮询 `/success` 握手、麦轮原语 right() 横移 / end() 倒车 / rotate() 原地转、航点 `~goalListX/Y/Yaw` 参数化。已知缺陷：轮询无 sleep、goto 恒返回 True、类别词写死 demo 残留。
2. 出问题时对照"官方原本长什么样"。

## 备注

- 官方 `vlm_params.yaml` 的 api_key 是空的——key 应运行时注入；真机 server.py 默认值里内嵌的旧 key 待作废轮换时清除。
- 识别图像官方素材 = 9 种水果（苹果/榴莲/西瓜/山竹/梨/火龙果/葡萄/香蕉/红枣），在培训资料 `识别图像.rar`，未入库。
- `m1-s/abot-M1-S.zip` 是另一台 Noetic 机器（~/abot 工作区，yolov8_ros/robot_slam 栈）的完整工作区，与本项目 Melodic/catkin_ws 不同栈，未入库，留在 D 盘培训资料里。
