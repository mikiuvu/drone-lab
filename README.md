# Drone Lab 桌面算法实验环境

双击 **Drone Lab 实验台.lnk** 启动，进入“⑤ 从这里开始”阅读操作说明。

本项目用于 DM-FC01、LicheeRV Nano 和两颗 VL53L5CX 的桌面采集与调试。

- GUI：双 8×8 热力图、姿态显示、接收速率、异常提示、演示数据。
- 连接：电脑串口接收 MAVLink；Nano SSH 检查、SFTP 部署及采集。
- 日志：JSONL 录制、暂停/倍速/拖动回放、区域明细 CSV 导出。
- 开发：VS Code 图形调试配置、WSL RISC-V 交叉编译、测试。
- 软件：QGroundControl、MobaXterm、Rufus、官方 Nano 镜像与 DM-FC01 固件。

详细说明见 docs/quickstart.html。硬件尚未连接验证；镜像写卡、固件刷写和具体接线由用户核对后操作。
当前为采集工具，不提供 SLAM、自动飞行或物理飞行仿真。

## 预研与电脑仿真

打开实验台“⑥ 预研与仿真”，可运行有界图像队列/CPU与NPU调度、双ToF三维射线与局部可观测性、时间错配与接口门控。
docs/research/index.html 为预研总览，说明算力判断、最终模块划分、风险和板上验证门槛。
每次仿真在独立进程执行并保存配置、模型源码摘要和JSON/PNG结果；板上性能需通过对应硬件测量。
命令行重现入口为 scripts/run_research.py，研究源码在 research，测试在 tests。

## 开发入口

- app/main.py：界面。
- app/workers.py：串口与 SSH。
- app/data.py：数据验证、录制和回放。
- nano/main.c：两颗 VL53 与可选 MAVLink 采集。
- nano/platform.c：Linux I²C 适配。
- scripts/build_nano.py：WSL 交叉编译。
- tests/：数据协议与 GUI 操作测试。

在 VS Code 打开整个目录，选择“Drone Lab 图形实验台”运行配置。日常启动使用快捷方式，不需要激活虚拟环境或输入命令。

第三方源码与许可保留于 vendor，工具下载来源和 SHA256 记录于 downloads/download-results.json。
.codegraph 为本地索引数据，不纳入提交；索引只覆盖项目自有源码。
