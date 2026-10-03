# Drone Lab 无人机算法实验台

面向 DM-FC01、LicheeRV Nano 的桌面采集、算法研究和 Gazebo/PX4 多机物理仿真。
双击 **Drone Lab 实验台.lnk** 启动；初次操作从“⑤ 从这里开始”进入。

**阅读源码先看 [项目源码阅读路线](docs/code-reading.html)**：介绍模块关系、数据流、坐标系、滤波变量和阅读顺序。

## 三条运行链路

| 链路 | 用途 | 入口 |
| --- | --- | --- |
| 实物采集 | 串口读取飞控，Nano 两颗 ToF 采集、录制与回放 | app/main.py → app/workers.py → nano/main.c |
| 离线预研 | 计算负载、通信、几何可观测性和控制策略模型 | app/research_panel.py → research/runner.py |
| 物理仿真 | Gazebo 传感器 → 伴随定位 → PX4 控制 → 多机任务 | simulation/launch_linux.py → simulation/companion/src/runner.cpp |

## 图形界面

- “① 实时数据”：ToF 热力图、飞控姿态、消息接收与录制。
- “② Nano 连接与部署”：SSH 检查、上传采集程序和启动采集。
- “③ 日志回放”：采集日志播放、拖动和 CSV 导出。
- “⑥ 预研与仿真”：运行离线数学模型，比较参数与结果。
- “⑦ Gazebo 多机”：选择场地、机群数量、基站和任务；“飞行数据曲线”显示轨迹、速度、误差及同步回放。

桌面采集使用两颗 VL53L5CX；物理仿真使用三颗 ToF、成品光流速度接口、机间 UWB 和可选固定基站。仿真高度直接提供给算法。RISC-V 程序已交叉编译，实体硬件尚未连接联调。

## 开发入口

1. [源码阅读路线](docs/code-reading.html)：从界面入口读到定位与任务调度。
2. [仿真操作说明](simulation/docs/team-mission.html)：场地、起飞点、四机快速任务和可视化。
3. [传感器接口](simulation/docs/sensor-interfaces.html)：光流、UWB 与机群配置。
4. simulation/companion/include/companion/types.hpp：数据类型、状态量、单位。
5. simulation/companion/src/estimator.cpp：预测、ToF/光流/UWB 融合。
6. simulation/companion/src/team_mission.cpp：任务阶段、接触轮次、共同返航。

在 VS Code 打开整个目录，选择“Drone Lab 图形实验台”运行配置。测试位于 tests/ 和 simulation/tests/；C++ 构建入口为 simulation/companion/CMakeLists.txt，Gazebo 插件入口为 simulation/gazebo/CMakeLists.txt。

第三方源码和许可证位于 vendor/，下载来源与 SHA256 记录于 downloads/download-results.json。simulation/runtime/ 保存运行配置和日志；生成模型应从生成脚本修改。 .codegraph/ 是本地工具索引，不纳入提交。
