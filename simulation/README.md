# Gazebo 四机软件在环

当前：真实 Gazebo Harmonic 8.15.0 + PX4 v1.15.4 四实例 + 四份 C++ companion。
仿真使用精确高度输入；其他水平位置/航向真值仅由评估器读取。

默认场地为 RMUC 2027 V1.0.0，使用 STEP 装配体生成的地形、机库起点与固定基地装甲。坐标、重建命令及检查结果见 [2027 场地说明](docs/rmuc2027.md)。

## 图形入口

打开 Drone Lab 的“Gazebo 多机仿真”页，选择地图和任务。界面默认勾选“自动解锁并执行任务（SITL）”；取消勾选后启动场景、传感器和定位记录。
任务选项包括“多机轮流撞击与返航”“定点悬停”和“地图航线”。2027 场地使用机库四机起点、编队航路及固定基地装甲目标。

## 标定房间实验记录

- 准确高度辅助：四个飞控实际外部高度融合，GPS关闭；测试时高度误差小于0.1m。
- 实际四机180g参考动力学起飞/悬停已经过30秒试验。
- 单探索机 + 三架参考机的65秒试验：探索机到达像素识别目标前约1.2m并对准，四机XY RMSE约1.4–2.5cm。
- 标定房间采用配置尺寸；2027 全场采用 RMUC2027 V1.0.0 STEP 装配体几何。
- 四机同时移动的测试失败，日志保留；完整参考交接/多机同时制导/撞击恢复尚未完成。
- GC4653仅提供渲染图像模型，UWB仅近似测距/方位/噪声/歧义；未完成硬件滚动快门、光学和射频标定。
- 同一核心已编译x86与riscv64；真实硬件I2C/CSI/UWB输入适配和NPU尚未接入。

## 主要代码

- gazebo/：真实引擎传感器桥接及UWB/高度插件。
- models/：x500参考与180g微型动力学生成器；generated_micro与generated_micro_horizontal。
- companion/：Eigen状态估计、ToF结构约束、UWB同伴约束、像素条带制导与MAVLink进程。
- launch_linux.py：Gazebo/PX4/companions进程管理。
- evaluate_linux.py：只读真值评估器，数据不提供给算法。
- tests/height_to_px4.py：高度-only诊断，运行前检查EV_CTRL=2。
- tests/sensor_smoke.py：实际渲染传感器测试。
- tests/flight_trial.py：独立记录真实物理飞行，不把脚本轨迹当测量。
- build_linux.py：在已安装的WSL环境重建plugins、companion、RV静态程序。

## 固定环境

WSL Ubuntu-22.04，PX4路径 /home/devcontainers/drone-sim/PX4-Autopilot，commit 99c40407ffd7ac184e2d7b4b293f36f10fe561ef。
Gazebo模型子模块 d754381a1cecdd7f17050acd72bf5bf1327bced6。
PX4配置CMake参数 CMAKE_DISABLE_FIND_PACKAGE_gazebo=ON 禁止自动下载旧Gazebo Classic。
输出 /home/devcontainers/drone-sim/build-sensors 与 build-companion；RV工具链位置在项目config/toolchain.json。

模型的控制分配CT=20用于绕过PX4 v1.15绝对阈值0.05，统一归一化不改变物理推力常数。
相机与ToF外参由启动器从sensor_contract.json同步到每次运行的companion_config.json。

完整日志在runtime/run_*/。参考成功记录184009与185621；失败记录175406、184356、184829。
