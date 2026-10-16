# IMU 冲量采样与积分校验

日期：2026-10-16。实现：`043c33f`、`4537269`、`5b3ebd7`。

## 结论

当前 Gazebo 配置的物理步长为 1 ms，micro 原生 IMU 默认以 250 Hz 发布。独立落体对照确认：短于发布周期的接触冲量可能没有进入 IMU 输出。相同刚体上的 1,000 Hz 传感器能够记录该次冲量。这解释了部分接触后的惯性积分偏差，但不能独立解释全部任务失败。

可选生成参数 `--imu-rate-hz 1000` 同时设置 IMU 更新频率和导出的 PX4 `IMU_INTEG_RATE=1000`。常规生成仍使用原有 250 Hz 配置。两组高频完整任务分别取得 1/4、3/4 合格接触，均为 ABORTED，四机均返航并确认原生停机；全队任务可靠性尚未改善到可接受程度。

## 同一刚体的独立物理对照

校验场景使用 1 kg、半径 0.1 m 的球体从中心高度 1 m 自由落下，同时安装无噪声的 250 Hz 和 1,000 Hz IMU。物理步长 1 ms，两个传感器位置、方向相同。记录器读取物理引擎实际速度，积分传感器比力并补回重力。

实际环境：gz-sim 8.15.0、gz-sensors 8.2.2、gz-physics 7.8.0；DART 插件链接 libdart 6.13。校验共记录 700 条物理速度、700 条高频 IMU、176 条低频 IMU。

| 窗口 | 实际垂向速度变化 / m/s | 250 Hz 积分 / m/s | 1,000 Hz 积分 / m/s |
|---|---:|---:|---:|
| 自由落体，0.100–0.250 s | −1.470998 | −1.470998 | −1.470998 |
| 接触，0.380–0.530 s | +3.727527 | −0.470719 | +3.727527 |

高频 IMU 在 0.429 s 记录到约 4,208.01 m/s² 比力；低频 IMU 相邻采样为 0.428 s、0.432 s，未记录该峰值。接触窗口低频积分误差为 4.198246 m/s，高频误差小于 1e−6 m/s。自由落体负对照同时通过，不能将该结果归因于简单的重力符号错误。

[物理场景](../simulation/tests/fixtures/imu_impulse.sdf)与[原生校验程序](../simulation/tests/test_gazebo_imu_impulse.cpp)可通过 Gazebo CMake 选项 `DRONE_GAZEBO_BUILD_INTEGRATION_TESTS=ON` 构建，以 `ctest -R gazebo_imu_impulse --output-on-failure` 运行。测试核验采样覆盖、高频接触积分与两种频率的自由落体积分；低频接触差异作为测量结果输出。

## 完整任务日志中的积分

[audit_contact_impulse.py](../scripts/audit_contact_impulse.py)按 `accelerometer_integral_dt` 积分原生 `sensor_combined`，在裁剪后的区间中点以 SLERP 取得姿态，将机体 FRD 比力转入 NED 并加入重力。姿态及速度优先使用 `timestamp_sample`。

缺失区间保留为缺口，部分积分不与整窗速度变化比较；重叠区间、非有限输入、非法四元数、非递增时间和非零加速度相对时间被拒绝。原生速度端点还需通过横向、垂向有效标志检查。

每次压力确认采用原生时间的前 100 ms 至后 650 ms。下表为积分与 Gazebo 位姿差分速度变化之间的三轴模长差。完整任务中的 `vehicle_local_position_groundtruth` 速度来自桥接端的位姿差分，与独立落体校验直接读取的物理速度不同。

| 运行 | 机号 | 估计姿态积分差 / m/s | 真值姿态积分差 / m/s |
|---|---:|---:|---:|
| logger-retention-first，250 Hz | 0 | 2.776 | 2.807 |
| logger-retention-first，250 Hz | 1 | 2.007 | 2.044 |
| logger-retention-first，250 Hz | 2 | 1.657 | 1.700 |
| logger-retention-first，250 Hz | 3 | 2.478 | 2.544 |
| logger-retention-repeat，250 Hz | 1 | 1.421 | 1.975 |
| logger-retention-repeat，250 Hz | 2 | 3.464 | 3.756 |
| impulse-imu1000-integ1000 | 2 | 0.172 | 0.149 |
| impulse-imu1000-integ1000-repeat | 1 | 0.333 | 0.077 |
| impulse-imu1000-integ1000-repeat | 2 | 0.215 | 0.129 |
| impulse-imu1000-integ1000-repeat | 3 | 0.237 | 0.019 |

十个接触窗口的 IMU 积分覆盖完整，均无加速度裁剪。真值姿态只用于误差归因，没有进入定位器或飞行控制。

在高频首组 2 号的同一条记录上，每四条保留一条并保持四步，重建四种 250 Hz 采样相位，接触窗口的差异分别为 0.268、2.119、0.258、2.036 m/s；原高频积分为 0.149 m/s。接触前平稳窗口约为 0.028 m/s，接触后平稳窗口约为 0.098 m/s，四种相位相近。这是同轨迹的采样重建，不能代替实际低频传感器运行。

接触窗口沿用 runner 首包 IMU 时间映射，不是独立时钟同步。窗口前后平移 20 ms 后，低频差异范围为 1.697–3.756 m/s，高频为 0.018–0.151 m/s；全部窗口覆盖完整。结果保存于 `clock-window-sensitivity.json`。

## 高频任务与记录完整性

| 运行 | 合格接触 | 终态 | 四机原生停机 | 最大返航误差 / m | 解锁后索引输入完整 |
|---|---:|---|---|---:|---:|
| run_20261016_081939 | 1/4 | ABORTED | 是 | 0.0614 | 4/4 |
| run_20261016_082855 | 3/4 | ABORTED | 是 | 0.1386 | 1/4 |

八机次实际参数均为 `IMU_INTEG_RATE=1000`，解锁后的 `sensor_combined` 发布间隔中位数均为 1 ms，但存在 2–5 ms 间隔。重复组的三份不完整日志缺失 EKF2 引用的 `sensor_combined`，里程计历史保留不能补回其他话题。首组日志写缓冲区未报告 dropout；重复组 0 号的这次缺失也没有写缓冲区 dropout。

所有压力确认均与独立物理接触匹配，未确认碰撞的参考退出及 ToF 抑制审计通过。第二组运行期间还有独立落体程序的构建和执行，不用于 CPU 或调度性能对比。两组样本不足以量化任务成功率。

只提高传感器频率、保留 250 Hz 积分的先行运行 `run_20261016_081636` 在仿真时间 4.541 s 停住，未进入任务，已单独保留失败记录。PX4 `VehicleIMU.cpp` 明确要求锁步下每条原始采样对应一次积分发布；匹配至 1,000 Hz 后两组正常推进。停滞的完整因果链尚未独立复现。

## 生成与验证

micro 模型生成器接受 `--imu-rate-hz 250` 或 `--imu-rate-hz 1000`，只在显式选择时把匹配的积分参数写入共享契约、物理清单和 `px4_micro_reference.params`。SDF 与参数文件必须成套使用；地图中同名 PX4 参数的覆盖也应保持一致。

模型、网格、积分与垂向速度检查共 39 项通过，包含两种频率的 SDF 与配套参数、噪声与几何保持一致、非法配置拒绝，以及实际 PX4 原始噪声配置对照。Gazebo 原生落体 CTest 通过。

默认模型文件哈希与高频对照前一致，默认 PX4 固件没有更换。压力评分、1 s 低高度保护、500 ms 控制流保护、原生着地及普通 DISARM 确认保持原值。

高频记录仍有输入缺失，全队任务仍有未命中；现有数据尚不足以验证严格 20 ms 导航、稳定全队成功及实际倒扣后的原生停机。

## 证据

- `build/nano-impulse-integration/physics-drop.json`：独立落体物理速度与双频 IMU。
- `build/nano-impulse-integration/pressure-window-comparison.json`：十个接触窗口积分。
- `build/nano-impulse-integration/sampling-phase-comparison.json`：同轨迹采样相位重建与平稳负对照。
- `build/nano-impulse-integration/clock-window-sensitivity.json`：窗口平移敏感度。
- `build/nano-impulse-integration/impulse-summary.json`：参数、采样间隔、输入覆盖及返航结果。
- `build/nano-impulse-integration/reference-manifest.json`：Gazebo、DART 参考源码来源及哈希。
- `build/nano-flight-feedback/impulse-imu1000-integ1000*/`：源文件快照、运行映射、原生日志、恢复和接触审计。
