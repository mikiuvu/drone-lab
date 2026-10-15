# micro 静止传感器 STALE 与噪声配置

日期：2026-10-16。模型修正：`28abbf6`。

## 连续采样中的数值冻结

`run_20261016_044906` 的 2 号机在 41 次 ARM 暂时拒绝后仍未解锁。其 `sensor_combined` 包含 13,977 条记录，时间从原生 1.420 s 持续到 57.328 s，最大间隔 8 ms。8 s 后的相邻三轴加速度变化始终小于 1e-6 m/s²。

PX4 `DataValidator` 分别检查采样超时和重复值。默认重复值门限为 100，逐轴比较阈值为 1e-6；连续几乎不变的输出会产生 `ERROR_FLAG_STALE_DATA`。

从该架次原生 8–12 s 提取 1,001 条实际加速度记录，链接当前 PX4 的原版 `libdata_validator.a` 回放：967 条触发 STALE，采样期间 TIMEOUT 为零。停止输入并超过校验器超时时间后，TIMEOUT 仍被检出。

原 micro 模型只配置了 IMU 和气压计更新率，没有噪声模型。完全静止时的理想输出可持续满足重复值判据。

## 原生传感器参数

micro 生成器采用 PX4 随附 `x500_base` 的高斯噪声及动态偏置参数，三轴均使用相同配置，均值为零。

| 输出 | 白噪声标准差 | 动态偏置标准差 | 相关时间 / s |
|---|---:|---:|---:|
| 角速度 | 0.00018665 rad/s | 0.000038785 rad/s | 1000 |
| 线加速度 | 0.00186 m/s² | 0.006 m/s² | 300 |
| 气压 | 0.01 Pa | — | — |

IMU 仍以 250 Hz、气压计以 50 Hz 更新。`sensor_contract.json` 和 `micro_physics_manifest.json` 的 `native_sensor_noise` 保存剖面名称、参数和单位。导入的 x500 模型保留原传感器配置。

参数来源为 PX4 `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` 使用的 Gazebo 模型提交 `d754381a1cecdd7f17050acd72bf5bf1327bced6`。原生飞控二进制、校验库及源码哈希保存于 `reference-hashes.json`。

普通和水平 ToF 的本地 micro 模型缓存均已更新并保留备份。移除新增原生噪声节点后，前后 SDF 的规范化内容哈希一致，覆盖质量、惯量、碰撞体、旋翼、传感器安装和视觉 URI；其余资源文件按字节核对。PX4 参数和健康检查门限保持原值。

## 回归与故障注入

新增回归在旧生成器上因缺少 `angular_velocity/x` 噪声配置而失败。修正后的生成器及现有网格接入测试共 13 项通过，包括六轴、气压噪声、与 PX4 源模型逐项一致及导入模型配置保留。两类缓存模型通过 SDF 校验。

两组新运行共八机次，分别提取起飞前原生 3–7 s 的 1,001 条实际加速度输入，均交给同一原版校验库处理。

| 输入 | 每段样本数 | 每段 STALE 次数 | 采样期间 TIMEOUT |
|---|---:|---:|---:|
| 八段实际静止输入 | 1001 | 0 | 0 |
| 冻结首个样本的故障注入 | 1001 | 967 | 0 |

停止输入后八段均触发 TIMEOUT。冻结故障使用单独的合成测试文件，原始 ULog 保留。

## 四机闭环

两组 Gazebo/PX4 SITL 使用默认地图、`EKF2_EV_CTRL=7`、`SDLOG_PROFILE=3`、执行器记录及阶段计时。

| 运行目录 | 解锁及起飞 | 合格压力接触 | 首次终态及仿真时刻 | 原生返航停机 | 最大起点水平误差 / m |
|---|---|---|---|---|---:|
| `run_20261016_050730` | 四机确认 | 4/4 | COMPLETE，63.441 s | 四机确认 | 0.11270 |
| `run_20261016_051116` | 四机确认 | 2/4，1、3 号 | ABORTED，69.461 s | 四机确认 | 0.04192 |

八机次原生日志均无 STALE，均成功解锁；启动期间仍按原健康检查等待就绪。六次合格接触与独立压力记录逐一匹配。五次未确认冲击等待 60 ms 后恢复，另两次候选被压力确认取消；恢复参考归零和 ToF 暂停融合检查通过。

八机次无本地失效锁存或原生 Navigation failure。接触成功率仍不稳定。实测导航间隔 P99 为 20.134–20.193 ms，UDP 参考提交最长 25.222 ms，严格 20 ms 上限未通过。

原生估计输入检查七份通过；第二组 2 号缺一条 `vehicle_visual_odometry` 引用，完整性检查失败。

## 测量文件

- `build/nano-native-startup/log-comparison.json`：失败机及成功机的原生采样比较。
- `build/nano-native-startup/validator-replay.json`：旧实际输入的原版校验器回放。
- `build/nano-native-startup/ideal-model-regression-final.log`、`generator-tests.log`：旧模型失败及 13 项回归。
- `build/nano-native-startup/{model-update,horizontal-model-update}.json`：缓存备份、模型哈希与物理内容核对。
- `build/nano-native-startup/reference/`、`reference-hashes.json`：原版校验器源码、参考 SDF、二进制与版本哈希。
- `build/nano-native-startup/native-noise-{default,repeat}-validator.json`：实际静止输入和冻结故障测试。
- `build/nano-native-startup/startup-summary.json`：逐机解锁、健康状态和输入检查。
- `build/nano-native-startup/navigation-intervals.json`：单调时钟间隔。
- `build/nano-flight-feedback/native-noise-{default,repeat}/`：源码及模型快照、ULog、压力和恢复审计。
