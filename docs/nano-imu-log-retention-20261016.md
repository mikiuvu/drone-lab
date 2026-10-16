# 高频 IMU 与时间索引的日志保留

日期：2026-10-16。实现：`959812f`。PX4 基线：`99c40407ffd7ac184e2d7b4b293f36f10fe561ef`。

## 缺失定位

1,000 Hz IMU 的两组对照中，八份解锁后索引输入只有五份完整，缺失话题为 `sensor_combined`。重复组 0 号在原生 23.352 s 的 logger 订阅代号从 22137 跳至 22139，覆盖了 EKF2 在 23.351 s 引用的 IMU 记录。写缓冲区 dropout 为零；同一时段的 `ekf2_timestamps` 订阅也有跳号。

这是订阅端覆盖，增加 ULog 写缓冲区不能恢复已覆盖的数据。既有的八条外部里程计保留不包含这两个高频话题。

## 可选保留策略

[imu-replay-log-retention.patch](../simulation/px4/imu-replay-log-retention.patch)在既有里程计历史补丁之后应用：

| 话题 | 日志历史容量 | logger 每轮读取上限 | 普通订阅队列元数据 |
|---|---:|---:|---:|
| vehicle_visual_odometry | 8 | 8 | 1 |
| sensor_combined | 16 | 16 | 1 |
| ekf2_timestamps | 16 | 16 | 1 |

普通订阅保持最新值读取，logger 使用独立代号读取保留历史。历史溢出仍产生代号跳变和 gap 记录。读取预算有界，其他话题按原策略处理；非全速日志订阅不切换到历史读取。

[独立构建器](../scripts/build_px4_logger_diagnostic.py)接受 `--retain-visual-odometry --retain-imu-replay --check-retention-contract`，在复制源码中顺序应用补丁并链接候选 logger/uORB 静态库。源 SDK 与默认固件哈希在构建后保持一致。

实际构建的 `sensor_combined_s` 为 48 B，`ekf2_timestamps_s` 为 24 B；两者由一条增加至十六条，比只保留里程计的候选增加 1,080 B 数据缓冲区，四个 SITL 进程共 4,320 B，不含分配器管理开销。DeviceNode 在基线、候选均为 176 B。

## 原生读取检查

[原生 uORB 测试](../simulation/tests/test_px4_odometry_retention.cpp)分别针对三个话题链接实际基线与候选静态库，调用 Manager：

- 基线均在三次突发发布后的顺序读取检查中失败，确认原问题可复现。
- 候选均通过突发读取、普通订阅读取最新值、独立读取者、迟到订阅、溢出与无历史话题的回退检查。
- 每个话题完成十万次并发发布，检查保留读取的时间递增及最终最新值；超过容量的中间记录允许覆盖。

三个候选原生测试退出码均为零，三个基线均在预期断言退出；核心转储关闭。构建器 7 项回归通过。

## 完整任务对照

两组使用相同的 1,000 Hz 传感器模型、匹配积分参数及同一候选二进制。保留压力评分、1 s 低高度保护、500 ms 控制流保护，以及原生着地后普通 DISARM 确认。

| 运行 | 合格接触 | 终态 | 解锁后索引输入完整 | 四机原生停机 | 最大返航误差 / m |
|---|---:|---|---:|---|---:|
| run_20261016_084434 | 3/4 | ABORTED | 4/4 | 是 | 0.0423 |
| run_20261016_084657 | 3/4 | ABORTED | 4/4 | 是 | 0.0398 |

八机次的三个保留话题在解锁后均无 logger 订阅跳号，写缓冲区 dropout 为零，EKF2 引用的索引输入完整。两组仍分别有 9、8 次其他话题跳号，不能据此声称所有日志无丢失。输入完整统计限定解锁后区间，不代表启动阶段或原生数值回放已完整验证。

六次压力确认均匹配独立物理记录，未确认碰撞的参考退出与 ToF 抑制审计通过。八机次没有原生 Navigation failure，最终均返航、着地并原生停机；两组各有一机未获得合格压力确认，全队任务没有完成。

导航墙钟间隔 P99 为 20.125–20.216 ms，最大 21.245 ms；OFFBOARD 参考提交间隔最大 21.273 ms，严格 20 ms 仍未通过。这些提交时间来自 runner 的实际发送调用记录。

## 构建与证据

候选二进制：`/home/devcontainers/drone-sim/build-imu-retention-v1/bin/px4`。

- 候选 SHA-256：`5df9e7c06cd3e790c1b37b5832b8ef6cf61ee2a1f0717dd2dd8ebb0d22170ef4`。
- 默认 SHA-256：`bca8bc47730c9f76b2f6a5433fe59229d1658c222116c5087786bb6f17d34676`。
- `build/nano-imu-retention/manifest.json`：编译链接参数、源码与补丁哈希。
- `build/nano-imu-retention/retention-contract*.json`：三个话题的基线失败和候选原生测试。
- `build/nano-imu-retention/baseline-missing-imu.json`：原高频日志缺失与相邻订阅跳号，包含解锁前记录。
- `build/nano-imu-retention/message-sizes.json`：实际编译的消息与节点尺寸。
- `build/nano-imu-retention/closed-loop-summary.json`：两组源快照哈希检查、原生停机、逐话题跳号与输入覆盖。
- `build/nano-imu-retention/timing-summary.json`：八机次导航与参考提交间隔。
- `build/nano-flight-feedback/imu-retained-*/`：源快照、原生日志、完整性与恢复审计。
