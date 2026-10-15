# Gazebo IMU 时钟与转子停转记录

日期：2026-10-16。平台：Gazebo/PX4 四机 SITL。

## 时钟来源

当前 PX4 构建的 `GZBridge.cpp`、`drv_hrt.cpp` 和 MAVLink 模块均启用了 `ENABLE_LOCKSTEP_SCHEDULER`。Gazebo 世界时钟和 IMU 消息头推进 PX4 锁步时钟；IMU 采样时间沿 `sensor_gyro.timestamp_sample`、`vehicle_imu.timestamp_sample` 传至 `HIGHRES_IMU.time_usec`，没有扣除进程启动纪元。

PX4 提交为 `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`，固件 SHA256 为 `bca8bc47730c9f76b2f6a5433fe59229d1658c222116c5087786bb6f17d34676`。源码、编译参数和哈希保存在 `build/nano-clock-contract/reference/` 与 `reference.json`。

从 `run_20261016_053416` 的正确 Gazebo partition 直接订阅 200 条 IMU 消息，覆盖 47.984–48.780 s。200 个时间戳均匹配原生 `sensor_combined`；同一窗口内收到的 80 条 MAVLink IMU 消息也全部匹配 Gazebo 源时刻。

旧首包映射把最新观测时间作为 IMU 纪元。在 `run_20261016_052719` 中，四机由此得到固定偏移 +1、−15、−11、+5 ms。四组运行共 99,612 条原始 IMU 消息，ULog 覆盖区间内的时间戳全部匹配原生采样；5,695 条位于日志覆盖区间外。`vehicle_imu` 在这些日志中仅约 2 Hz，不能把该主题的未匹配记录直接解释为输入丢失。

## 运行参数

启动器和 runner 支持 `--imu-clock arrival|gazebo`，默认 `arrival`。

| 模式 | IMU 到观测时间的映射 | 使用条件 |
|---|---|---|
| `arrival` | 首个可用包建立相对纪元，后续使用源时间增量 | 保留已有输入行为 |
| `gazebo` | 直接使用 IMU 源时间戳 | PX4 与传感器共享已核验的 Gazebo 锁步时钟 |

两种模式均拒绝零时间戳及与最新观测相差超过 500 ms 的映射；相对映射检查无符号溢出。`imu_clock_alignment` 记录模式、IMU 源时间、映射时间与当时的观测时间。八个 `gazebo` 机次的映射偏移均为零。

```powershell
$ErrorActionPreference = 'Stop'
wsl -d Ubuntu-22.04 --exec python3 '/mnt/d/Robomaster code/drone/simulation/launch_linux.py' start --mission team_contact --companions --fly --no-tof-rays --imu-clock gazebo
if ($LASTEXITCODE -ne 0) { throw 'Simulation launch failed' }
```

`DRONE_RECORD_IMU_INPUT=1` 按需记录 `imu_input` 和 `attitude_input`，包含源时间、收包单调时钟、最新观测时间、惯性字段有效位、加速度、角速度及姿态。`abs_pressure` 保留接收字段原值。默认关闭。`ATTITUDE.time_boot_ms` 仍来自原生姿态发布时间，姿态实际采样时间与陀螺平均窗口需要另外核对。

## 四机验证

四组均使用原噪声模型、默认任务地图、`EKF2_EV_CTRL=7`、`SDLOG_PROFILE=3`，记录原始惯性消息和实际转子关节状态。

| 模式及组别 | 运行目录 | 合格压力接触 | 首次终态 / s | 原生停机 | 最大起点水平误差 / m |
|---|---|---|---|---|---:|
| arrival 基线 | `run_20261016_052719` | 4/4 | COMPLETE，58.081 | 四机确认 | 0.05862 |
| gazebo 第一组 | `run_20261016_053122` | 0/4，起飞阶段超时 | ABORTED，55.881 | 四机确认 | 0.07009 |
| gazebo 重复组 | `run_20261016_053416` | 2/4，0、2 号 | ABORTED，77.821 | 四机确认 | 11.77767 |
| arrival 重复组 | `run_20261016_053640` | 0/4，起飞阶段超时 | ABORTED，55.601 | 四机确认 | 0.10554 |

`gazebo` 重复组 3 号在接触阶段落地并锁存任务失败，停在目标附近，没有返回起点。两次起飞超时分别包含未离地的 2 号和 3 号。原生 Navigation failure 均为零。

六次合格压力确认均匹配独立压力记录。`gazebo` 重复组五次未确认冲击的等待、恢复参考归零及 ToF 暂停融合检查通过；arrival 基线没有未确认冲击，恢复动作检查不适用。

原生 EKF 时间索引输入检查 12/16 份完整。其余分别为 gazebo 第一组 0 号、gazebo 重复组 1 号，以及 arrival 重复组 1、3 号，保留为完整性失败。

导航间隔 P99 为 20.127–20.187 ms，成功 UDP 参考提交间隔最大 27.453 ms；严格 20 ms 上限未通过。原始消息记录增加了诊断日志量，这些运行不用于推断默认关闭记录时的吞吐收益。

共享时钟的来源和零偏移已得到验证，闭环结果尚未证明任务可靠性收益，默认仍使用 `arrival`。

## 非零电机参考下的实际停转

在 gazebo 第一组 2 号、arrival 重复组 3 号，PX4 已确认解锁，位置估计高度仍接近地面。解锁后 2–6 s 的四路归一化电机参考均值分别约为 0.718–0.725、0.613–0.854。

11–16 s 的独立 Gazebo 物理记录中，两架失败机的四转子关节速度均接近零，绝对峰值低于 1.5e-26 rad/s；同组已起飞机的关节平均转速约 59.7 rad/s。两种时钟模式都复现了该现象。

PostUpdate 的 `JointVelocityCmd` 在健康机上也可能为零，不能据此判断命令未发送。PX4 ESC 状态回读其自身发布的电机命令，也不能替代实际关节速度。后续需同时捕获原生执行输出、Gazebo 电机话题和物理插件执行状态，定位命令到实际转动之间的中断位置。

## 回归与记录

- CTest：20/20，通过 15 项 UDP/MAVLink 输入场景；包含独立纪元、观测相位偏移、500 ms 时差拒绝、溢出和控制流中断检查。
- 启动器测试：3/3。
- `build/nano-clock-contract/ctest-opencv.log`：完整主机回归。
- `build/nano-clock-contract/closed-loop-summary.json`、`navigation-intervals.json`：四组闭环及单调时钟间隔。
- `build/nano-flight-feedback/clock-*/`：源码快照、原生 ULog、惯性时钟审计、压力、恢复与执行器记录。
- `build/nano-clock-contract/final-process-status.json`：仿真进程退出检查。
