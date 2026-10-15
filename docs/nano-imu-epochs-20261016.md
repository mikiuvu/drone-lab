# IMU 与姿态源时间对齐

日期：2026-10-16。姿态缓冲：`1749f70`；runner 接入及 UDP 回归：`b5f7939`。

## 输入时间与姿态采样

当前 PX4 的 `ATTITUDE` 将 `vehicle_attitude.timestamp / 1000` 写入 `time_boot_ms`；`HIGHRES_IMU` 使用 `vehicle_imu.timestamp_sample`。runner 原来保留最新欧拉角，将其直接附到每条后续 IMU 的时间上。两个流分别请求 50 Hz、100 Hz，曝光历史中的姿态时间因而与原消息不同。

`ImuAttitudeHistory` 保存最近 16 个带源时间的 ENU/FLU 四元数。IMU 位于两次姿态之间时使用 SLERP；晚于最新姿态时，以本条 IMU 的机体系角速度外推：

```text
q_IMU = q_attitude × quaternion(omega_body × delta_t)
```

机体系增量右乘，角速度由 FRD 的 `[p,q,r]` 转为 FLU 的 `[p,-q,-r]`，不作为欧拉角导数相加。插值间隔和外推时长上限均为 100 ms；外推要求三个陀螺字段均已更新且有限。早于历史覆盖范围、过大的插值间隔或不可外推的输入不进入预测。重复及乱序姿态不刷新历史，重复及倒退的 IMU 时间不进入预测。32 位毫秒启动时间按附近的 64 位 IMU 时钟展开，覆盖回绕。

短时外推采用恒定机体系角速度模型。起点航向对齐、地图航向修正、8 秒启动等待、飞控反馈有效期及控制流中断门限沿用原值。直接姿态控制仍使用原生飞控航向。

## UDP 回归

通过 runner 的回环 MAVLink 输入，先建立静止姿态，再发送晚 20 ms、机体 Z 角速度为 1 rad/s 的 IMU。应得到 ENU 航向变化 −0.02 rad。冻结的旧 runner 输出 0，新 runner 输出正确值；随后插入更早、航向不同的姿态消息，新的 IMU 仍得到 −0.03 rad。

旧程序在同一新用例中按预期返回失败，失败信息和二进制、源码及链接库哈希均已保存。新实现 OpenCV 17/17 项 CTest 通过，无 OpenCV 算法测试 15/15 项、IO 测试 1/1 项通过。新增用例覆盖三轴转动、航向跨 ±π、四元数符号、机体系角速度、历史覆盖、无效输入、时间边界和启动时间回绕。

## Nano 数值与开销

无噪声解析序列包含恒定三轴角速度、50 Hz 姿态和 100 Hz IMU，IMU 采样比发送轮次早 3 ms。分别按姿态先到、IMU 先到构造消息顺序，每种 1,000 次查询，在 Nano 上重复三轮。

| 消息顺序 | 旧保持姿态角度 RMSE / ° | 对齐后角度 RMSE / ° | 旧重力补偿残差 RMS / m/s² | 对齐后残差 RMS / m/s² | 姿态查询均值 / µs |
|---|---:|---:|---:|---:|---:|
| 姿态先到 | 0.482132 | 4.37e-14 | 0.052828 | 5.07e-15 | 1.785–1.877 |
| IMU 先到 | 1.163072 | 4.92e-14 | 0.127438 | 5.60e-15 | 1.613–1.735 |

两种顺序均接收 999 次，首次查询早于已有历史而被拒绝。姿态先到时包含 499 次插值和 500 次外推；IMU 先到时为 999 次外推。Nano 缓冲对象大小 656 B，三轮查询墙钟最大值为 136.52 µs。板端 `test_imu_attitude` 通过，上传文件哈希匹配，前后启动 ID 相同，临时目录已清理。

表中精度对应恒定角速度解析输入，实际运动中的角加速度、陀螺误差及姿态发布时间延迟仍会产生误差。

## 原生发布时间

既有运行 `run_20261016_020156` 的四份原生日志中，姿态发布时间减采样时间的中位数均为 0，P99 分别为 2、1、1、0 ms，最大值为 12、1、1、1 ms。`ATTITUDE` 只有量化到毫秒的发布时间，对齐到该时间不能恢复消息中未携带的原始采样时间。

runner 的首个 IMU 到仿真时钟映射保留原算法，并增加 `imu_clock_alignment` 记录。两组新运行的映射偏移分别为 `[5,-7,-7,-11]` ms、`[-15,-7,-11,1]` ms；这是首包配对得到的偏移，不是独立的时钟同步测量。

## 四机闭环

两组均使用默认地图、`EKF2_EV_CTRL=7`、`SDLOG_PROFILE=3` 和执行器记录。

| 运行目录 | 合格接触 | 首次终态与时刻 | 原生着地停机 | 最大起点水平误差 / m |
|---|---|---|---|---:|
| `run_20261016_023233` | 1/4，1 号 | ABORTED，52.681 s | 四机就地降落停机 | 10.5267 |
| `run_20261016_023356` | 3/4，0、2、3 号 | ABORTED，65.961 s | 四机返航降落停机 | 0.11336 |

第一组在 42.521–42.541 s 记录四机控制流中断锁存，其中两机同时记录心跳中断。该组没有返回起点；四机最终距起点约 8.43–10.53 m。中断的具体原因尚未定位，结果没有证明全队可靠性改善。

两组最大姿态外推时长分别为 20、24 ms，最大有效插值间隔为 22 ms；IMU 姿态采样缺失计数分别为 17、0，姿态及 IMU 时间戳拒绝计数均为零。ToF 姿态历史缺失记录全部早于首次原生解锁。

八架次均通过原生着地停机及禁止重新解锁检查，原生日志均无 Navigation failure，首次原生解锁后的回放输入时间戳审计通过。四次压力确认均对应独立合格压力；四次碰撞恢复均等待 60 ms，另一次候选由压力确认取消，零参考稳定等待和 ToF 抑制检查通过。

## 数据文件

- 源码时间字段及哈希：`build/nano-imu-alignment/native-source/`、`native-source-manifest.json`。
- 旧 runner 失败用例：`build/nano-imu-alignment/baseline-io-regression.log`、`baseline-io-regression.json`。
- 主机回归：`build/nano-imu-alignment/final-ctest.log`、`noopencv-ctest.log`、`noopencv-io-ctest.log`。
- Nano 原始结果、三轮汇总及构建哈希：`build/nano-imu-alignment/board-result.json`、`board-summary.json`、`build-manifest.json`。
- 原生发布时间测量：`build/nano-imu-alignment/native-epochs.json`。
- 闭环汇总：`build/nano-imu-alignment/closed-loop-summary.json`。
- 压力、停机、ULog 和源码审计：`build/nano-flight-feedback/imu-epoch-default/`、`imu-epoch-repeat/`。
