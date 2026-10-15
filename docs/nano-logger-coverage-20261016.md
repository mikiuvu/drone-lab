# 里程计输入覆盖与 logger 订阅跳号

日期：2026-10-16。审计实现：`249b859`；独立诊断构建器：`318bd6c`、`e4119b9`。

## 缺失发生在订阅阶段

`run_20261016_072245` 的 2 号机在原生时刻 35.644 s 的 EKF2 记录中，引用了 35.641 s 的 `vehicle_visual_odometry` 输入。ULog 没有该输入，前后实际记录分别为 35.625 s 和 35.644 s。

同一次运行的 logger 逐话题诊断输出：

```text
subscription gap topic=vehicle_visual_odometry previous=1562 current=1564 sample_us=35644000
```

logger 从第 1562 代直接读到 1564 代，跳过的输入时刻与 EKF2 的缺失引用对应。ULog 写缓冲区 dropout 为零。

本机 PX4 `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` 的生成元数据将该话题队列深度设为 1。`DeviceNode::copy` 对深度为 1 的话题直接返回最新消息；logger 每轮对每个订阅只取一次更新。前一条输入虽已被 EKF2 使用，但在 logger 读取前被后一条发布覆盖，不能通过扩大 ULog 写缓冲区补回。

另一份只增加 `logger_status` 记录的诊断运行也出现同类缺失：`run_20261016_071646/1` 引用 24.091 s，日志只保留相邻的 24.073 s 和 24.092 s；25.008 s 的状态显示 `message_gaps` 从 2 增至 3，写缓冲区 dropout 仍为零。

## 审计输出

审计在原有逐条索引检查之外，增加：

- `nearest_logged_before_us`、`nearest_logged_after_us`：缺失时刻两侧的实际记录，没有记录时为 `null`。
- `write_dropout_events`、`write_dropout_duration_ms`：ULog 写缓冲区溢出的次数与累计持续时间。
- `full_log_message_gaps_max`：0 号 logger 状态实例中记录的最大订阅跳号事件计数；日志未保存该状态时为 `null`。

订阅跳号计数与写缓冲区溢出计数相互独立。一次跳号可能跨过多条发布；跳过的发布也不一定被 EKF2 使用。因此，输入索引完整不代表每次发布都被保存，零 dropout 也不能证明输入完整。

8 项审计测试覆盖负相对时间、无对应话题、时间窗边界、相邻记录、独立丢失计数和多实例拒绝。12 份真实 ULog 与旧审计逐项比较，输入完整性及缺失索引保持一致，11 份完整、1 份失败。

## 可重复的诊断构建

[构建器](../scripts/build_px4_logger_diagnostic.py)复制两个 logger 源文件，应用[诊断补丁](../simulation/px4/logger-subscription-diagnostics.patch)，复用本机编译数据库、其他对象和链接参数，生成独立的 `bin/px4`。原 SDK 源码、静态库及默认固件不被替换。构建步骤和分析命令见 [PX4 logger 订阅诊断](../simulation/px4/README.md)。

6 项构建器测试覆盖带空格参数、复合 Ninja 命令、缺少或重复 logger 库、多个链接步骤、额外 shell 操作及父仓库隔离。嵌套在工作区内的独立构建也通过，复制源码的补丁应用保持 LF 换行，不修改父仓库文件。构建器在当前 SDK 上编译链接通过，最终程序的四机起飞与主动中止返航验证通过；四机均经原生确认停机，均记录 `logger_status`，四份索引输入完整。

| 程序 | SHA-256 |
|---|---|
| 默认固件 | `bca8bc47730c9f76b2f6a5433fe59229d1658c222116c5087786bb6f17d34676` |
| 只记录 logger 状态的诊断程序 | `eb205cfdf5d201b2b39c01267c3070b1aeb929b8e2007921ff5fc689214b21ae` |
| 逐话题代号诊断程序 | `c04eecf9ecb71932f2418d8ed728ca8fabf0bfc0fcfc81c540131794afa240fd` |
| 最终构建器生成并运行的程序 | `b85d3e0a15419841be3c7891b3e9a16bb60a1b7f230f723f6885ccef4f095039` |

逐话题日志增加运行负载，其时序结果不能直接等同于默认固件。

## 运行结果

所有运行均为 Gazebo/PX4 四机 SITL，runner 与到达模型保持当前默认配置。

| 运行 | 固件 | 合格接触 | 任务结果 | 原生停机 | 解锁后索引完整性 |
|---|---|---|---|---|---|
| `run_20261016_070233` | 默认 | 3/4 | ABORTED，全部返航 | 4/4 | 4/4 |
| `run_20261016_071120` | 默认 | 未进入接触阶段 | 主动中止并返航 | 4/4 | 4/4 |
| `run_20261016_071646` | logger 状态诊断 | 未进入接触阶段 | 起飞阶段中止并停机 | 4/4 | 3/4 |
| `run_20261016_072245` | 逐话题代号诊断 | 4/4 | COMPLETE，全部返航 | 4/4 | 3/4 |
| `run_20261016_072846` | 最终构建器程序 | 未进入接触阶段 | 主动中止并返航 | 4/4 | 4/4 |

逐话题诊断组四机的物理返航误差为 0.0210、0.0219、0.0266、0.0052 m。四次压力确认均与独立物理记录匹配，两次未确认碰撞的零速度恢复参考、60 ms 等待及 ToF 抑制检查通过。任务完成与日志完整性分别判定，该组日志完整性仍为失败。

原生 CLI 状态查询还有独立的可观测性问题：Unix socket 的 `SO_PEERCRED` PID 与运行管理器记录的 PX4 PID 和启动时刻匹配，但查询返回 logger 未运行、uORB 仅少量话题，与同进程生成的 ULog 不一致。这些 CLI 输出不用于判定订阅丢失，原因尚未确定。

## 后续修复条件

队列扩容需要同时核验 EKF2 与其他订阅者的取数语义。当前多元素队列按未读代号顺序返回数据，直接扩大公共话题队列会改变其取数行为。候选方案须保留飞控采用最新输入的语义，使 logger 有界地保存已发布或已融合的输入，并验证突发发布、订阅滞后与队列溢出。

严格 20 ms 导航间隔、稳定全队任务成功率和原生回放数值一致性继续单独验收。

## 本地证据

- `build/nano-logger-coverage/audit-regression.json`：12 份真实日志的新旧审计对照。
- `build/nano-logger-coverage/diagnostic-gap-summary.json`：只增加状态记录的缺失与计数变化。
- `build/nano-logger-coverage/generation-trace-summary.json`：逐话题代号与缺失引用的对应关系。
- `build/nano-logger-coverage/builder-final-manifest.json`、`builder-runtime-summary.json`：最终构建参数、哈希和运行记录。
- `build/nano-logger-coverage/reference/`：原始源码、队列元数据及默认固件快照。
- `build/nano-flight-feedback/logger-{gaps-default,peer-abort,status-diagnostic,generation-trace,builder-abort}/`：逐组源码快照、原生日志和恢复审计。
