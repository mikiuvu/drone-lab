# 起飞执行反馈与中止返航

日期：2026-10-15。平台：Gazebo/PX4 四机仿真、LicheeRV Nano riscv64。

## 未离地证据

`run_20261015_084412` 的 2 号实际高度始终约为 0.516 m。PX4 收到向上 0.6 m/s 的速度参考，11 s 附近推力约为 0.56，20 s 达到 1.0，电机输出达到约 1200 rad/s；原生解锁状态有效，lockdown、manual_lockdown 和 force_failsafe 均未置位。

8–20 s 的 `sensor_combined` 采样间隔 P99 和最大值均为 4 ms，没有超过 20 ms 的间隔。STALE 告警不能单独证明这次未离地由传感器断流造成。

PX4 仿真的 ESC 反馈订阅其发布的 `/micro_N/command/motor_speed`，其中的转速是命令回读。实际转子关节速度需要独立读取 Gazebo 物理状态。该历史运行没有记录关节速度，未离地原因尚未确定。

原生执行输出、速度参考和 IMU 间隔分析保存在 `build/nano-takeoff-debug/probe.json`。

## 执行记录器

启动器增加 `--actuation-log`，在运行目录生成 `actuation_0.jsonl` 等文件。默认关闭。记录器通过世界的模型 include 挂载，生成的模型文件无需修改。

```powershell
$ErrorActionPreference = 'Stop'
wsl -d Ubuntu-22.04 --exec python3 '/mnt/d/Robomaster code/drone/simulation/launch_linux.py' start --mission team_contact --companions --fly --no-tof-rays --actuation-log
if ($LASTEXITCODE -ne 0) { throw 'Simulation launch failed' }
```

记录包含仿真时刻、机号、模型和四个转子关节的 `JointVelocity`、`JointVelocityCmd`。读取发生在 PostUpdate，使用只读 ECM，不创建反馈组件，不发送控制命令。缺失组件保存为 null，实际零值保存为零。默认频率为 20 Hz，最高 100 Hz；输出必须是绝对路径且不能覆盖已有文件。

PostUpdate 时 `JointVelocityCmd` 可能已被物理系统清零；健康旋转时也观察到零值。判断命令是否到达应同时检查传输命令或 PX4 ULog，不能只看这个字段。模型的转速减速比为 10，电机命令约 600 rad/s 对应仿真关节约 60 rad/s。

| 场景 | 运行目录 | 实际峰值高度 / m | 减速比校正后转速比中位数 |
|---|---|---:|---:|
| JointStatePublisher 对照 1 | `run_20261015_181736` | 1.504–1.515 | 1.0007–1.0010 |
| JointStatePublisher 对照 2 | `run_20261015_181855` | 1.505–1.507 | 1.0005–1.0012 |
| 只读记录器 | `run_20261015_182628` | 1.510–1.514 | 1.0001–1.0010 |

三组均未观察到高命令下四转子同时停转，未复现历史未离地问题。前两组挂载过会创建反馈组件的 JointStatePublisher；第三组使用只读记录器。临时模型改动均有逐字节恢复校验。更早的两次观察使用了错误的 Gazebo partition，没有获得执行观测。

原始传输命令、关节速度、物理轨迹、汇总和模型恢复记录位于 `build/nano-takeoff-debug/{actuator-03,actuator-04,passive-01}/`。外部观察进程应使用 `simulation/runtime/manager.json` 中该次运行的 `gazebo_environment`。

## 未起飞成员导致的返航阻塞

`run_20261015_183521` 于 8.021 s 进入 ABORT_RETURN。0、1 号从未解锁，其伴随程序各发出 197 次恢复阶段的 ARM 请求，持续至约 179.7 s；2、3 号已经起飞，等待未起飞成员满足返航高度条件，180 s 上限处仍未着地停机。

伴随程序现在同时检查本机 `phase` 与全队 `mission_phase`。收到 RETURN_ALL、ABORT_RETURN、LAND_ALL、COMPLETE 或 ABORTED 后锁存禁止再次请求 ARM/OFFBOARD；本机收到 HOLD 也适用。未解锁成员只可在全队 TAKEOFF 阶段请求启动。

协调器将返航阶段具有新鲜“已着地、未解锁”反馈的成员保留在地面，排除在空中路线屏障之外。所有成员均已停机时直接进入降落确认；中止终态要求全队新鲜着地停机反馈，正常 COMPLETE 仍保留返航位置条件。

## 闭环验收

两组均将起飞阶段期限设为 3 s，并清空去程航点，以触发中止。时间取 `team_mission.jsonl` 的首次阶段记录。

| 场景 | 运行目录 | ABORT_RETURN / s | LAND_ALL / s | ABORTED / s |
|---|---|---:|---:|---:|
| 四机起飞后中止 | `run_20261015_185014` | 7.821 | 9.421 | 16.601 |
| 0 号留地、其余三机起飞 | `run_20261015_185329` | 16.441 | 17.161 | 23.841 |

第二组在 WAIT_READY、2.901 s、0 号确认未解锁时暂停其伴随进程 10 秒现实时间。全队至 13.441 s 才进入 TAKEOFF，因此暂停没有跨越中止时刻。0 号恢复后仍处于飞控连接恢复状态，原生日志全程未解锁，物理高度范围为 0.515995–0.515999 m；其余三机均实际起飞。这组验证了留地成员与空中成员混合的返航条件。

两组共八个机次均取得 PX4 原生着地、未解锁以及伴随程序停止时未解锁确认。进入返航恢复后 ARM/OFFBOARD 请求数均为零。测试没有进入接触阶段。

`scripts/summarize_recovery_ulog.py --require-stopped` 同时检查终态、原生着地停机、失效后的重启请求和从返航开始的重启请求。报告保留首次原生解锁、首次离地时刻；从未解锁成员的对应值为 null。

原生 ULog、源码快照、二进制哈希和严格恢复汇总位于 `build/nano-flight-feedback/{include-01,include-02,grounded-10s}/`；跨运行检查位于 `build/nano-takeoff-debug/recovery-audit.json`。

## 回归

| 检查 | 结果 |
|---|---|
| 完整 CTest | 16/16 通过，含 10 项 UDP/MAVLink 收包回归 |
| 无 OpenCV CTest | 15/15 通过 |
| Gazebo 插件测试 | 3/3 通过 |
| 启动器世界生成测试 | 2/2 通过 |
| 恢复报告测试 | 5 项通过，覆盖中止和正常返航期间重启、地面成员、原生未停机拒绝 |
| Nano 静态机群测试 | 返回 0，覆盖 1/4/6/12 架及混合地面成员的返航 |

旧恢复报告未拒绝返航至降落之间的重启请求，同一回归在增加返航阶段检查后通过。Nano 静态测试二进制 SHA256 为 `c7f9a18fec82cdebba752208447ab252f8202c35299838d9729acdda4ed04832`，启动 ID 为 `0bbba2cb-823b-4477-86c2-3bf1f140a056`；记录位于 `build/nano-takeoff-debug/board-tests/result.json`。
