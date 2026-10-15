# 电机命令、物理积分与实际转动

日期：2026-10-16。Gazebo Sim 8.15.0、PX4 四机 SITL。

## 记录字段

`--actuation-log` 的只读记录器在电机系统之后、物理积分之前读取现有关节命令，再于 PostUpdate 记录实际速度。PreUpdate 默认优先级为 1，当前电机系统为 0；记录器不创建或修改执行组件。

| 字段 | 含义与单位 |
|---|---|
| `pre_update_velocity_command_rad_s` | 物理积分前关节速度命令，rad/s |
| `velocity_rad_s` | PostUpdate 实际关节速度，rad/s |
| `velocity_command_rad_s` | PostUpdate 剩余速度命令，rad/s |
| `pre_update_force_command_nm` | 物理积分前转动关节力矩命令，N·m |
| `pre_update_velocity_reset_rad_s` | 物理积分前关节速度重置，rad/s |
| `actuator_component_velocity_rad_s` | 模型现有 Actuators 组件中的电机速度数组，rad/s |

缺失组件保留为 null。速度命令的前后采样使用同一仿真步；没有该步 PreUpdate 快照时，前采样字段为 null。

Gazebo 8.15.0 的物理系统会在更新后清零 `JointVelocityCmd`，因此健康旋转时 PostUpdate 命令也可能为零。存在 `JointForceCmd` 时，物理系统优先使用力命令；存在速度重置时忽略速度命令。电机系统优先读取模型 Actuators 组件，否则使用传输话题的消息。

对应上游源码、来源地址和哈希保存在 `build/nano-motor-delivery/reference/`。已安装电机、物理插件和原生 ESC 桥的哈希另存于 `binary-manifest*.json`。

## 历史未离地架次

`run_20261016_053122` 的 2 号、`run_20261016_053640` 的 3 号在原生 11–16 s 均已解锁，lockdown、manual_lockdown 和 force_failsafe 为零。四路原生执行输出分别为约 748–1065 rad/s、707–1200 rad/s，ESC 状态也回读到非零命令；独立 Gazebo 关节速度却低于 1.5e-26 rad/s。

PX4 ESC 状态订阅自身发布的电机话题，不能证明物理电机已执行。两次历史架次没有物理积分前命令与独立话题接收记录，尚不能区分插件接收与物理执行的问题。

## 同步观测与无额外订阅对照

同步观测使用独立进程订阅 `/micro_N/command/motor_speed`，保存当次 partition；三组短起飞诊断还保存发布者、订阅者信息。收包逐条计数，写盘限于约 20 Hz；记录间隔不能用作实际命令传输间隔。两组无额外订阅对照只启用 ECM 记录器。

所有运行使用默认 `arrival` 时钟模式及原模型参数。

| 场景 | 运行目录 | 接触与终态 | 原生停机 |
|---|---|---|---|
| 完整任务，同步话题 | `run_20261016_054517` | 4/4，COMPLETE | 四机确认 |
| 完整任务重复，同步话题 | `run_20261016_054706` | 1/4，180 s 时仍在 LAND_ALL | 0 号未确认；进程已回收 |
| 起飞后计划中止，同步话题 1 | `run_20261016_055203` | ABORTED | 四机确认 |
| 起飞后计划中止，同步话题 2 | `run_20261016_055240` | ABORTED | 四机确认 |
| 起飞后计划中止，同步话题 3 | `run_20261016_055316` | ABORTED | 四机确认 |
| 起飞后计划中止，无额外订阅 1 | `run_20261016_055425` | ABORTED | 四机确认 |
| 起飞后计划中止，无额外订阅 2 | `run_20261016_055506` | ABORTED | 四机确认 |

28 机次均正常起飞，未复现四转子停转。11–16 s 共记录 2,800 个模型样本、11,200 个关节样本，物理积分前速度命令与积分后实际速度的最大绝对差为 4.41e-9 rad/s。模型 Actuators 组件在该窗口均不存在。后五组的力命令、速度重置组件均未出现；前两组未记录这两项，保留为 null。

无额外订阅的两组也没有复现停转，尚无证据把故障归因于订阅者数量。间歇停转仍未解决，正常样本只验证观测链路能够区分前命令、后命令和实际转动。

## 接触与恢复失败

完整任务第一组四次、第二组一次合格压力确认均匹配独立压力记录。两组共八次未确认冲击的等待、恢复参考归零和 ToF 暂停融合检查通过。

第二组 0 号在约 19.2 s 报告原生 roll 健康故障；原生姿态在 20 s 的 roll 约 −174.8°，之后长时间接近 ±180°。伴随程序没有锁存 unrecoverable，水平位置估计持续发散，飞行流程未取得原生停机确认。物理评估显示机体停留在目标附近低处。运行达到 180 s 仿真上限后回收进程，严格恢复检查失败。

原生文本有 Failsafe activated，但约 0.5 s 间隔的 `vehicle_status` 记录未覆盖到 failsafe=true，不能据此判定伴随程序收到并忽略了异常心跳。后续需核对持续倒置的姿态保护、实际心跳与原生着陆判定。

28 份解锁后的 EKF 时间索引输入检查均完整；这不代表原生数值回放已一致，也不抵消恢复失败。

## 验证与数据

Gazebo CTest 3/3 通过。记录器回归覆盖缺失组件、零值、物理阶段间清零、力命令与速度重置、只读性、采样周期、暂停及输出失败。

- `build/nano-motor-delivery/summary.json`：七组运行和命令／实际速度比较。
- `build/nano-motor-delivery/native-outputs.json`：历史失败架次的原生执行输出及锁定状态。
- `build/nano-motor-delivery/ctest-sensors-final.log`：插件回归。
- `build/nano-flight-feedback/motor-*/`：原生日志、源码快照、电机话题及物理记录、压力、恢复和输入完整性检查。
- `build/nano-flight-feedback/motor-delivery-repeat/{native-failure-summary,roll-samples,failsafe-timing}.json`：恢复失败的原生证据。
- `build/nano-motor-delivery/final-process-status.json`：进程退出、观测进程和插件哈希检查。
