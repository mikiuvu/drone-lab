# PX4 原生估计输入与回放检查

日期：2026-10-16。环境：四机 Gazebo/PX4 micro 仿真。日志配置及审计工具：`512b3d6`。

## 日志配置

启动器将 `SDLOG_PROFILE` 默认设为 3，记录默认话题和全速估计器回放输入。地图中的 `px4_parameters` 仍可覆盖该值。

旧运行 `run_20261015_234902` 的配置为 131，同时包含默认、估计器回放、视觉避障三组话题。当前 PX4 的 `logged_topics.cpp` 先注册全速回放话题，随后视觉避障配置将 `vehicle_visual_odometry` 的间隔重设为 30 ms；重复话题采用后一次设置的间隔。

0 号旧日志包含 5,797 条外部里程计，但 EKF2 时间戳记录索引了 9,685 次，缺少 3,888 次对应时间戳。原生回放实际执行的匹配报告为 3,887 次错误。这份日志不能用于严格的相同输入对照。

## 时间戳审计

`scripts/audit_ulog_replay.py` 按原生回放的 0.1 ms 时间单位核对 IMU、外部里程计及其他带相对时间索引的传感器记录。有符号相对时间先转换成 Python 整数，避免与无符号时间戳相加时溢出。

输出同时保留整个日志及分析区间的请求数、缺失数和每条缺失记录。分析区间有缺失时返回 1，无缺失时返回 0；空区间和多实例歧义会报错。该检查覆盖时间戳索引输入，直接重放的状态、着地及 GPS 消息另行核验。

使用默认配置的新运行中，0 号首次原生解锁时间为 9.279 s：

```powershell
$ErrorActionPreference = 'Stop'
& ./.venv/Scripts/python.exe -X utf8 scripts/audit_ulog_replay.py build/nano-flight-feedback/native-log-default/ulog/px4_0/log/2026-10-15/16_35_39.ulg --begin-s 9.279 --output build/nano-native-innovation/default-0-coverage.json
if ($LASTEXITCODE -ne 0) { throw 'EKF2 replay input timestamps incomplete' }
```

两组配置为 3 的四机运行中，八份日志从各机首次原生解锁到日志结束，所有已检查的传感器时间戳均存在。起飞前仍有 1–30 条外部里程计索引缺失，保留在完整审计结果中。默认配置运行的四机飞行区间分别索引了 3,243、3,257、3,166、3,183 次外部里程计，均无缺失。

## 外部航向融合对照

`EKF2_EV_CTRL=7` 融合外部位置、高度和速度，15 额外启用航向。在当前 LOCAL_FRD 接口中，未融合外部航向时，PX4 使用两套姿态的滤波差值旋转外部位置；姿态、采样时刻与坐标系的一致性会影响融合输入。

| 运行目录 | EV_CTRL | SDLOG_PROFILE | 合格接触 | 原生着地、停机 | 最大起点水平误差 / m |
|---|---:|---:|---:|---|---:|
| `run_20261016_001508` | 15 | 131 | 2/4，0、3 号 | 四机确认 | 13.679 |
| `run_20261016_001705` | 15 | 131 | 1/4，3 号 | 四机确认 | 8.897 |
| `run_20261016_002506` | 7 | 3 | 2/4，2、3 号 | 四机确认 | 13.834 |
| `run_20261016_003531` | 7 | 3 | 3/4，1、2、3 号 | 四机确认 | 0.0416 |

四组均未出现原生日志的 Navigation failure。实际轨迹各自生成，结果没有证明外部航向融合优于默认配置；默认保留 EV_CTRL=7。所有压力确认均对应独立合格压力，惯性候选按 60 ms 确认等待与压力取消规则处理。

`run_20261016_003531` 在 68.701 s 进入 ABORTED，0 号未取得合格接触，四机均已返航停机。单次 3/4 结果仍未达到全队重复完成要求。

## 原生回放的数值检查

原生回放使用相同 PX4 二进制、输入 ULog 和 EV_CTRL=7。全速采集的 0 号日志仍缺少起飞前的两条外部里程计，时间为 7.242、7.259 s；首次解锁为 9.273 s。

原运行与回放在 10 s 后共有 2,435 个状态采样时间戳，位置状态单分量最大差约 3.738 m，未通过数值一致性检查。因此，飞行区间的时间戳完整不能替代原生回放状态一致性，也不能据此给出配置变化的严格因果结论。

## 验证与记录

启动器相关 18 项 pytest 通过，启动器与审计脚本通过 Python 编译检查。审计脚本在旧日志、完整全速日志、限定时间窗三种真实输入上，分别返回预期的 1、1、0。

- 审计退出码和输入计数：`build/nano-native-innovation/coverage-verification.json`。
- 八份飞行区间检查：`build/nano-native-innovation/armed-coverage-verification.json`。
- 原生回放匹配计数：`build/nano-native-innovation/replay-baseline-rcs/`、`full-rate-baseline/`。
- 回放状态差异：`build/nano-native-innovation/replay-state-comparison.json`。
- 四组闭环、ULog 哈希与原生航向融合标记：`build/nano-native-innovation/trial-summary.json`。
- PX4 日志、回放及融合源码哈希：`build/nano-native-innovation/native-source-manifest.json`。
- 运行源码、压力和停机审计：`build/nano-flight-feedback/native-ev-yaw-default/`、`native-ev-yaw-repeat/`、`native-replay-capture/`、`native-log-default/`。
