# 终端横向误差与过板退避

日期：2026-10-15。场景：Gazebo/PX4，`rmuc_2026_sentry26`，四机 `team_contact`。

## 横向误差分解

`run_20261015_070018` 中，0、3 号首次提交时的横向位置估计偏差分别为 −55.5、+133.8 mm。两机的瞄准点穿面误差约为 +9.0、−16.3 mm，机体穿面误差却为 +133.2、−175.9 mm。

0 号提交后 0.1 s，伴随程序的世界 Y 速度参考约为 −0.331 m/s，评估轨迹的横向速度约为 +0.078 m/s；0.3 s 时参考达到 −1.483 m/s，轨迹横向速度约为 −0.045 m/s。3 号提交时的估计横向速度约为 −0.371 m/s，轨迹横向速度约为 −0.217 m/s，较大的正向参考在末段才出现。

横向定位偏差、速度估计与运动响应需要分别检查。上述参考由伴随机体系参考按估计航向换算；轨迹速度用 40 ms 中心差分计算。它们不能单独确定飞控内部的延迟或执行器模型。

分析命令：

```powershell
$ErrorActionPreference = 'Stop'
& ./.venv/Scripts/python.exe -X utf8 scripts/summarize_terminal_response.py simulation/runtime/run_20261015_070018 --output build/nano-plane-exit/baseline-response.json
if ($LASTEXITCODE -ne 0) { throw 'Terminal response analysis failed' }
```

输出包含每次提交的 Y 定位偏差，以及提交前 0.3 s 至后 0.6 s 的估计、轨迹和速度参考。

## 可选过板退避

`team_mission.dive_attack` 参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `passed_plane_retreat_enabled` | false | 启用沿任务航向的过板退避 |
| `exit_margin_m` | 0.25 | 超过提交目标平面的距离，m；范围 0–5 m |

启用后，导航可用、本机沿任务航向超过目标平面裕量、且尚未确认接触时，立即退出前推并回到入口。接触反馈已收到时不触发该空间条件；俯冲时间上限始终生效。判定只使用本机估计、已提交目标点和任务航向。

## 接触时窗与闭环

五组历史运行中的 10 次已确认接触，其反馈记录顺序均早于首次超过 0.25 m 裕量的候选退避时刻。数据：`build/nano-plane-exit/contact-window.json`。这项回放核验不代替新轨迹的接触测试。

启用过板退避的闭环运行 `run_20261015_071455`：

| 成员 | 任务内接触 | 过板退避 | 停止时飞控状态 |
|---:|---|---|---|
| 0 | 确认 | 无 | 着地、停机 |
| 1 | 确认 | 无 | 着地、停机 |
| 2 | 确认 | 无 | 着地、停机 |
| 3 | 超时 | 0.50 s、0.52 s | 未确认着地，仍解锁 |

3 号的两次空间退避分别在估计超过板面 0.288 m、0.380 m 时触发；另一次尝试在 1.22 s 由时间上限退出。退避动作经过导航发布周期离散执行。

3 号返航于约 71.25 s 接近地面，随后继续位移并下落。全队保持 LAND_ALL，降落超时后没有返回返航阶段，也没有重新发出 ARM/OFFBOARD 请求。恢复报告 `all_stopped=false`，`--require-stopped` 返回失败，因此过板退避保留为默认关闭的可选策略。单次运行中的接触成员变化不足以证明成功率改善。

## 验证与数据

- 完整 CTest 14/14、无 OpenCV CTest 13/13 通过。
- 过板几何测试覆盖裕量边界、旋转航向、非有限值和溢出。
- Nano riscv64 musl 静态 `test_intercept` 返回 0，临时程序已移除。记录：`build/nano-plane-exit/board-tests/result.json`。
- 闭环源码快照、配置与二进制哈希：`build/nano-plane-exit/enabled/source-manifest.json`。
- 接触、估计响应、退避事件与 ULog 汇总：同目录的 `trial-report.json`、`response.json`、`plane-exit-events.json`、`recovery.json`。
