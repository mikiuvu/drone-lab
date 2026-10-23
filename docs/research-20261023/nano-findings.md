# Nano 板端算法负载测试结果

测试对象：`build/nano-research-20261023/algorithm_load_test`（SHA256 `790c2d44...a8919`，C906FDV Release，riscv64）。结果数据：`logs/research-20261023/nano-20261023-133926/`。

## 结论

在约 128 MiB 单核板卡上，历史地形（`original_base/terrain_fixed.f32`）的全部工况都在预算内完成：相机 5/15 Hz 时设备 CPU 占用 38.9%/65.3%，相机 30 Hz 时达到 96.8% 且视觉队列开始丢帧，是单核的实际上限。2027 全场地形（`models/terrain/rmuc_2027/terrain_triangles.f32`，776888 面）在加载阶段被内核 OOM 杀死，未进入主循环。

## 设备与输入

板卡 `licheervnano-8455`，`riscv64`，单核（`nproc=1`），内核 5.10.4。内存 130868 KiB，无 swap。上传后逐文件核对 `sha256sum` 一致，远端 `uname -m` 为 `riscv64`。

历史位姿来自 `simulation/runtime/run_20261010_201647/companion_{0..3}.jsonl` 中全部 `accepted` 的 `visual_pose`，共 2093 条（502/535/440/616），导出为 `poses.json`。图像 fixture：`benchmark-armor.ppm`、`camera/camera_0.ppm`（均 640x360）。

内存预算（`manifest.json` 的 `memory_budget`）：`commit_limit_kib=65432`、`overcommit_memory=0`、`overcommit_ratio=50`、`swap_total_kib=0`。无 swap 时提交上限约为物理内存的 50%，是全图加载的硬约束。

## 方法

每个工况在独立通道上运行测试进程，另起一个通道以 `/bin/sh`（busybox）每 1 秒采样设备级资源：`/proc/stat`、`/proc/loadavg`、`/proc/meminfo`、`/sys/class/thermal/thermal_zone*/temp`、`cpufreq`，以及测试进程的 `/proc/<pid>/stat` 与 `/proc/<pid>/status`。测试前采集 10 秒空闲基线。停止监视用本工况的停止文件，超时按 PID 精确终止，不使用 `pkill`。

CPU 口径：

- 设备 CPU（`device_cpu_percent`）：`/proc/stat` 首行差分。`busy = total - idle - iowait`，`total` 已排除 `guest`/`guest_nice`（它们已计入 `user`/`nice`），结果为整机 0–100，不再按核数缩放。
- 进程 CPU（`process_cpu_percent`）：程序 `getrusage` 口径，100% 表示占满一个核。
- 进程容量占比（`process_cpu_capacity_percent`）：进程 CPU 除以核数，表示占整机算力比例。
- `loadavg` 单独记录，不作为 CPU 指标；本次 1 分钟负载在 3.65–4.38 之间，而实测设备 CPU 为 38.9%–96.8%。

板端无 `cpufreq` 节点，`cpufreq_available=false`；无 `getconf`，`clk_tck` 标记 `unavailable`，进程 CPU 以程序 `getrusage` 为准，`/proc` 采样仅保留原始 ticks（`proc_ticks_delta`）。

## 工况结果

设备 CPU 为整个测试窗口的 `/proc/stat` 差分值；稳态 RSS 取程序最后一次 1 秒资源采样，峰值 RSS 为内核高水位（`VmHWM`/`ru_maxrss`）。

| 工况 | 时长 s | 相机 Hz | 设备 CPU % | 进程 CPU % | 容量 % | 峰值 RSS KiB | 稳态 RSS KiB | 温度 °C 峰值 | 跳过 tick | 超时 tick |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| camera5 | 60 | 5 | 38.93 | 33.86 | 33.86 | 20772 | 15376 | 62.3 | 0 | 381 |
| camera15 | 60 | 15 | 65.32 | 59.21 | 59.21 | 20916 | 15560 | 63.0 | 1 | 830 |
| camera30 | 60 | 30 | 96.81 | 90.44 | 90.44 | 20800 | 15560 | 64.4 | 2 | 362 |
| sustained15 | 180 | 15 | 67.09 | 60.14 | 60.14 | 20884 | 16984 | 64.4 | 2 | 2581 |
| allbranches15 | 60 | 15 | 76.08 | 68.59 | 68.59 | 20920 | 15340 | 64.0 | 1 | 820 |
| uwbfallback15 | 60 | 15 | 66.04 | 59.89 | 59.89 | 20772 | 15668 | 64.0 | 2 | 554 |
| rmuc2027_15 | — | 15 | 89.86 | — | — | — | — | 63.0 | — | — |

空闲基线设备 CPU 4.67%，温度 62.3 °C。“超时 tick”指完成滞后超过 10 ms 的 tick 计数（`deadline_misses`），随运行时长累积，不等同于丢帧。

温度最高 64.4 °C，`cooling_device0` 状态始终为 0，未触发主动散热或降频。单秒窗口的设备 CPU 峰值在 87.7%–100% 之间，camera15、camera30 与 sustained15 的个别秒触及 100%。

## 阶段频率与计时

调度频率全部达标：`imu_predict` 100 Hz、`flow_height` 50 Hz、`navigation` 50 Hz、`uwb_peers` 20 Hz、`team_impact_schedule` 10 Hz、`tof_frame` 45.0 Hz（3 路传感器 × 15 Hz）。导航间隔均值 20.0 ms、p95 约 20.5 ms、最大 28–33 ms。

视觉阶段（每帧毫秒，括号内为实测频率）：

| 工况 | vision_detect_pnp 均值/p95/最大 | vision_pixel 均值/p95/最大 | 实测视觉 Hz |
| --- | --- | --- | --- |
| camera5 | 18.53 / 53.49 / 79.34 | 18.39 / 42.12 / 50.01 | 5.00 |
| camera15 | 18.99 / 40.98 / 78.24 | 17.07 / 41.77 / 58.32 | 15.00 |
| camera30 | 19.47 / 28.62 / 78.83 | 15.05 / 23.72 / 66.60 | 27.17 |
| sustained15 | 19.30 / 39.43 / 80.86 | 17.30 / 39.41 / 54.88 | 15.00 |
| allbranches15 | 19.67 / 41.96 / 75.09 | 16.65 / 35.25 / 59.88 | 15.00 |
| uwbfallback15 | 19.24 / 44.44 / 78.08 | 17.16 / 40.62 / 63.88 | 15.00 |

单帧视觉计算上限在 115–128 ms 之间（`vision_pipeline.max_compute_ms`）。相机 5/15 Hz 时 `pending_replaced=0`，提交与完成数一致；相机 30 Hz 时提交 1799、完成 1630，`pending_replaced=168`，即 168 帧因工作线程未空而被替换，实测视觉频率降到 27.17 Hz，无法维持 30 Hz。

ToF 队列（`tof_queue`，bounded，仅导航槽位启动）：各工况提交约 2702、替换 1、取用 2701，历史地形有效像素 `32/12/12`，全部被接受。帧龄均值 3.34 ms，导航后启动均值约 0.05 ms、结束均值约 3.9–4.8 ms。

周期运动模型：60 秒工况 `periodic_model_ready=false`；180 秒 sustained15 在运行中转为 `true`（`periodic_period_s=5.605`）。后台工作线程单次最大耗时随负载上升，从 camera5 的 435 ms 增至 camera30 的 1337 ms，无丢弃、无失败。

## 分支覆盖

`allbranches15` 使用 `--vision-branches all`：`green_guide` 阶段 15 Hz、接受 900、均值 7.88 ms、p95 11.53 ms、最大 40.93 ms。相对 camera15 的设备 CPU 增量 10.76 个百分点。

`uwbfallback15` 叠加 UWB 回退：`uwb_fallback` 阶段 20 Hz、接受 3600、均值 0.32 ms。相对 camera15 的设备 CPU 增量 0.72 个百分点。

## 全图 2027 地形内存结论

`rmuc2027_15` 未产生结果文件，进程在主循环前被内核 OOM 杀死。`dmesg` 证据（`rmuc2027_15.kernel.log`）：

```
oom-kill:constraint=CONSTRAINT_NONE,nodemask=(null),task=algorithm_load_,pid=1727,uid=0
Out of memory: Killed process 1727 (algorithm_load_) total-vm:114440kB, anon-rss:84760kB, file-rss:4kB, shmem-rss:0kB, UID:0 pgtables:232kB
```

启动阶段约 9 秒，进程匿名 RSS 峰值 84760 KiB、虚拟峰值 114440 KiB；外部 `/proc` 采样到的进程高水位 70556 KiB、RSS 峰值 64380 KiB，`MemAvailable` 最低 24748 KiB。峰值已超过 `CommitLimit` 65432 KiB，而 `MemAvailable` 仍有约 24 MiB，说明限制来自提交上限而非空闲内存。

全图 776888 面，`TerrainMap` 需要原始三角形数组、重排副本、BVH 节点与阈值数组，合计远超单核可用提交额度。历史地形 4.49 MB 对应稳态 RSS 约 15–17 MiB、峰值约 21 MiB，可作为放缩参考。该工况失败不影响其余 6 个历史地形工况。

## 覆盖范围与限制

本次测量的是算法内核的定时回放，覆盖定位、视觉、导航与撞击任务调度。不覆盖相机采集、ToF 硬件 I/O、MAVLink 传输与执行机构动作，也不覆盖 UWB 无线链路。

板端 ToF 帧由地形射线投射生成（固定起点 `{10.6, 0, 1.44}`），UWB 为融合调用而非真实射频输入；因此 `accepted`/`rejected` 反映算法内部门控，不代表真实传感器或对抗条件下的检测正确性。测试为开环负载，无真值反馈控制。

`deadline_misses`、`skipped_ticks` 与队列替换计数由程序自身统计，依赖其计时实现；设备 CPU 与内存来自内核接口差分与高水位，两者来源独立。

## 机器可读产物

目录 `logs/research-20261023/nano-20261023-133926/`：

- `summary.json`：`idle`、`device`、`memory_budget`、`build` 与 7 个工况的 `profiles` 数组；每个工况含 `spec`、`argv`、`binary`（`elapsed_s`、`process_cpu_percent`、`process_cpu_capacity_percent`、`peak_rss_kib`、`steady_rss_kib`、`valid_tof_cells`、`tof_queue`、`vision_pipeline`、`navigation_intervals`、`periodic_*`、`resources_trace`）、`stages`（每阶段 `samples`、`accepted`、`rejected`、`mean_ms`、`p50_ms`、`p95_ms`、`p99_ms`、`max_ms`、`achieved_hz`）、`device`（`device_cpu_percent`、区间统计、`loadavg_1min_max`、`mem_available_kib_min`、`thermal_c_max`、`cpufreq_*`、`proc_*`）、`cold_start`（仅 rmuc2027_15）与 `verification`。
- `manifest.json`：输入 SHA256、`build`、`remote_checksums`、`memory_budget`、各工况命令行与运行记录。
- `<profile>.json`：程序原始输出；`<profile>.resources.log`：板端原始资源采样；`<profile>.test.log`：程序 stdout/stderr；`rmuc2027_15.kernel.log`：内核 OOM 证据。
- `device.txt`：板端 `uname`、`/proc/cpuinfo`、`/proc/meminfo`、`/proc/stat`、`free`、`df`、thermal。

`verification` 逐工况判定：必需阶段样本、`exit_status==0`、`host_arch=riscv64`、有限值、时长完成、ToF 覆盖，以及 allbranches 的 `green_guide` 接受数与 uwbfallback 的 `uwb_fallback` 接受数。历史地形 6 个工况 `ok=true`，`rmuc2027_15` 为 `ok=false`（`kernel_oom`）。

复现：`python scripts/run_research_nano.py --output <dir>`（全矩阵），或 `--resume --profile <name>` 重跑单个工况并合并到同一 `summary.json`。
