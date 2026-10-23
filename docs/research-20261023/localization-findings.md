# 四机定位与通信消融结果

实验程序：`simulation/research/localization_study.cpp`（目标 `localization_study`）。
结果数据：`build/research-20261023/localization-transit.json` 与 `build/research-20261023/localization-transit.trace.jsonl`。
共享场景与噪声配置：`simulation/research/scenarios.json`；共享 ToF 误差模型：`simulation/research/tof_error_model.hpp`。

## 结论

在本次配置下，协同 UWB 更新对定位结果的影响取决于本地地图解是否已经成立，而不是稳定带来提升。

当 ToF+IMU+高度辅助组的地图解未收敛时，开启通信会把估计拉回绝对参考：transit 窗 `noise` 场景 RMSE 由 0.2613 m 降到 0.0613 m，`combined` 由 0.5397 m 降到 0.0855 m，`blackout` 由 1.1730 m 降到 0.0944 m，有效比例由 0.77 附近升到 0.999。这类收益的实质是补上缺失的绝对参考，而不是在已经收敛时提高精度。

当本地解已经成立时，估计器以 `independent_map_preferred` 为由拒绝绝大多数队友更新，接受率仅 0.06%–6.67%；在光流组里，少数被接受的更新还会带来轻微劣化：transit 窗 6 个场景的 RMSE 配对差值（on − off）为 +0.0001 至 +0.0235 m，`false_valid_fraction` 与 `common_drift_rmse_m` 同步上升。

`adverse` transit 是本次最差工况：ToF+IMU+高度辅助组 off 为 2.1382 m、on 为 4.2849 m，其中种子 6151 的 on 达 9.5533 m。大测距噪声下少量被接受的 UWB 更新会与地图解互相牵引，因此不能把"开启通信"当作单调改善。

遮挡方面，真实地面机器人在巡航段遮住 4.08% 的 ToF 射线，在降落段只有 0.52%：20° 下俯、4 m 量程的射线扇面在 2.2 m 巡航高度上多数越过 0.4–1.1 m 高的地面占位，只有近距离通过和下降段才会产生遮挡。托盘平台上的非比赛站位（`pressure_robots`）若纳入，降落段遮挡会升到 84.9%，本次未使用。

## 方法与输入

轨迹为一次真实 RMUC2027 运行 `simulation/runtime/run_20261016_215504/truth_evaluation_only.jsonl`：4 车，每车 3332 点，跨度 103.98 s，日志间隔 20–25 ms，插值到 10 ms。速度与加速度由插值位置做跨度 0.1 s 的中心差分得到，避免日志间隔抖动造成伪加速度。

地形为 `simulation/models/terrain/rmuc_2027/terrain_triangles.f32`（776888 面，加载后 728933 面）。传感器与估计器先验使用同一份网格与外参。

| 项 | 取值 |
| --- | --- |
| 主循环 | 100 Hz |
| IMU | 100 Hz，加速度固定偏差 0.008 m/s² + 白噪声 0.02 m/s²，yaw 固定偏差 0.01 rad + 白噪声 0.002 rad，roll/pitch 恒为 0 |
| 高度辅助 | 50 Hz，带噪真值高度，白噪声 0.01 m，来源 `SIM_HEIGHT_ASSIST` |
| 光流 | 50 Hz，机体系速度，比例 1.005，白噪声 0.015 m/s |
| ToF | 15 Hz × 3 传感器，8×8，FOV 45°，安装 yaw 0/±120°，半径 0.055 m，pitch_down 0.349 rad，量程 0.02–4.0 m，未归一化射线保持轴向深度 |
| UWB | 20 Hz，距离噪声 0.15 m，方位噪声 0.08 rad，无固定基站 |
| 初值 | 位置偏移 (+0.08, −0.06) m，yaw 偏移 +0.02 rad，协方差对角 0.02，运行中不重置真值 |
| 评分窗 | 窗内第 3 s 起 |
| 窗口 | `transit` 起 7 s 长 24 s（评分 [10, 31] s）；`landing` 起 90 s 长 10 s（评分 [93, 100] s） |

ToF 逐像素误差直接调用共享模型 `MakeTofNoiseDevice` / `MakeTofFrameError` / `ApplyTofPixel`：设备固定偏差与比例按 (种子, 车辆, 传感器) 整段恒定，帧公共误差同帧 64 像素共享，逐像素白噪声标准差为 `noise_std_m + range_sigma_per_m · d`，越界与丢弃一律标记无效且不限幅。随机量由键派生，回调顺序不影响同帧结果，因此真实 gateway 与离线实验对同一 (种子, 车辆, 传感器, 帧时间) 产生同一组误差。

六个场景与三档剖面：

| 场景 | 剖面 | 机器人 | 说明 |
| --- | --- | --- | --- |
| ideal | ideal | 0 | 无噪声、无占位 |
| noise | nominal | 0 | 仅测距误差 |
| occluders | ideal | 9 | 理想测距 + 地面占位 |
| combined | nominal | 9 | 测距误差 + 地面占位 |
| adverse | adverse | 9 | 恶劣测距 + 地面占位 |
| blackout | nominal | 9 | 叠加 0 号机 ToF 8–16 s 黑障 |

通信条件：`on` 在本地传感器更新后发布估计器快照、再融合全部队友链路；`off` 不调用 `fuseUwb`；`intermittent` 额外 30% 丢包并在窗内 8–16 s 断开 0 号机链路。队友位姿、协方差与参考来源字段全部来自各估计器快照，不含真值。三个通信条件共用同一组量测：同 (窗口, 场景, 种子) 的输入指纹 FNV-1a 完全一致（契约测试逐组校验）。

剖面取值来自 `scenarios.json`：nominal 为 `noise_std_m=0.01, range_sigma_per_m=0.005, bias_bound_m=0.015, scale_bound=0.005, frame_bias_std_m=0.005, drop_probability=0.03`；adverse 为 `0.025, 0.015, 0.03, 0.01, 0.015, 0.1`；ideal 六项全为 0。

## 传感器模型

每个 (窗口, 场景) 聚合 3 个种子。`rays` 为全部射线，其余为占该窗口的比例。

| 窗口 | 场景 | rays | 机器人遮挡 % | 无回波 % | 丢弃 % | 越界 % | 黑障像素 | 有效返回 % |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| transit | ideal | 831744 | 0.00 | 71.5 | 0.00 | 0.18 | 0 | 28.4 |
| transit | noise | 831744 | 0.00 | 71.5 | 0.85 | 0.62 | 0 | 27.1 |
| transit | occluders | 831744 | 4.08 | 70.6 | 0.00 | 2.61 | 0 | 26.8 |
| transit | combined | 831744 | 4.08 | 70.6 | 0.88 | 2.82 | 0 | 25.7 |
| transit | adverse | 831744 | 4.08 | 70.6 | 2.95 | 2.65 | 0 | 23.8 |
| transit | blackout | 831744 | 4.08 | 70.6 | 0.81 | 2.81 | 18822 | 23.5 |
| landing | ideal | 347904 | 0.00 | 16.7 | 0.00 | 2.00 | 0 | 81.3 |
| landing | noise | 347904 | 0.00 | 16.7 | 2.51 | 13.32 | 0 | 67.5 |
| landing | occluders | 347904 | 0.52 | 16.5 | 0.00 | 2.00 | 0 | 81.5 |
| landing | combined | 347904 | 0.52 | 16.5 | 2.51 | 13.32 | 0 | 67.7 |
| landing | adverse | 347904 | 0.52 | 16.5 | 8.33 | 19.79 | 0 | 55.4 |
| landing | blackout | 347904 | 0.52 | 16.5 | 2.51 | 13.32 | 0 | 67.7 |

巡航段无回波比例达 70.6%，因为 20° 下俯、4 m 量程的扇面在 2.2 m 高度上大部分方向既触不到地面也触不到墙。降落段高度降到约 1.617 m 后无回波降到 16.5%，有效返回升到 67.5%–81.5%，但越界比例同步升高（ideal 已有 2.00%）：机体贴近地面时下俯射线在极近距离触地，低于 0.02 m 下限被判无效。

对"有效且未被机器人遮挡"的返回，实测深度减真实几何深度的分布（3 个种子均值）：

| 窗口 | 场景 | 均值 m | 标准差 m | P50 m | P95 m | 绝对最大 m |
| --- | --- | --- | --- | --- | --- | --- |
| transit | ideal | +0.00000 | 0.00000 | +0.00000 | +0.00000 | 0.00000 |
| transit | noise | −0.00091 | 0.02821 | −0.00106 | +0.04590 | 0.14264 |
| transit | occluders | +0.00000 | 0.00000 | +0.00000 | +0.00000 | 0.00000 |
| transit | combined | −0.00077 | 0.02887 | −0.00083 | +0.04694 | 0.14264 |
| transit | adverse | −0.00416 | 0.07669 | −0.00382 | +0.12233 | 0.36087 |
| transit | blackout | −0.00035 | 0.02889 | −0.00042 | +0.04743 | 0.14264 |
| landing | ideal | +0.00000 | 0.00000 | +0.00000 | +0.00000 | 0.00000 |
| landing | noise | +0.00208 | 0.01529 | +0.00220 | +0.02587 | 0.11437 |
| landing | occluders | +0.00000 | 0.00000 | +0.00000 | +0.00000 | 0.00000 |
| landing | combined | +0.00212 | 0.01518 | +0.00221 | +0.02580 | 0.11412 |
| landing | adverse | +0.01031 | 0.03704 | +0.01054 | +0.06558 | 0.31029 |
| landing | blackout | +0.00212 | 0.01518 | +0.00221 | +0.02580 | 0.11412 |

ideal 与 occluders 误差恒为 0，说明零噪声下测量等于几何深度；nominal 与 adverse 的标准差分别约为 0.028/0.015 m 与 0.077/0.037 m，与剖面一致。均值接近 0 而单边最大可达 0.36 m，说明误差以对称白噪声为主，极端值来自帧公共项与设备固定项。

## 通信结果

每个 (窗口, 场景, 通信条件) 为 3 个种子 × 2 个融合模式的均值。`uwb 调用` 为单次运行的调用次数。

| 窗口 | 场景 | 通信 | 调用/次 | 接受/次 | 接受率 % | 主要拒绝原因 |
| --- | --- | --- | --- | --- | --- | --- |
| transit | ideal | on | 5772 | 3.7 | 0.06 | independent_map_preferred=34488, reference_stale=99, reference_already_seen=20 |
| transit | noise | on | 5772 | 11.2 | 0.19 | independent_map_preferred=34387, reference_stale=99, reference_already_seen=51 |
| transit | occluders | on | 5772 | 102.8 | 1.78 | independent_map_preferred=33297, reference_path_loop=263, reference_already_seen=236 |
| transit | combined | on | 5772 | 94.5 | 1.64 | independent_map_preferred=33294, reference_already_seen=291, reference_path_loop=261 |
| transit | adverse | on | 5772 | 85.8 | 1.49 | independent_map_preferred=29157, reference_stale=2451, uwb_innovation_gated=1671 |
| transit | blackout | on | 5772 | 385.2 | 6.67 | independent_map_preferred=29603, reference_already_seen=1259, reference_path_loop=1163 |
| transit | ideal | intermittent | 3399 | 2.3 | 0.07 | independent_map_preferred=20310, reference_stale=65 |
| transit | noise | intermittent | 3399 | 19.7 | 0.58 | independent_map_preferred=20115, reference_stale=72 |
| transit | occluders | intermittent | 3399 | 76.7 | 2.26 | independent_map_preferred=19453, reference_path_loop=183 |
| transit | combined | intermittent | 3399 | 89.7 | 2.64 | independent_map_preferred=19280, reference_path_loop=219 |
| transit | adverse | intermittent | 3399 | 55.7 | 1.64 | independent_map_preferred=18189, reference_stale=932 |
| transit | blackout | intermittent | 3399 | 57.8 | 1.70 | independent_map_preferred=17422, reference_stale=1327 |
| landing | ideal | on | 2412 | 34.8 | 1.44 | independent_map_preferred=13681, reference_stale=288 |
| landing | noise | on | 2412 | 14.7 | 0.61 | independent_map_preferred=14150, reference_already_seen=100 |
| landing | occluders | on | 2412 | 38.0 | 1.58 | independent_map_preferred=12928, reference_stale=642, uwb_innovation_gated=288 |
| landing | combined | on | 2412 | 15.3 | 0.64 | independent_map_preferred=14110, reference_stale=141 |
| landing | adverse | on | 2412 | 8.8 | 0.37 | independent_map_preferred=14125, reference_stale=105 |
| landing | blackout | on | 2412 | 15.3 | 0.64 | independent_map_preferred=14110, reference_stale=141 |

`off` 条件三列恒为 0，程序在该条件下完全不调用 `fuseUwb`。`intermittent` 的调用数在 transit 为 `on` 的 58.9%（3399/5772），在 landing 为 70.7%（1704/2412）：30% 丢包作用于全部 12 条有向链路，0 号机链路断链只覆盖窗内 8–16 s，transit 受影响的比例因此高于 landing。两种条件下的量测本身完全相同，只有丢包判定不同。

拒绝原因中 `independent_map_preferred` 占绝对多数（29 157–34 488 次），即本地地图解完整时估计器优先保留独立定位。只有 `blackout` transit 的接受率升到 6.67%，因为 0 号机黑障期间链路对端失去结构解，门控不再拦下更新。

## 精度结果

按种子配对的 RMSE（on − off，负值表示开启通信更好）与有效性；括号内为逐种子差值。

| 窗口 | 场景 | 融合 | RMSE off m | RMSE on m | RMSE int m | Δ(on−off) m | Δ(int−off) m | 有效 off | 有效 on | 最长失效 off s | 最长失效 on s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| transit | ideal | ToF+IMU+高度辅助 | 0.1099 | 0.0373 | 0.0374 | −0.0726 (0, −0.2178, 0) | −0.0725 | 0.972 | 1.000 | 0.98 | 0.03 |
| transit | ideal | ToF+IMU+高度辅助+光流 | 0.0188 | 0.0190 | 0.0193 | +0.0001 | +0.0004 | 1.000 | 1.000 | 0.00 | 0.00 |
| transit | noise | ToF+IMU+高度辅助 | 0.2613 | 0.0613 | 0.0733 | −0.2000 | −0.1880 | 0.954 | 0.999 | 2.92 | 0.03 |
| transit | noise | ToF+IMU+高度辅助+光流 | 0.0242 | 0.0256 | 0.0248 | +0.0014 | +0.0005 | 1.000 | 1.000 | 0.00 | 0.00 |
| transit | occluders | ToF+IMU+高度辅助 | 0.2056 | 0.0676 | 0.0701 | −0.1380 | −0.1355 | 0.940 | 0.999 | 1.56 | 0.04 |
| transit | occluders | ToF+IMU+高度辅助+光流 | 0.0199 | 0.0315 | 0.0319 | +0.0116 | +0.0120 | 1.000 | 1.000 | 0.00 | 0.00 |
| transit | combined | ToF+IMU+高度辅助 | 0.5397 | 0.0855 | 0.0941 | −0.4543 | −0.4456 | 0.942 | 0.999 | 2.57 | 0.04 |
| transit | combined | ToF+IMU+高度辅助+光流 | 0.0242 | 0.0373 | 0.0367 | +0.0131 | +0.0125 | 1.000 | 1.000 | 0.00 | 0.00 |
| transit | adverse | ToF+IMU+高度辅助 | 2.1382 | 4.2849 | 0.8481 | +2.1466 (−1.8168, −0.0635, +8.3201) | −1.2901 | 0.749 | 0.889 | 7.25 | 7.04 |
| transit | adverse | ToF+IMU+高度辅助+光流 | 0.0390 | 0.0512 | 0.0523 | +0.0121 | +0.0132 | 1.000 | 1.000 | 0.00 | 0.00 |
| transit | blackout | ToF+IMU+高度辅助 | 1.1730 | 0.0944 | 1.5111 | −1.0787 | +0.3380 | 0.774 | 0.999 | 8.93 | 0.04 |
| transit | blackout | ToF+IMU+高度辅助+光流 | 0.0260 | 0.0495 | 0.0377 | +0.0235 | +0.0116 | 1.000 | 1.000 | 0.00 | 0.00 |
| landing | ideal | ToF+IMU+高度辅助 | 0.7072 | 0.1135 | 0.1118 | −0.5937 | −0.5954 | 0.853 | 0.999 | 5.40 | 0.03 |
| landing | ideal | ToF+IMU+高度辅助+光流 | 0.0179 | 0.0315 | 0.0406 | +0.0137 | +0.0228 | 1.000 | 1.000 | 0.00 | 0.00 |
| landing | noise | ToF+IMU+高度辅助 | 1.0229 | 0.0783 | 0.1119 | −0.9446 | −0.9110 | 0.750 | 0.999 | 6.19 | 0.02 |
| landing | noise | ToF+IMU+高度辅助+光流 | 0.0274 | 0.0387 | 0.0331 | +0.0113 | +0.0057 | 1.000 | 1.000 | 0.00 | 0.00 |
| landing | occluders | ToF+IMU+高度辅助 | 0.7553 | 0.5577 | 0.1141 | −0.1976 (−0.3836, +0.3714, −0.5808) | −0.6413 | 0.822 | 0.957 | 6.18 | 0.66 |
| landing | occluders | ToF+IMU+高度辅助+光流 | 0.0182 | 0.0331 | 0.0340 | +0.0149 | +0.0158 | 1.000 | 1.000 | 0.00 | 0.00 |
| landing | combined | ToF+IMU+高度辅助 | 0.9374 | 0.0917 | 0.1054 | −0.8458 | −0.8321 | 0.773 | 0.998 | 6.19 | 0.03 |
| landing | combined | ToF+IMU+高度辅助+光流 | 0.0241 | 0.0371 | 0.0288 | +0.0130 | +0.0047 | 1.000 | 1.000 | 0.00 | 0.00 |
| landing | adverse | ToF+IMU+高度辅助 | 0.7114 | 0.1204 | 0.1339 | −0.5910 | −0.5775 | 0.786 | 0.996 | 5.57 | 0.06 |
| landing | adverse | ToF+IMU+高度辅助+光流 | 0.0389 | 0.0401 | 0.0455 | +0.0012 | +0.0066 | 1.000 | 1.000 | 0.00 | 0.00 |
| landing | blackout | ToF+IMU+高度辅助 | 0.9374 | 0.0917 | 0.1054 | −0.8458 | −0.8321 | 0.773 | 0.998 | 6.19 | 0.03 |
| landing | blackout | ToF+IMU+高度辅助+光流 | 0.0241 | 0.0371 | 0.0288 | +0.0130 | +0.0047 | 1.000 | 1.000 | 0.00 | 0.00 |

两组的差别来自光流是否参与。ToF+IMU+高度辅助组在多个工况下无法从固定初值偏移收敛：`fuseTof` 的单帧 XY 修正受 `terrain_max_translation_update_m = 0.15` 信任域约束，0.1 m 的固定初值偏移叠加姿态/速度误差后时常越界，被 `terrain_update_trust_region` 拒绝；`bounded_recovery` 取默认 `false`，因此没有多帧一致的分步恢复，`xy_valid` 随之落到 0.75 附近并长时间失效（最长 8.93 s）。

ToF+IMU+高度辅助+光流组在全部 24 个 (窗口, 场景, 融合) 单元格上有效比例为 1.000，因为光流持续提供 XY 约束；代价是 `xy_valid` 不再反映地图解是否成立。

其余质量指标（3 种子均值）：

| 窗口 | 场景 | 融合 | 通信 | P95 m | 最大 m | 大误差仍有效比例 | 共同漂移 RMSE m | 机间相对 RMSE m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| transit | adverse | ToF+IMU+高度辅助 | off | 5.3825 | 7.5218 | 0.4380 | 1.1236 | 2.3176 |
| transit | adverse | ToF+IMU+高度辅助 | on | 11.7101 | 19.3200 | 0.4506 | 2.0937 | 5.4083 |
| transit | blackout | ToF+IMU+高度辅助 | off | 2.9927 | 4.6808 | 0.1777 | 0.4084 | 1.0169 |
| transit | blackout | ToF+IMU+高度辅助 | on | 0.2160 | 0.4663 | 0.1616 | 0.0482 | 0.0825 |
| transit | blackout | ToF+IMU+高度辅助+光流 | off | 0.0478 | 0.1003 | 0.0006 | 0.0192 | 0.0196 |
| transit | blackout | ToF+IMU+高度辅助+光流 | on | 0.1078 | 0.3059 | 0.0591 | 0.0290 | 0.0464 |
| landing | noise | ToF+IMU+高度辅助 | off | 2.7893 | 3.7388 | 0.0931 | 0.3790 | 1.4940 |
| landing | noise | ToF+IMU+高度辅助 | on | 0.1543 | 0.4313 | 0.0895 | 0.0366 | 0.0897 |

开启通信在收敛失败的工况上同时压低 P95、最大值、共同漂移和机间相对误差；在光流组则整体抬高这几项。`transit adverse` 的 on 是唯一在所有指标上同时恶化的单元格。

## 局限

1. 真值日志只提供位置与 yaw，roll/pitch 固定为 0，属于仅 yaw 的姿态重建。真实飞行中 roll/pitch 会改变 ToF 射线扇面的指向，遮挡比例与结构可观测性都建立在这一假设上。
2. 传感器与估计器先验共用同一份静态网格和同一套安装外参。结构定位精度因此不含建图误差、外参标定误差与网格简化误差，只反映滤波与遮挡的影响。
3. 高度辅助用带噪真值高度直接替换 z，不是真实高度传感器。两个融合模式都是 ToF+IMU+高度辅助，不存在"纯 ToF"组；z 方向误差不参与评分。
4. 机器人以 AABB 射线交点只进入传感器模型，不进入估计器先验；遮挡比例已如实给出（巡航 4.08%，降落 0.52%）。高空巡航时地面占位几乎不遮挡，这是本次实验的结论之一。
5. UWB 距离与方位由真值相对几何加噪合成，仅用于生成合成量测；队友位姿、协方差与参考来源字段全部来自各估计器快照，不含真值。
6. `fuseTof` 以 `bounded_recovery = false` 调用。生产 runner 只在 `LAND_ALL`/`RETURN_ALL`、近地、低速且开启 `bounded_landing_relocalization` 时传入 `true`。ToF+IMU+高度辅助组的门控不收敛是该配置下的实际结果，不代表启用分步恢复后的表现。
7. 重复单位是独立种子（3 个），不是帧。同一运行内的帧高度相关，本文的配对差值按种子配对给出，未把帧当作独立样本做区间估计。
8. 轨迹来自单次真实运行，窗口为 [7, 31] s 与 [90, 100] s；结论限于该轨迹与这四个出生位置。
9. `intermittent` 的 8–16 s 断链窗口是窗内相对秒；对 10 s 的降落窗只覆盖 8–10 s。
10. 未纳入 `scenarios.json` 中托盘平台的 `pressure_robots`。它们位于非比赛站位，纳入后会把降落段遮挡从 0.52% 抬到 84.9%，与正式比赛工况不符。

## 数据 manifest

| 文件 | 大小 | 内容 |
| --- | --- | --- |
| `build/research-20261023/localization-transit.json` | 693548 B | 全部结构化结果 |
| `build/research-20261023/localization-transit.trace.jsonl` | 31389460 B | 每个运行每 0.1 s 的估计、真值、误差与标志 |
| `docs/research-20261023/localization-manifest.json` | 7224 B | 机器可读清单（含结果文件 SHA256） |
| `simulation/research/scenarios.json` | 6470 B | 共享场景、剖面与机器人定义（A 提供，只读） |
| `simulation/research/tof_error_model.hpp` | 13636 B | 共享 ToF 误差模型（A 提供，只读） |

`localization-transit.json` 顶层字段：

| 字段 | 说明 |
| --- | --- |
| `schema` | 固定为 1 |
| `terrain` / `trajectory` / `windows` | 地图、轨迹与两个窗口的起止、评分起点、是否被记录末尾裁剪 |
| `rates_hz` / `sensor_noise` / `tof_geometry` / `tof_error_model` | 采样率、噪声参数、ToF 外参与共享模型来源 |
| `profiles` / `scenarios` / `scenarios_source` | 三档剖面、六个场景与配置来源，含 `used_builtin_defaults` |
| `communication_modes` / `fusion_modes` | 三个通信条件与两个融合模式的定义 |
| `initialization` | 固定初值偏移、协方差与通信延迟 |
| `limitations` | 本文"局限"一节的机器可读副本 |
| `sensor_model` | 每个 (窗口, 场景, 种子) 的射线统计与误差分布 |
| `runs` | 每个 (窗口, 场景, 种子, 通信, 融合) 的精度与计数指标 |
| `paired_by_seed` | 按种子配对的通信对比 |

`runs[]` 的关键字段：`xy_rmse_m`、`xy_p50_m`、`xy_p95_m`、`max_xy_error_m`、`xy_valid_fraction`、`xy_structural_fraction`、`xy_peer_fraction`、`false_valid_fraction`、`common_drift_rmse_m`、`pair_relative_rmse_m`、`tof_calls`、`tof_accepted`、`uwb_calls`、`uwb_accepted`、`uwb_rejections`（按原因计数）、`collaboration_evidence`、`input_checksum`、`vehicles[]`（逐机的 `xy_rmse_m`、`xy_p50_m`、`xy_p95_m`、`max_xy_error_m`、`xy_valid_fraction`、`xy_structural_fraction`、`longest_invalid_s`、`init_offset_m`、`init_yaw_offset_rad`）。`false_valid_fraction` 为误差大于 0.1 m 且 `xy_valid` 的样本占全部样本的比例，分母是全部样本而非有效样本。

`sensor_model[]` 的关键字段：`rays`、`robot_occluded_rays`、`robot_occluded_fraction`、`no_return`、`dropped`、`out_of_range`、`vehicle0_blackout`、`valid_returns`、`terrain_error_mean_m`、`terrain_error_std_m`、`terrain_error_p50_m`、`terrain_error_p95_m`、`terrain_error_absmax_m`。

trace 每行一个 (运行, 车辆) 样本：`w` 窗口、`s` 场景、`k` 种子、`c` 通信、`f` 融合、`t` 秒、`v` 车辆、`ex/ey/ez` 估计、`tx/ty` 真值、`e` XY 误差、`xyv/stv/pvv` 有效性标志、`m` 定位模式。仅评分窗内每 0.1 s 输出。

复现命令（在仓库根目录，WSL 内 g++ 11.4 / cmake / ninja）：

```sh
cmake -S simulation/research -B simulation/research/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build simulation/research/build -j
./simulation/research/build/localization_study \
  --terrain simulation/models/terrain/rmuc_2027/terrain_triangles.f32 \
  --trajectory simulation/runtime/run_20261016_215504/truth_evaluation_only.jsonl \
  --scenarios simulation/research/scenarios.json \
  --output build/research-20261023/localization-transit.json \
  --trace build/research-20261023/localization-transit.trace.jsonl \
  --start 7 --seconds 24 --eval-start 3 \
  --landing-start 90 --landing-seconds 10 \
  --seeds 2027,4099,6151 --jobs 8
```

契约测试：`python -m pytest simulation/research/test_localization_study.py`，覆盖组合完整性、指纹跨模式一致、`off` 条件零 UWB 调用、协同结论需有接受更新、传感器统计字段与 trace 时间戳单调性。
