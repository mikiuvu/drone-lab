# ToF 误差与帧率来源核对

## 器件与文档

| 项 | 内容 |
| --- | --- |
| 器件 | ST VL53L5CX，8×8 / 4×4 多区 ToF 测距 |
| 数据手册 | DS13754 Rev 5，2021-12-15，38 页，标题 "Time-of-Flight 8x8 multizone ranging sensor with wide field of view" |
| 用户手册 | UM2884 Rev 2，2021-09-03，19 页，标题 "A guide to using the VL53L5CX multizone Time-of-Flight ranging sensor with wide field of view Ultra Lite Driver (ULD)" |
| 官方地址 | https://www.st.com/resource/en/datasheet/vl53l5cx.pdf ； https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf |

本次核对使用的副本（ST 官方 CDN 对本机网络返回 HTTP 567 拦截页，改用同文档镜像，并通过 PDF 元数据确认文档号与版本）：

- DS13754 Rev 5：https://www.pololu.com/file/0J1878/vl53l5cx.pdf
- UM2884 Rev 2：https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library/blob/main/documents/um2884-a-guide-to-using-the-vl53l5cx.pdf

## 手册原文

**DS13754 Rev 5**

- 第 1 页 Features：`Up to 400 cm ranging`；`60 Hz frame rate capability`。
- 第 3 页 Table 1 Technical specifications：`Ranging 2 to 400 cm per zone`；`Sample rate Up to 60 Hz`。
- 第 3 页 Table 2 FoV angles：探测体积（detection volume）水平 45°、垂直 45°、对角 63°。表下注释说明该探测体积在 8×8、14 % sharpener、Continuous 模式 15 Hz、1 m 白靶 88 % 反射率、无环境光条件下测得。
- 第 20 页 §5.2.3 Table 18：8×8 Continuous 15 Hz 的最大测距能力，暗环境白靶内侧典型 3500 mm。
- 第 21 页 §5.2.4 Table 19 Range accuracy（Continuous）：
  - 20–200 mm，8×8 15 Hz，灰靶 17 %，全分区：暗环境 ±15 mm，5 klux 环境 15 mm。
  - 201–4000 mm，4×4 30 Hz：白靶 ±4 %（暗）/ 7 %（5 klux）；灰靶 ±5 % / 8 %。
  - 201–4000 mm，8×8 15 Hz：白靶 ±5 %（暗）/ 8 %（5 klux）；灰靶 ±5 % / 11 %。
- 第 19、22 页 §5.2.1 / §5.3.1 测量条件：测距精度按 2.7σ 给出，即 99.3 % 的测量落在标注精度内；最大测距能力按 90 % 检出率给出。
- 第 22 页 Table 21：Autonomous 模式 8×8 在 15 Hz、5 ms 积分时间下测距。

**UM2884 Rev 2**

- 第 1 页 Introduction：`Each zone of the VL53L5CX measures the distance of the target up to 4 meters, at a maximum frequency of 60 Hz.`
- 第 8 页 §4.4 Table 2 Minimum and maximum ranging frequencies：4×4 为 1–60 Hz，8×8 为 1–15 Hz；默认 1 Hz。
- 第 14 页 §5.5 Table 4 目标状态列表：状态 5 表示量程有效；其余状态（信号过弱、相位、σ 估计过大、一致性失败、无目标等）均为无效或低置信。手册要求使用方按状态过滤无效量测。

## 与仿真参数的对应

| 仿真项 | 取值 | 依据 |
| --- | --- | --- |
| 分辨率 | 8×8 | 本工程三向 ToF 均为 8×8。 |
| 帧率 | 15 Hz | UM2884 第 8 页 8×8 最大 15 Hz；DS13754 第 20、21 页 8×8 特性均在 15 Hz 下给出。 |
| 水平/垂直视场 | 45° / 45° | DS13754 第 3 页 Table 2。 |
| 量程 | 0.02–4.0 m | DS13754 第 3 页 Table 1 `2 to 400 cm per zone`。 |
| 量程外处理 | 标记无效，不限幅 | UM2884 第 14 页按目标状态过滤无效量测的用法。 |

## 研究假设（不是数据手册保证）

`simulation/research/scenarios.json` 的 `nominal` 与 `adverse` 是按上述手册指标构造的研究假设，用于描述一类测距传感器的性能量级，不代表 VL53L5CX 在任意距离、反射率、环境光下的实测分布。

- `nominal`：逐像素 1σ 在 2 m 处为 `0.010 + 0.005×2 = 0.020 m`，按手册 2.7σ 口径约 0.054 m，即 2 m 处 2.7 %；叠加设备固定偏差上界 0.015 m 与比例上界 0.5 %（2 m 处 0.010 m）后，最坏约 0.079 m，即 4 %，落在 8×8 15 Hz 暗环境白靶 ±5 % 之内。
- `adverse`：逐像素 1σ 在 2 m 处为 `0.025 + 0.015×2 = 0.055 m`，2.7σ 约 0.149 m，即 7.4 %，接近手册 5 klux 环境光下 8×8 的 8 % 量级，用作恶劣工况上界。
- `drop_probability`：手册最大测距能力按 90 % 检出率定义，因此 0.10 的逐像素失效概率对应最坏检出率口径；0.03 为常用值假设。
- 设备固定偏差与比例误差、逐帧公共误差在手册中未给出分布形状，本次按有界均匀（偏差、比例）与零均值高斯（逐帧公共项）建模，属于研究假设。
- 手册的精度指标是整机在特定靶标、无盖板、默认标定条件下的统计结果，仿真中的逐像素独立白噪声、边缘混合像素失效等属于工程化简化。
