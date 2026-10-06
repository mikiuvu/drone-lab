# 真实机体外观接入

把 SolidWorks 里的 RMUC 整机装配换成 Gazebo 里的机体视觉，只替换外观，不动物理。

## 为什么需要你导出一次

`rmuc无人机.SLDASM` 是 SolidWorks 原生装配格式，Gazebo 与 FreeCAD 都读不了，仓库里
也没有任何能解析它的工具。唯一能走的路径是从 SolidWorks 导出成通用网格格式。

压缩包里自带的 `运算平台拓展板.step` 只是运算平台那块板，不是整机；`scripts/step_measure.py`
量出来是 63.0 × 54.5 × 11.7 mm，可以作为尺寸参考，但撑不起整机外观。

## 需要导出什么

在 SolidWorks 里打开 `rmuc无人机.SLDASM`，用**另存为**导出以下任一格式：

| 格式 | 说明 |
| --- | --- |
| **STL**（推荐） | 二进制的即可，生成器能自动量包围盒并判断单位 |
| OBJ | 同样支持自动量测 |

导出前请注意：

- **单位**：SolidWorks 的 STL 导出默认按**毫米**。生成器会按包围盒自动判定（任一轴超过 1
  就当作毫米，缩放 0.001）；如果整机尺寸恰好落在模糊区间，用 `--visual-mesh-scale` 显式指定。
- **坐标系**：SolidWorks 默认 Z 轴向上，机体系是 FLU（X 前、Y 左、Z 上）。导出的网格会先被
  平移到包围盒中心对齐机体原点；如果机头方向不对，用 `--visual-mesh-pose` 的 roll/pitch/yaw 转正。
- **零件数量**：整机装配导出的 STL 会包含所有可见零件，这正是想要的。但**不要**导出内部
  被完全包住的零件（电池、飞控板），否则面数会白涨几倍。先在 SolidWorks 里隐藏不需要的零件。
- **面数**：建议控制在 20 万面以内。机载相机和深度相机不会被这些几何影响（视觉只作用于
  渲染），但面数过高会拖慢 GUI。

## 接入命令

```bash
python3 simulation/models/generate_sensor_models.py \
  --dynamics micro \
  --output simulation/models/generated_micro \
  --vehicle-count 4 \
  --visual-mesh /path/to/rmuc无人机.stl
```

生成器会：

1. 量出网格包围盒，自动判定单位并打印缩放后的尺寸；
2. 把网格装到 `simulation/models/generated_micro/drone_visual.stl`（只装一份，四机用相对 URI 共用）；
3. 把 `base_link` 的 `body_visual` 从占位方块换成该网格，位姿自动居中；
4. 在 `sensor_contract.json` 里记下 `visual_mesh` 的来源、缩放、面数。

**碰撞体、质量、惯量、旋翼几何全部不变**——那些决定 PX4 的分配矩阵和飞行特性，换掉会连带
改变控制表现。保护圈圆柱仍是碰撞体，所以接触判定和撞机测试的结果不受外观影响。

可选参数：

| 参数 | 用途 |
| --- | --- |
| `--visual-mesh-scale` | 显式指定缩放，覆盖自动判定 |
| `--visual-mesh-pose X Y Z ROLL PITCH YAW` | 在自动居中基础上追加机体系平移与旋转（m、rad） |
| `--visual-mesh-transparency` | 网格透明度 0..1，用来同时看到内部结构 |

## 先量一下再接入

拿到导出文件后，先不接进模型，单独量一下尺寸是否合理：

```bash
python scripts/step_measure.py 模型.step     # STEP 直接读文本里的 CARTESIAN_POINT
python -c "import sys; sys.path.insert(0,'simulation/models'); from mesh_bounds import read_bounds; print(read_bounds('模型.stl'))"
```

整机尺寸应该在 0.10–0.25 m 量级。当前占位模型的保护圈半径是 0.052 m（外径 0.104 m），
真实整机如果明显更大或更小，说明单位判断错了，或者导出时选了缩放。

## 验证

```bash
pytest tests/test_visual_mesh.py -q          # 解析、缩放、居中、SDF 接线
python3 simulation/tests/sensor_smoke.py     # 真实 Gazebo 加载一次模型
```

`tests/test_visual_mesh.py` 覆盖二进制/ASCII STL 与 OBJ、毫米与米两种单位、显式覆盖参数、
以及不指定网格时保持占位方块。`sensor_smoke.py` 会用 Gazebo 真正加载模型，网格 URI 解析
失败时会直接报错（已验证：把 URI 改坏会得到
`uri [../does_not_exist.stl] could not be resolved`）。

接入后启动带 GUI 的仿真即可看到真实外观：

```bash
python3 simulation/launch_linux.py --gui --mission team_contact
```

## 恢复占位方块

去掉 `--visual-mesh` 重新生成即可，生成的模型目录不进版本库（`.gitignore` 里的
`simulation/models/generated*/`）。
