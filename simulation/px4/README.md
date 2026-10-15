# PX4 logger 订阅诊断

补丁对应 PX4 `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`，在估计器回放配置中记录 `logger_status`，并在全速话题订阅跳号时输出话题、前后代号和当前样本时刻。

`dropouts` 统计写缓冲区溢出；`message_gaps` 统计全速话题的订阅跳号。后者是事件数，每次跳号可能丢失多条消息。日志没有 `logger_status` 时，审计输出的 `full_log_message_gaps_max` 为 `null`。

## 独立构建

使用已有 PX4 SITL Ninja 构建的编译数据库、静态库和链接参数。构建器在输出目录复制并修改两个 logger 源文件，重新编译 logger 对象并链接单独的程序。输出目录须为空且位于 SDK 与原构建目录之外。

在仓库根目录以 PowerShell 7 执行：

```powershell
$ErrorActionPreference = 'Stop'
wsl -d Ubuntu-22.04 --exec python3 '/mnt/d/Robomaster code/drone/scripts/build_px4_logger_diagnostic.py' --sdk /home/devcontainers/drone-sim/PX4-Autopilot --output /home/devcontainers/drone-sim/build-logger-diagnostic
if ($LASTEXITCODE -ne 0) { throw 'PX4 logger diagnostic build failed' }
```

输出包括 `bin/px4`、模块命令符号链接、修改后的源码、logger 静态库和 `manifest.json`。清单记录基线与诊断二进制哈希、源码哈希及编译链接参数。其余组件复用现有对象。

诊断运行需要选择输出的 `bin/px4`；现有启动器的默认固件路径不变。`SDLOG_PROFILE=3` 包含估计器回放话题。逐话题跳号输出会增加运行时日志负载，耗时比较应保留相同诊断配置。

## 分析

输入完整性仍按 EKF2 索引逐条检查。例如：

```powershell
$ErrorActionPreference = 'Stop'
.venv/Scripts/python.exe -X utf8 scripts/audit_ulog_replay.py 'flight.ulg' --begin-s 9.346 --output 'coverage.json'
if ($LASTEXITCODE -notin @(0,1)) { throw 'ULog audit failed' }
```

退出码 0 表示索引输入完整，1 表示存在缺失；其他错误需检查日志内容或参数。`nearest_logged_before_us` 与 `nearest_logged_after_us` 给出缺失时刻两侧的实际记录，没有对应记录时为 `null`。

将缺失输入之后的首条实际记录与 `subscription gap topic=... sample_us=...` 对照，可定位发生跳号的订阅。完整性检查覆盖 EKF2 索引输入；它不等同于所有话题的每次发布都已保存。

实际复现与运行结果见 [里程计输入覆盖与 logger 订阅跳号](../../docs/nano-logger-coverage-20261016.md)。
