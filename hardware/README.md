# Nano 到 companion 数据桥

tof_bridge.py 接收 Nano 的 JSONL type=tof 记录，校验 64 个 distance_mm 和 status，把毫米转换为米，并向 companion 的 observation UDP 端口发送 kind=tof。实际报文包含 64 个距离和有效标志。status 默认仅允许 5、9，通过 --allowed-status 或配置文件调整。传感器编号限制为 0..2。

Nano 的 source_time_us 原样保留为来源字段，时间基准保留为 source_time_basis，不会被当作飞控时间。

--time-mode monotonic 和 --time-mode receive 使用 bridge 主机的 time.monotonic_ns() 接收时刻，适合 bridge 和 companion runner 在同一台电脑上运行。time_error_limit_us 标为配置的接收时间误差上限，属于部署假设，不是硬件测量结果。

--time-mode timesync 通过 MAVLink TIMESYNC 把 bridge 单调时钟映射到飞控启动时钟。仅在收到有效同步样本后发送 ToF；time_basis=fc_boot_timesync，clock_error_us 为往返时延一半的保守估计，time_error_limit_us 还加上 receive_time_error_us。该估计假设链路延迟近似对称，不能证明 ToF 实际曝光时刻的误差；Nano 没有与飞控共享的时间戳，因此输出时刻仍是 bridge 收到 JSONL 的时刻。

bridge 提供单进程串口 MAVLink 转发：companion runner UDP remote 14580 与 tof_bridge 的串口互通，companion runner UDP local 14540 接收串口上行帧。bridge 会把串口收到的完整 MAVLink 帧转发到 runner 的本地 UDP 端口，把 runner 发往 remote 端口的数据写入串口，并每秒发起一次 TIMESYNC。串口锁文件防止多个 bridge 进程同时占用同一条链路。bridge 不发送解锁、模式切换或控制命令。

## 电脑回环测试

在仓库根目录使用 PowerShell 7：

    $ErrorActionPreference = 'Stop'
    python -m unittest discover -s hardware -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'bridge tests failed' }

## 部署示例

1. 在 companion 主机安装项目依赖，确认 pyserial==3.5 可导入。
2. 复制并修改 hardware/tof_bridge_config.example.json，为每个 vehicle 使用不同的 ToF 端口、MAVLink UDP 端口和串口。
3. 启动 bridge，并把 Nano 采集程序的 stdout 接入 bridge stdin：

    ./drone-nano --sensor /dev/i2c-1:0x29 --mavlink /dev/ttyS1 --baud 115200 | python3 hardware/tof_bridge.py --config hardware/tof_bridge_config.example.json

Nano 的 MAVLink 接收串口和 bridge 的飞控 MAVLink 串口必须是两条实际链路；若 Nano 只采集 ToF，则不要把同一串口交给两个进程。

4. 运行 companion runner 时不改源码，指定 bridge 的 UDP 端口：

    ./drone_companion_runner --vehicle 0 --config simulation/config.json --map simulation/maps/calibration.json --runtime simulation/runtime --mavlink-local-port 14540 --mavlink-remote-port 14580

5. 首次部署和回环测试不要使用 runner 的 --arm 参数，不自动解锁真机。

当前验证仅覆盖电脑上的 JSONL、UDP、MAVLink 帧解析和 TIMESYNC 数学闭环；没有实物串口、飞控、Nano、相机或 UWB 验证，因此不能据此声称已上板。相机和 UWB 标定仍需实测。
