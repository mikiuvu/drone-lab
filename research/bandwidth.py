"""@file bandwidth.py
@brief 估算共享 I²C 读取及全双工 UART 的理想带宽预算；输入是配置假设，输出为占用率和耗时下限。
"""

import math

DEFAULT_CONFIG = {
    "i2c_khz": 400.0,
    "sensor_count": 2.0,
    "sensor_hz": 15.0,
    "payload_bytes": 1500.0,
    "chunk_bytes": 256.0,
    "uart_baud": 115200.0,
    "attitude_hz": 100.0,
    "odometry_hz": 30.0,
}


def i2c_read_bits(payload, chunk):
    """@brief 估算分块组合读的总位数。
    @param payload 单帧载荷字节数，正整数。
    @param chunk 每块最大载荷字节数，正整数。
    @return 载荷与每块四个地址/寄存器字节所需的位数，每字节按 8 位加 ACK/NACK。
    @note 不计 START/STOP 时长、时钟拉伸和 Linux 调度。
    """
    if (
        not isinstance(payload, int)
        or not isinstance(chunk, int)
        or payload <= 0
        or chunk <= 0
    ):
        raise ValueError("Positive integer byte counts required")
    # Per combined read: write address + two register bytes + read address.
    # Data and all address/register bytes each require 8 bits + ACK/NACK.
    transactions = math.ceil(payload / chunk)
    return 9 * (payload + 4 * transactions)


def run_bandwidth(config=None):
    """@brief 按配置计算共享 I²C 和两个 UART 方向的独立预算。
    @param config 默认参数的覆盖字典；i2c_khz 为 kHz，频率为 Hz，载荷/分块为字节，uart_baud 为 bit/s。
    @return 含 config、metrics、summary、assumptions 的字典；占用率是无量纲比例，可超过 1。
    @note 默认帧载荷是示例假设，硬件实测应使用采集器的累计载荷计数；此函数不访问设备。
    """
    cfg = DEFAULT_CONFIG | (config or {})
    if set(cfg) != set(DEFAULT_CONFIG):
        raise ValueError("Unknown bandwidth parameter")
    if any(
        isinstance(v, bool)
        or not isinstance(v, (int, float))
        or not math.isfinite(v)
        or v <= 0
        for v in cfg.values()
    ):
        raise ValueError("All bandwidth parameters must be positive and finite")
    for key in ("sensor_count", "payload_bytes", "chunk_bytes"):
        if cfg[key] != int(cfg[key]):
            raise ValueError(key + " must be integer")
        cfg[key] = int(cfg[key])
    if (
        cfg["sensor_count"] > 4
        or cfg["payload_bytes"] > 100000
        or cfg["chunk_bytes"] > 4096
    ):
        raise ValueError("Unsupported budget size")
    bits = i2c_read_bits(cfg["payload_bytes"], cfg["chunk_bytes"])
    read_ms = bits / cfg["i2c_khz"]
    bus_usage = bits * cfg["sensor_hz"] * cfg["sensor_count"] / (cfg["i2c_khz"] * 1000)
    # MAVLink2 unsigned framing=12B; ATTITUDE payload28, ODOMETRY max payload233.
    # UART full duplex capacities are separate. Do not sum RX and TX utilization.
    rx_bytes = (28 + 12) * cfg["attitude_hz"] + (9 + 12) * 1
    tx_bytes = (233 + 12) * cfg["odometry_hz"]
    capacity = cfg["uart_baud"] / 10
    return {
        "summary": f"共享 I²C 总线仅距离帧读取就占 {bus_usage * 100:.1f}%（理想下限）；单帧读取至少 {read_ms:.1f} ms。",
        "metrics": {
            "i2c_read_bits_per_frame": bits,
            "i2c_read_minimum_ms": read_ms,
            "shared_bus_lower_bound_utilization": bus_usage,
            "uart_8n1_bytes_per_second": capacity,
            "fc_to_nano_rx_utilization": rx_bytes / capacity,
            "nano_to_fc_tx_utilization": tx_bytes / capacity,
            "fc_to_nano_example_bytes_s": rx_bytes,
            "nano_to_fc_example_bytes_s": tx_bytes,
        },
        "config": cfg,
        "assumptions": [
            "payload_bytes=1500 是示例帧载荷；必须用驱动实际 data_read_size 和读写字节计数替换。",
            "I²C每次分块按16位寄存器、重复起始、7位地址建模；9位/字节。忽略起止时间、时钟拉伸、总线竞争和Linux ioctl调度，结果是乐观耗时下限。",
            "sensor_count个传感器共享一条总线，独立总线需分别计算；不因为总线并列就默认线程同时读取。",
            "UART按8N1、MAVLink2未签名最大载荷估算；RX示例为ATTITUDE和1Hz心跳，TX示例为ODOMETRY。",
            "UART全双工两个方向分别计算，不能相加成一个方向负载；未计其他消息、签名、丢包重传和驱动缓冲。",
            "ODOMETRY 的消息频率用于计算 Nano 到飞控的发送带宽预算。",
        ],
    }
