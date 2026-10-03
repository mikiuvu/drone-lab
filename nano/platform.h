/**
 * @file platform.h
 * @brief ST VL53L5CX ULD 的 Linux I²C 平台接口与每设备状态。
 * @note 此接口访问真实设备；研究模型的合成距离不经过此层。
 */
#ifndef DRONE_PLATFORM_H
#define DRONE_PLATFORM_H
#include <stdint.h>
#include <string.h>
#include <signal.h>
/** @brief 每颗传感器独立的总线句柄、停止条件和累计传输量。 */
typedef struct {
    uint16_t address; // ULD 的 8 位地址形式；I2C_RDWR 时右移一位得到 7 位地址。
    int fd; // 已打开的 /dev/i2c-N 文件描述符；生命周期由采集器管理。
    const volatile sig_atomic_t *running; // 非空时，指向信号处理函数更新的运行标志。
    uint64_t deadline_us; // CLOCK_MONOTONIC 绝对微秒期限；0 禁用，仅在系统调用之间检查。
    int io_errno; // 锁存首个错误，阻止 ULD 在失败后继续发起传输；调用方负责清零。
    uint64_t read_payload_bytes, write_payload_bytes; // 成功分块的载荷字节数，不含地址、寄存器和 ACK。
} VL53L5CX_Platform;
// 每区保留一个目标；关闭未用于 JSONL 的结果字段以控制帧载荷。
#define VL53L5CX_NB_TARGET_PER_ZONE 1U
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_AMBIENT_DMAX
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define PROCESSOR_LITTLE_ENDIAN
#define SWAP_UINT16(x) (x)
#define SWAP_UINT32(x) (x)
/**
 * @brief 读取一个寄存器字节，使用 RdMulti 的错误与取消规则。
 * @param p 设备平台状态，须包含有效总线句柄。
 * @param reg 16 位寄存器地址。
 * @param v 可写的单字节输出。
 * @return 0 成功，1 失败。
 */
uint8_t RdByte(VL53L5CX_Platform *, uint16_t, uint8_t *);
/** @brief 写入一个寄存器字节；参数依次为设备状态、寄存器地址和值，返回 0 成功、1 失败。 */
uint8_t WrByte(VL53L5CX_Platform *, uint16_t, uint8_t);
/**
 * @brief 分块读取连续寄存器；每块以组合事务发送寄存器地址再读取载荷。
 * @param p 设备平台状态；实际传输时不得为空。
 * @param reg 起始寄存器地址，分块后递增。
 * @param data 至少容纳 count 字节的可写缓冲区。
 * @param count 载荷字节数；每次最多读取 256 字节。
 * @return 0 成功，1 失败；失败时整个输出缓冲区清零。
 * @note 截止时间不能中断正在执行的 ioctl；成功计数不包含协议开销。
 */
uint8_t RdMulti(VL53L5CX_Platform *, uint16_t, uint8_t *, uint32_t);
/**
 * @brief 分块写入连续寄存器；每块将大端寄存器地址与最多 256 字节载荷一起发送。
 * @param p 设备平台状态；实际传输时不得为空。
 * @param reg 起始寄存器地址。
 * @param data 至少包含 count 字节的输入缓冲区。
 * @param count 待写载荷字节数。
 * @return 0 成功，1 失败；先前成功写入的分块不会回滚。
 */
uint8_t WrMulti(VL53L5CX_Platform *, uint16_t, uint8_t *, uint32_t);
/**
 * @brief 以毫秒等待，并在等待前后和信号打断后检查取消与期限。
 * @param p 设备状态；可为空，此时信号可提前结束等待。
 * @param ms 等待毫秒数。
 * @return 0 等待完成或空状态的等待被信号打断，1 平台或睡眠失败。
 */
uint8_t WaitMs(VL53L5CX_Platform *, uint32_t);
/** @brief 原地逆序每组 4 字节；参数为可写缓冲区及字节数，末尾不足 4 字节保持原样。 */
void SwapBuffer(uint8_t *, uint16_t);
/** @brief 读取本机单调时钟微秒数；失败打印错误并返回 0，不是 UTC 或飞控启动时间。 */
uint64_t monotonic_us(void);
#endif
