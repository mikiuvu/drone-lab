#ifndef DRONE_PLATFORM_H
#define DRONE_PLATFORM_H
#include <stdint.h>
#include <string.h>
#include <signal.h>
/* ST ULD uses an 8-bit address; Linux I2C_RDWR uses its 7-bit form. */
typedef struct {
    uint16_t address;
    int fd;
    const volatile sig_atomic_t *running;
    uint64_t deadline_us; /* Checked between syscalls; cannot interrupt an ioctl. */
    int io_errno; /* Latched: ULD may continue calling the platform after failure. */
    uint64_t read_payload_bytes, write_payload_bytes;
} VL53L5CX_Platform;
#define VL53L5CX_NB_TARGET_PER_ZONE 1U
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_AMBIENT_DMAX
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define PROCESSOR_LITTLE_ENDIAN
#define SWAP_UINT16(x) (x)
#define SWAP_UINT32(x) (x)
uint8_t RdByte(VL53L5CX_Platform *, uint16_t, uint8_t *);
uint8_t WrByte(VL53L5CX_Platform *, uint16_t, uint8_t);
uint8_t RdMulti(VL53L5CX_Platform *, uint16_t, uint8_t *, uint32_t);
uint8_t WrMulti(VL53L5CX_Platform *, uint16_t, uint8_t *, uint32_t);
uint8_t WaitMs(VL53L5CX_Platform *, uint32_t);
void SwapBuffer(uint8_t *, uint16_t);
uint64_t monotonic_us(void);
#endif
