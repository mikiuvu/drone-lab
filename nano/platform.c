/**
 * @file platform.c
 * @brief 将 ULD 寄存器操作适配为 Linux I2C_RDWR，并提供单调时钟和可取消等待。
 */
#define _POSIX_C_SOURCE 200809L
#include "platform.h"
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <time.h>
uint64_t monotonic_us(void) {
    struct timespec now;
    if(clock_gettime(CLOCK_MONOTONIC, &now)) {perror("CLOCK_MONOTONIC");return 0;}
    return (uint64_t)now.tv_sec*1000000ULL + (uint64_t)now.tv_nsec/1000ULL;
}
/**
 * @brief 检查取消标志、绝对期限和已锁存错误。
 * @param p 设备状态，可为空，空状态只跳过设备检查。
 * @return 1 允许继续，0 停止；停止时恢复锁存的 errno。
 * @note 只检查系统调用之间的状态，不对内核阻塞作硬实时保证。
 */
static int platform_ready(VL53L5CX_Platform *p) {
    if(!p)return 1;
    if(!p->io_errno && p->running && !*p->running)p->io_errno=ECANCELED;
    if(!p->io_errno && p->deadline_us) {
        uint64_t now=monotonic_us();
        if(!now)p->io_errno=EIO;
        else if(now>=p->deadline_us)p->io_errno=ETIMEDOUT;
    }
    if(p->io_errno){errno=p->io_errno;return 0;}
    return 1;
}
uint8_t RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *data, uint32_t count) {
    uint8_t *begin=data;uint32_t total=count;
    /* 1. ULD 在读取失败后仍可能解码，先清零并在失败时清除已读分块，避免旧帧头被当作新帧。 */
    memset(data,0,count);
    while (count) {
        if(!platform_ready(p)){memset(begin,0,total);return 1;}
        /* 2. 连续寄存器按 256 字节分块；寄存器地址以高字节在前发送。 */
        uint16_t size=count>256 ? 256 : (uint16_t)count;
        uint8_t address[2]={(uint8_t)(reg>>8),(uint8_t)reg};
        struct i2c_msg messages[2]={
            {.addr=p->address>>1,.flags=0,.len=2,.buf=address},
            {.addr=p->address>>1,.flags=I2C_M_RD,.len=size,.buf=data}};
        /* 3. 两条消息作为一次组合事务，重复起始后读取，避免拆成独立调用。 */
        struct i2c_rdwr_ioctl_data request={.msgs=messages,.nmsgs=2};
        int result=ioctl(p->fd,I2C_RDWR,&request);
        if(result!=2) {
            p->io_errno=result<0?errno:EIO;memset(begin,0,total);
            errno=p->io_errno;return 1;
        }
        // 只有内核确认两条消息均完成才统计载荷并推进寄存器。
        p->read_payload_bytes+=size;
        reg+=size;data+=size;count-=size;
    } return 0;
}
uint8_t WrMulti(VL53L5CX_Platform *p,uint16_t reg,uint8_t *data,uint32_t count) {
    while(count) {
        if(!platform_ready(p))return 1;
        uint16_t size=count>256 ? 256 : (uint16_t)count;
        // 地址占两字节，载荷最多 256 字节；一次写消息保持这一块的地址与数据连续。
        uint8_t buffer[258]={(uint8_t)(reg>>8),(uint8_t)reg};memcpy(buffer+2,data,size);
        struct i2c_msg message={.addr=p->address>>1,.flags=0,.len=(uint16_t)(size+2),.buf=buffer};
        struct i2c_rdwr_ioctl_data request={.msgs=&message,.nmsgs=1};
        int result=ioctl(p->fd,I2C_RDWR,&request);
        if(result!=1) {p->io_errno=result<0?errno:EIO;errno=p->io_errno;return 1;}
        p->write_payload_bytes+=size;
        reg+=size;data+=size;count-=size;
    } return 0;
}
uint8_t RdByte(VL53L5CX_Platform *p,uint16_t reg,uint8_t *v) {return RdMulti(p,reg,v,1);}
uint8_t WrByte(VL53L5CX_Platform *p,uint16_t reg,uint8_t v) {return WrMulti(p,reg,&v,1);}
uint8_t WaitMs(VL53L5CX_Platform *p,uint32_t ms) {
    if(!platform_ready(p))return 1;
    struct timespec wait={.tv_sec=ms/1000,.tv_nsec=(long)(ms%1000)*1000000L};
    while(nanosleep(&wait,&wait)) {
        if(errno!=EINTR){if(p)p->io_errno=errno;return 1;}
        if(!platform_ready(p))return 1;
        if(!p)return 0; /* 采集主循环的空状态等待被信号打断后立即返回，便于退出。 */
    }
    return platform_ready(p)?0:1;
}
void SwapBuffer(uint8_t *data,uint16_t size) {
    for(uint16_t i=0;i+3<size;i+=4) {uint8_t a=data[i],b=data[i+1];data[i]=data[i+3];data[i+1]=data[i+2];data[i+2]=b;data[i+3]=a;}
}
