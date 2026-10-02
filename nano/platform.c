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
    /* ULD decodes even after a failed read. Never leave stale frame headers. */
    memset(data,0,count);
    while (count) {
        if(!platform_ready(p)){memset(begin,0,total);return 1;}
        uint16_t size=count>256 ? 256 : (uint16_t)count;
        uint8_t address[2]={(uint8_t)(reg>>8),(uint8_t)reg};
        struct i2c_msg messages[2]={
            {.addr=p->address>>1,.flags=0,.len=2,.buf=address},
            {.addr=p->address>>1,.flags=I2C_M_RD,.len=size,.buf=data}};
        struct i2c_rdwr_ioctl_data request={.msgs=messages,.nmsgs=2};
        int result=ioctl(p->fd,I2C_RDWR,&request);
        if(result!=2) {
            p->io_errno=result<0?errno:EIO;memset(begin,0,total);
            errno=p->io_errno;return 1;
        }
        p->read_payload_bytes+=size;
        reg+=size;data+=size;count-=size;
    } return 0;
}
uint8_t WrMulti(VL53L5CX_Platform *p,uint16_t reg,uint8_t *data,uint32_t count) {
    while(count) {
        if(!platform_ready(p))return 1;
        uint16_t size=count>256 ? 256 : (uint16_t)count;
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
        if(!p)return 0; /* A signal interrupts the collector loop sleep. */
    }
    return platform_ready(p)?0:1;
}
void SwapBuffer(uint8_t *data,uint16_t size) {
    for(uint16_t i=0;i+3<size;i+=4) {uint8_t a=data[i],b=data[i+1];data[i]=data[i+3];data[i+1]=data[i+2];data[i+2]=b;data[i+3]=a;}
}
