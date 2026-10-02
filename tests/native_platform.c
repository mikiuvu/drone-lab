/* Inject I2C syscall outcomes into the real Linux platform implementation. */
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdarg.h>
#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <time.h>
static int fake_ioctl(int,unsigned long,...);
static int fake_sleep(const struct timespec *,struct timespec *);
#define ioctl fake_ioctl
#define nanosleep fake_sleep
#include "../nano/platform.c"
#undef ioctl
#undef nanosleep
static int calls,fail_at,short_result,sleep_error;
static unsigned expected_reg;
static int fake_ioctl(int fd,unsigned long op,...) {
    (void)fd;assert(op==I2C_RDWR);calls++;
    va_list args;va_start(args,op);struct i2c_rdwr_ioctl_data *request=va_arg(args,struct i2c_rdwr_ioctl_data *);va_end(args);
    assert(request->msgs[0].addr==0x29);
    unsigned reg=((unsigned)request->msgs[0].buf[0]<<8)|request->msgs[0].buf[1];
    assert(reg==expected_reg);
    unsigned size=request->nmsgs==2?request->msgs[1].len:request->msgs[0].len-2;
    assert(size<=256);expected_reg+=size;
    if(calls==fail_at){errno=ENXIO;return short_result?1:-1;}
    if(request->nmsgs==2)memset(request->msgs[1].buf,0x5a,size);
    return (int)request->nmsgs;
}
static int fake_sleep(const struct timespec *wait,struct timespec *rest){(void)wait;(void)rest;errno=sleep_error;return sleep_error?-1:0;}
int main(void) {
    VL53L5CX_Platform p={.fd=10,.address=0x52};uint8_t bytes[600];
    expected_reg=0x100;assert(!RdMulti(&p,0x100,bytes,sizeof(bytes)));assert(calls==3&&p.read_payload_bytes==600);
    for(unsigned i=0;i<sizeof(bytes);i++)assert(bytes[i]==0x5a);
    calls=0;expected_reg=0x200;fail_at=2;memset(bytes,0xff,sizeof(bytes));
    assert(RdMulti(&p,0x200,bytes,sizeof(bytes))==1&&p.io_errno==ENXIO&&calls==2);
    for(unsigned i=0;i<sizeof(bytes);i++)assert(bytes[i]==0);
    assert(p.read_payload_bytes==856); /* Successful first chunk is still counted. */
    assert(WrMulti(&p,0,bytes,1)==1&&calls==2); /* Error latch prevents more transfers. */
    p.io_errno=0;calls=0;expected_reg=0;fail_at=1;short_result=1;
    assert(RdMulti(&p,0,bytes,4)==1&&p.io_errno==EIO);
    p.io_errno=0;calls=0;fail_at=0;expected_reg=0x200;
    assert(!WrMulti(&p,0x200,bytes,sizeof(bytes))&&calls==3&&p.write_payload_bytes==600);
    volatile sig_atomic_t active=0;p.running=&active;p.io_errno=0;calls=0;
    assert(RdMulti(&p,0,bytes,4)==1&&p.io_errno==ECANCELED&&calls==0);
    active=1;p.io_errno=0;p.deadline_us=1;
    assert(WrMulti(&p,0,bytes,1)==1&&p.io_errno==ETIMEDOUT&&calls==0);
    p.deadline_us=0;p.io_errno=0;sleep_error=EINVAL;
    assert(WaitMs(&p,2)==1&&p.io_errno==EINVAL);
    p.io_errno=0;sleep_error=EINTR;active=0;
    assert(WaitMs(&p,2)==1&&p.io_errno==ECANCELED);
    puts("PASS platform chunks / partial reads / error latch / cancellation / deadline / sleep failure");
    return 0;
}
