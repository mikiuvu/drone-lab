/* Real collector loop with sensor calls mocked; no physical devices are opened. */
#define _DEFAULT_SOURCE
#include <assert.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <poll.h>
static int fake_open(const char *,int,...);
static ssize_t fake_read(int,void *,size_t);
static ssize_t fake_write(int,const void *,size_t);
static int fake_poll(struct pollfd *,nfds_t,int);
static uint64_t fake_monotonic_us(void);
#define open fake_open
#define read fake_read
#define write fake_write
#define poll fake_poll
#define monotonic_us fake_monotonic_us
#define vl53l5cx_is_alive fake_alive
#define vl53l5cx_init fake_init
#define vl53l5cx_set_resolution fake_resolution
#define vl53l5cx_set_ranging_frequency_hz fake_frequency
#define vl53l5cx_start_ranging fake_start
#define vl53l5cx_stop_ranging fake_stop
#define vl53l5cx_check_data_ready fake_ready
#define vl53l5cx_get_ranging_data fake_data
#define main collector_main
#include "../nano/main.c"
#undef main
#undef open
#undef read
#undef write
#undef poll
#undef monotonic_us
extern uint64_t monotonic_us(void);
static const char *mode;
static int init_count,stop_count,frames;
static int read_fault,write_short;
static uint64_t time_offset;
static uint64_t fake_monotonic_us(void){return monotonic_us()+time_offset;}
static ssize_t fake_read(int fd,void *data,size_t size) {
    if(read_fault){errno=read_fault;return -1;}
    if(mode && !strcmp(mode,"read-error")){errno=EIO;return -1;}
    return read(fd,data,size);
}
static ssize_t fake_write(int fd,const void *data,size_t size) {
    if(write_short)return (ssize_t)size-1;
    return write(fd,data,size);
}
static int fake_poll(struct pollfd *descriptors,nfds_t count,int timeout) {
    if(mode && !strcmp(mode,"read-error")){descriptors[0].revents=POLLIN;return 1;}
    return poll(descriptors,count,timeout);
}
static int fake_open(const char *path,int flags,...) {
    if(!strncmp(path,"/dev/i2c-",9))return open("/dev/null",O_RDWR);
    if(flags&O_CREAT) {
        va_list args;va_start(args,flags);int permissions=va_arg(args,int);va_end(args);
        return open(path,flags,permissions);
    }
    return open(path,flags);
}
uint8_t fake_alive(VL53L5CX_Configuration *d,uint8_t *alive){(void)d;*alive=1;return 0;}
uint8_t fake_init(VL53L5CX_Configuration *d){
    (void)d;init_count++;
    if(!strcmp(mode,"slow-init") && init_count==2)time_offset+=6000000ULL;
    if(!strcmp(mode,"init-stop"))running=0;
    return !strcmp(mode,"init-fail");
}
uint8_t fake_resolution(VL53L5CX_Configuration *d,uint8_t resolution){(void)d;assert(resolution==64);return 0;}
uint8_t fake_frequency(VL53L5CX_Configuration *d,uint8_t hz){(void)d;assert(hz>=1&&hz<=15);return 0;}
uint8_t fake_start(VL53L5CX_Configuration *d){(void)d;return !strcmp(mode,"start-fail");}
uint8_t fake_stop(VL53L5CX_Configuration *d){(void)d;stop_count++;return !strcmp(mode,"stop-fail");}
uint8_t fake_ready(VL53L5CX_Configuration *d,uint8_t *ready){
    (void)d;*ready=strcmp(mode,"uart")!=0;
    if(!strcmp(mode,"ready-fail"))return 1;
    if(!strcmp(mode,"good") || !strcmp(mode,"stop-fail") || !strcmp(mode,"slow-init")){if(frames>=2)running=0;}
    return 0;
}
uint8_t fake_data(VL53L5CX_Configuration *d,VL53L5CX_ResultsData *r){
    (void)d;
    if(!strcmp(mode,"read-fail"))return 1;
    frames++;memset(r,0,sizeof(*r));
    for(int z=0;z<64;z++){r->distance_mm[z]=1000;r->target_status[z]=5;r->nb_target_detected[z]=1;}
    return 0;
}
static void units(void) {
    unsigned value=99;
    const char *bad[]={"","-1","+10"," 10","10x","4294967296","18446744073709551616","\v10"};
    for(size_t i=0;i<sizeof(bad)/sizeof(bad[0]);i++)assert(!parse_unsigned(bad[i],10,&value));
    assert(parse_unsigned("15",10,&value)&&value==15);
    Sensor sensor={0};assert(parse_sensor("/dev/i2c-1:0x29",&sensor)&&sensor.address==41);
    assert(parse_sensor("/dev/i2c-1:2f",&sensor)&&sensor.address==47);
    assert(!parse_sensor("/dev/i2c-:0x29",&sensor));
    assert(!parse_sensor("/dev/i2c-1/../2:0x29",&sensor));
    assert(!parse_sensor("/dev/i2c-1:0x100000029",&sensor));
    int descriptors[2];assert(!pipe(descriptors));
    assert(!fcntl(descriptors[0],F_SETFL,O_NONBLOCK));
    assert(!poll_mavlink(descriptors[0]));
    assert(write(descriptors[1],"x",1)==1);
    read_fault=EAGAIN;assert(!poll_mavlink(descriptors[0]));
    read_fault=EINTR;assert(!poll_mavlink(descriptors[0]));
    read_fault=EIO;assert(poll_mavlink(descriptors[0])==EXIT_SERIAL);
    read_fault=0;uint8_t byte;assert(read(descriptors[0],&byte,1)==1);
    close(descriptors[1]);
    assert(poll_mavlink(descriptors[0])==EXIT_SERIAL);close(descriptors[0]);
    assert(poll_mavlink(-2)==0); /* poll ignores negative fd; collector never calls it. */
    assert(poll_mavlink(1000000)==EXIT_SERIAL);
    /* Maximal legal storage values must still fit an atomic Linux pipe line. */
    for(int z=0;z<64;z++){
        sensor.result.distance_mm[z]=INT16_MIN;sensor.result.target_status[z]=UINT8_MAX;
        sensor.result.nb_target_detected[z]=UINT8_MAX;sensor.result.signal_per_spad[z]=UINT32_MAX;
        sensor.result.ambient_per_spad[z]=UINT32_MAX;sensor.result.range_sigma_mm[z]=UINT16_MAX;
    }
    sensor.sequence=UINT64_MAX;sensor.poll_start=UINT64_MAX;
    sensor.read_start=1;sensor.read_end=UINT64_MAX;
    sensor.device.platform.read_payload_bytes=UINT64_MAX;sensor.device.platform.write_payload_bytes=UINT64_MAX;
    assert(!emit_tof(&sensor,1,UINT64_MAX));assert(record_size<=4096);
    uint64_t sequence=sensor.sequence;
    write_short=1;assert(emit_tof(&sensor,1,UINT64_MAX)==-1 && errno==EIO);
    assert(sensor.sequence==sequence);write_short=0;
    fprintf(stderr,"PASS collector parser / hangup / max record (%zu bytes)\n",record_size);
}
int main(int argc,char **argv) {
    assert(argc>=2);mode=argv[1];
    if(!strcmp(mode,"units")){units();return 0;}
    int result=collector_main(argc-1,argv+1);
    fprintf(stderr,"TEST init=%d stop=%d frames=%d exit=%d\n",init_count,stop_count,frames,result);
    return result;
}
