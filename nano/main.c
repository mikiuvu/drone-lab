#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "vl53l5cx_api.h"
#include "common/mavlink.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

enum { EXIT_SENSOR=4, EXIT_SERIAL=5, EXIT_OUTPUT=6 };
static volatile sig_atomic_t running=1;
static void interrupted(int number) {(void)number;running=0;}
/* One complete JSON line per write. A full pipe is a visible collector failure. */
static char record[4096];
static size_t record_size,record_limit=sizeof(record);
static bool record_valid;
static void append_record(const char *format,...) {
    if(!record_valid)return;
    va_list args;va_start(args,format);
    int size=vsnprintf(record+record_size,sizeof(record)-record_size,format,args);
    va_end(args);
    if(size<0 || (size_t)size>=sizeof(record)-record_size){record_valid=false;return;}
    record_size+=(size_t)size;
}
static void begin_record(const char *kind) {
    record_size=0;record_valid=true;
    append_record("{\"schema\":1,\"type\":\"%s\",\"source\":\"NANO\",",kind);
}
static int finish_record(void) {
    if(!record_valid || record_size>record_limit){errno=EMSGSIZE;return -1;}
    ssize_t size;
    do {size=write(STDOUT_FILENO,record,record_size);} while(size<0 && errno==EINTR && running);
    if(size<0)return -1;
    if((size_t)size!=record_size){errno=EIO;return -1;}
    return 0;
}
static int prepare_output(int *original_flags) {
    struct stat info;
    *original_flags=fcntl(STDOUT_FILENO,F_GETFL);
    if(*original_flags<0 || fstat(STDOUT_FILENO,&info))return -1;
    if(S_ISFIFO(info.st_mode)) {
        long limit=fpathconf(STDOUT_FILENO,_PC_PIPE_BUF);
        if(limit<1){errno=EINVAL;return -1;}
        if((size_t)limit<record_limit)record_limit=(size_t)limit;
    }
    return fcntl(STDOUT_FILENO,F_SETFL,*original_flags|O_NONBLOCK);
}
typedef struct {
    VL53L5CX_Configuration device;
    VL53L5CX_ResultsData result;
    char bus[128];unsigned address;
    uint64_t sequence,last_frame,poll_start,read_start,read_end;
    bool started;
} Sensor;
static int emit_tof(Sensor *s,int number,uint64_t ready_time) {
    VL53L5CX_ResultsData *r=&s->result;begin_record("tof");
    append_record("\"sensor\":%d,\"sequence\":%llu,\"sensor_streamcount\":%u,"
           "\"source_time_us\":%llu,\"read_end_us\":%llu,\"time_basis\":\"nano_data_ready_poll\","
           "\"poll_start_us\":%llu,\"read_start_us\":%llu,\"data_read_us\":%llu,"
           "\"i2c_read_payload_bytes_total\":%llu,\"i2c_write_payload_bytes_total\":%llu,"
           "\"temperature_c\":%d,\"distance_mm\":[",number,(unsigned long long)s->sequence,s->device.streamcount,
           (unsigned long long)ready_time,(unsigned long long)s->read_end,
           (unsigned long long)s->poll_start,(unsigned long long)s->read_start,
           (unsigned long long)(s->read_end-s->read_start),
           (unsigned long long)s->device.platform.read_payload_bytes,
           (unsigned long long)s->device.platform.write_payload_bytes,r->silicon_temp_degc);
    for(int z=0;z<64;z++)append_record("%s%d",z?",":"",r->distance_mm[z]);
    append_record("],\"status\":[");for(int z=0;z<64;z++)append_record("%s%u",z?",":"",r->target_status[z]);
    append_record("],\"targets\":[");for(int z=0;z<64;z++)append_record("%s%u",z?",":"",r->nb_target_detected[z]);
    append_record("],\"signal_kcps_spad\":[");for(int z=0;z<64;z++)append_record("%s%u",z?",":"",r->signal_per_spad[z]);
    append_record("],\"ambient_kcps_spad\":[");for(int z=0;z<64;z++)append_record("%s%u",z?",":"",r->ambient_per_spad[z]);
    append_record("],\"sigma_mm\":[");for(int z=0;z<64;z++)append_record("%s%u",z?",":"",r->range_sigma_mm[z]);
    append_record("]}\n");
    if(finish_record())return -1;
    s->sequence++;return 0;
}
static bool parse_unsigned(const char *text,int base,unsigned *value) {
    if(!text || !*text)return false;
    /* strtoul accepts signs and whitespace; the CLI accepts only unsigned digits. */
    if(!((*text>='0'&&*text<='9') || (base==16 &&
        ((*text>='a'&&*text<='f')||(*text>='A'&&*text<='F')))))return false;
    char *end;errno=0;unsigned long parsed=strtoul(text,&end,base);
    if(errno || *end || parsed>UINT_MAX)return false;
    *value=(unsigned)parsed;return true;
}
static bool parse_sensor(const char *text,Sensor *s) {
    const char *colon=strchr(text,':');
    if(!colon || (size_t)(colon-text)>=sizeof(s->bus) || strncmp(text,"/dev/i2c-",9) || colon<=text+9)return false;
    for(const char *p=text+9;p<colon;p++)if(*p<'0'||*p>'9')return false;
    if(!parse_unsigned(colon+1,16,&s->address) || s->address<8 || s->address>0x77)return false;
    memcpy(s->bus,text,(size_t)(colon-text));s->bus[colon-text]=0;return true;
}
static bool serial_speed(unsigned baud,speed_t *speed) {
    switch(baud) {
        case 57600:*speed=B57600;break;case 115200:*speed=B115200;break;case 230400:*speed=B230400;break;
        case 460800:*speed=B460800;break;case 921600:*speed=B921600;break;
        default:return false;
    }
    return true;
}
static int open_serial(const char *path,unsigned baud) {
    speed_t speed;if(!serial_speed(baud,&speed)){errno=EINVAL;return -1;}
    int fd=open(path,O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);if(fd<0){perror(path);return -1;}
    struct termios tty;if(tcgetattr(fd,&tty)){perror("tcgetattr");close(fd);return -1;}
    cfmakeraw(&tty);tty.c_cflag|=CLOCAL|CREAD;tty.c_cflag&=~CRTSCTS;
    tty.c_cc[VMIN]=1;tty.c_cc[VTIME]=0;
    if(cfsetispeed(&tty,speed)||cfsetospeed(&tty,speed)||tcsetattr(fd,TCSANOW,&tty)) {
        perror("UART configure");close(fd);return -1;
    }
    return fd;
}
static uint64_t uart_bytes_total;
/* Bounded drain: continuous traffic must also leave time for the two ToF sensors. */
static int poll_mavlink(int fd) {
    uint8_t bytes[2048];
    static mavlink_message_t message;static mavlink_status_t status;
    for(int batch=0;batch<4 && running;batch++) {
        struct pollfd descriptor={.fd=fd,.events=POLLIN};
        int events=poll(&descriptor,1,0);
        if(events<0){if(errno==EINTR)return 0;return EXIT_SERIAL;}
        if(!events)return 0;
        if(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL)){errno=EIO;return EXIT_SERIAL;}
        if(!(descriptor.revents&POLLIN))return 0;
        ssize_t count=read(fd,bytes,sizeof(bytes));
        if(count<0) {
            if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)return 0;
            return EXIT_SERIAL;
        }
        if(!count){errno=EIO;return EXIT_SERIAL;}
        uint64_t received=monotonic_us();uart_bytes_total+=(uint64_t)count;
        for(ssize_t i=0;i<count && running;i++) {
            if(!mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&message,&status))continue;
            if(message.msgid==MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t a;mavlink_msg_attitude_decode(&message,&a);
                if(!isfinite(a.roll)||!isfinite(a.pitch)||!isfinite(a.yaw))continue;
                begin_record("attitude");
                append_record("\"system_id\":%u,\"component_id\":%u,\"source_time_us\":%llu,"
                       "\"nano_rx_monotonic_us\":%llu,\"time_basis\":\"fc_boot\","
                       "\"mavlink_sequence\":%u,\"uart_bytes_total\":%llu,"
                       "\"roll_deg\":%.5f,\"pitch_deg\":%.5f,\"yaw_deg\":%.5f}\n",
                       message.sysid,message.compid,(unsigned long long)a.time_boot_ms*1000ULL,
                       (unsigned long long)received,message.seq,(unsigned long long)uart_bytes_total,
                       a.roll*57.295779513,a.pitch*57.295779513,a.yaw*57.295779513);
                if(finish_record())return EXIT_OUTPUT;
            } else if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t h;mavlink_msg_heartbeat_decode(&message,&h);begin_record("heartbeat");
                append_record("\"system_id\":%u,\"component_id\":%u,\"armed\":%s,\"autopilot\":%u,"
                       "\"nano_rx_monotonic_us\":%llu,\"mavlink_sequence\":%u,\"uart_bytes_total\":%llu}\n",
                       message.sysid,message.compid,(h.base_mode&128)?"true":"false",h.autopilot,
                       (unsigned long long)received,message.seq,(unsigned long long)uart_bytes_total);
                if(finish_record())return EXIT_OUTPUT;
            }
        }
    }
    return 0;
}
static void usage(void) {
    puts("drone-nano --sensor /dev/i2c-N:0x29 [--sensor /dev/i2c-M:0x29] [--hz 10] [--mavlink /dev/ttyS1 --baud 115200]");
    puts("Addresses are 7-bit. Same bus/address pairs are rejected. Default 8x8 / 10 Hz. No flight commands are sent.");
}
int main(int argc,char **argv) {
    Sensor sensors[2]={0};int count=0,result=1,serial_fd=-1,output_flags=-1;
    unsigned hz=10,baud=115200;const char *serial_path=NULL;speed_t speed;
    running=1;uart_bytes_total=0;record_limit=sizeof(record);
    for(int n=0;n<2;n++)sensors[n].device.platform.fd=-1;
    for(int i=1;i<argc;i++) {
        if(!strcmp(argv[i],"--help")){usage();return 0;}if(i+1>=argc){usage();return 2;}
        if(!strcmp(argv[i],"--sensor")) {
            if(count>=2 || !parse_sensor(argv[++i],&sensors[count])){fprintf(stderr,"Invalid sensor argument (maximum two sensors)\n");return 2;}
            count++;
        } else if(!strcmp(argv[i],"--hz")) {
            if(!parse_unsigned(argv[++i],10,&hz)){fprintf(stderr,"Invalid Hz\n");return 2;}
        } else if(!strcmp(argv[i],"--baud")) {
            if(!parse_unsigned(argv[++i],10,&baud)){fprintf(stderr,"Invalid baud rate\n");return 2;}
        } else if(!strcmp(argv[i],"--mavlink"))serial_path=argv[++i];else{usage();return 2;}
    }
    if(count<1||hz<1||hz>15 || !serial_speed(baud,&speed) ||
       (serial_path && (strncmp(serial_path,"/dev/",5) || !serial_path[5]))) {usage();return 2;}
    if(count==2&&!strcmp(sensors[0].bus,sensors[1].bus)&&sensors[0].address==sensors[1].address){
        fprintf(stderr,"Two sensors cannot share a bus/address. Use separate buses or assign addresses using LPn first.\n");return 2;
    }
    struct sigaction action={0};action.sa_handler=interrupted;sigemptyset(&action.sa_mask);
    if(sigaction(SIGINT,&action,NULL)||sigaction(SIGTERM,&action,NULL)||sigaction(SIGPIPE,&action,NULL))return 1;
    int lock=open("/tmp/drone-nano.lock",O_CREAT|O_RDWR|O_CLOEXEC,0600);
    if(lock<0||flock(lock,LOCK_EX|LOCK_NB)){fprintf(stderr,"Another collector is running (or lock unavailable)\n");if(lock>=0)close(lock);return 3;}
    if(prepare_output(&output_flags)){perror("stdout configure");result=EXIT_OUTPUT;goto cleanup;}
    if(serial_path){serial_fd=open_serial(serial_path,baud);if(serial_fd<0){result=EXIT_SERIAL;goto cleanup;}}
    for(int n=0;n<count && running;n++) {
        Sensor *s=&sensors[n];VL53L5CX_Platform *p=&s->device.platform;
        p->running=&running;p->deadline_us=monotonic_us()+30000000ULL;
        p->fd=open(s->bus,O_RDWR|O_CLOEXEC);p->address=(uint16_t)(s->address<<1);
        if(p->fd<0){perror(s->bus);goto cleanup;}
        uint8_t alive=0;fprintf(stderr,"Sensor %d: probe %s at 0x%02x\n",n+1,s->bus,s->address);
        if(vl53l5cx_is_alive(&s->device,&alive)||!alive){fprintf(stderr,"Sensor %d not detected\n",n+1);goto cleanup;}
        fprintf(stderr,"Sensor %d: loading ST firmware\n",n+1);
        if(!running||vl53l5cx_init(&s->device)||!running||vl53l5cx_set_resolution(&s->device,64)||
           !running||vl53l5cx_set_ranging_frequency_hz(&s->device,(uint8_t)hz)||!running) {
            fprintf(stderr,"Sensor %d initialization failed\n",n+1);goto cleanup;
        }
        /* Start can reach hardware before reporting failure; cleanup must try stop. */
        s->started=true;
        if(vl53l5cx_start_ranging(&s->device)||!running){fprintf(stderr,"Sensor %d ranging start failed\n",n+1);goto cleanup;}
        p->deadline_us=0;
    }
    if(!running)goto cleanup;
    /* Sensor 1 must not time out just because sensor 2 firmware loading took time. */
    for(int n=0;n<count;n++)sensors[n].last_frame=monotonic_us();
    fprintf(stderr,"Capture ready; polling timestamps are not exposure timestamps. UART is receive-only.\n");result=0;
    while(running) {
        if(serial_fd>=0 && (result=poll_mavlink(serial_fd)))goto stream_failure;
        for(int n=0;n<count && running;n++) {
            Sensor *s=&sensors[n];uint8_t ready=0;s->poll_start=monotonic_us();
            if(vl53l5cx_check_data_ready(&s->device,&ready)){fprintf(stderr,"Sensor %d readiness check failed\n",n+1);result=EXIT_SENSOR;goto cleanup;}
            if(!running)break;
            if(ready) {
                uint64_t stamp=monotonic_us();s->read_start=stamp;
                if(vl53l5cx_get_ranging_data(&s->device,&s->result)){fprintf(stderr,"Sensor %d frame read failed\n",n+1);result=EXIT_SENSOR;goto cleanup;}
                s->read_end=monotonic_us();s->last_frame=s->read_end;
                if(!running)break;
                if(emit_tof(s,n,stamp)){result=EXIT_OUTPUT;goto stream_failure;}
            }
            if(monotonic_us()-s->last_frame>5000000ULL){fprintf(stderr,"Sensor %d: no new data for 5 seconds\n",n+1);result=EXIT_SENSOR;goto cleanup;}
            if(serial_fd>=0 && running && (result=poll_mavlink(serial_fd)))goto stream_failure;
        }
        if(running && WaitMs(NULL,2)){result=EXIT_SENSOR;goto cleanup;}
    }
    goto cleanup;
stream_failure:
    perror(result==EXIT_SERIAL?"UART receive failed":"JSON output failed (slow/disconnected consumer or partial write)");
cleanup:
    for(int n=0;n<count;n++) {
        VL53L5CX_Platform *p=&sensors[n].device.platform;
        if(sensors[n].started) {
            /* Cleanup has its own budget and ignores the acquisition stop flag. */
            p->running=NULL;p->io_errno=0;p->deadline_us=monotonic_us()+250000ULL;
            if(vl53l5cx_stop_ranging(&sensors[n].device)||p->io_errno) {
                fprintf(stderr,"Sensor %d stop failed; hardware state unconfirmed\n",n+1);
                if(!result)result=EXIT_SENSOR;
            }
        }
        if(p->fd>=0)close(p->fd);
    }
    if(serial_fd>=0)close(serial_fd);
    if(output_flags>=0 && fcntl(STDOUT_FILENO,F_SETFL,output_flags)<0 && !result)result=EXIT_OUTPUT;
    close(lock);return result;
}
