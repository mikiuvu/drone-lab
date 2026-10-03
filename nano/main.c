/**
 * @file main.c
 * @brief Nano 实板采集入口：轮询最多两颗 VL53L5CX，接收 MAVLink，并输出逐行 JSON。
 * @note stdout 只输出 JSONL，诊断写 stderr；传感器配置和测距会改变硬件状态，串口只接收。
 */
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

// 运行期故障分别表示传感器、串口和输出错误；初始化失败还可返回 1。
enum { EXIT_SENSOR=4, EXIT_SERIAL=5, EXIT_OUTPUT=6 };
// 信号处理函数只改变 sig_atomic_t 标志，资源清理由主循环统一执行。
static volatile sig_atomic_t running=1;
static void interrupted(int number) {(void)number;running=0;}
/* 先组装完整 JSON 行再一次写出；消费者过慢时报告失败，避免丢帧或写出半行后继续。 */
static char record[4096];
static size_t record_size,record_limit=sizeof(record);
static bool record_valid;
/** @brief 将格式化片段追加到当前记录；容量不足后锁存无效状态，后续追加不再写入。
 * @param format printf 格式及随后对应的可变参数，文本属于同一条记录。
 */
static void append_record(const char *format,...) {
    if(!record_valid)return;
    va_list args;va_start(args,format);
    int size=vsnprintf(record+record_size,sizeof(record)-record_size,format,args);
    va_end(args);
    if(size<0 || (size_t)size>=sizeof(record)-record_size){record_valid=false;return;}
    record_size+=(size_t)size;
}
/** @brief 开始一条 schema=1、source=NANO 的记录，重置长度与有效标志。
 * @param kind 内部固定的记录类型字符串，须适合直接放入 JSON 字符串。
 */
static void begin_record(const char *kind) {
    record_size=0;record_valid=true;
    append_record("{\"schema\":1,\"type\":\"%s\",\"source\":\"NANO\",",kind);
}
/** @brief 一次写出已组装记录；由调用方追加 JSON 结束符及换行。
 * @return 0 完整写出，-1 容量超限、短写或系统调用失败，errno 标明原因。
 * @note 不重试短写；管道中小于等于 PIPE_BUF 的非阻塞写避免记录交错。
 */
static int finish_record(void) {
    if(!record_valid || record_size>record_limit){errno=EMSGSIZE;return -1;}
    ssize_t size;
    do {size=write(STDOUT_FILENO,record,record_size);} while(size<0 && errno==EINTR && running);
    if(size<0)return -1;
    if((size_t)size!=record_size){errno=EIO;return -1;}
    return 0;
}
/** @brief 保存 stdout 标志，按管道原子写上限限制记录大小，并启用非阻塞输出。
 * @param original_flags 输出原有标志，供退出时恢复。
 * @return 0 成功，-1 检查或配置失败。
 */
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
/** @brief 单颗传感器的 ULD 状态和采集时序，设备之间不共享帧缓存。 */
typedef struct {
    VL53L5CX_Configuration device; // ULD 配置，内嵌 Linux 平台句柄。
    VL53L5CX_ResultsData result; // 最近一次成功读取的 64 区结果。
    char bus[128];unsigned address; // 设备路径与 CLI 的 7 位 I²C 地址。
    uint64_t sequence,last_frame,poll_start,read_start,read_end; // sequence 为输出帧序号，其余为 Nano 单调微秒。
    bool started; // 已尝试启动测距，退出时应尝试停止，即使启动调用报错。
} Sensor;
/** @brief 将 ULD 的 64 区结果及采集诊断编码成一条 tof JSONL。
 * @param s 已读取有效结果的设备状态，成功输出后 sequence 加一。
 * @param number 传感器索引，从 0 开始。
 * @param ready_time 发现数据就绪后的 Nano 单调微秒，不是曝光时间。
 * @return 0 完整输出，-1 输出失败。
 * @note 数组保留原区序；距离和 sigma 单位 mm，温度 °C，信号和环境光 kcps/SPAD。
 */
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
/** @brief 严格解析无符号整数，拒绝前导符号、空白、尾部字符与溢出。
 * @param text 输入字符串。
 * @param base 十进制或十六进制基数；地址允许 0x 前缀。
 * @param value 成功时接收结果。
 * @return true 解析成功，false 格式或范围错误。
 */
static bool parse_unsigned(const char *text,int base,unsigned *value) {
    if(!text || !*text)return false;
    /* strtoul 本身接受符号和空白，CLI 先检查首字符以约束输入。 */
    if(!((*text>='0'&&*text<='9') || (base==16 &&
        ((*text>='a'&&*text<='f')||(*text>='A'&&*text<='F')))))return false;
    char *end;errno=0;unsigned long parsed=strtoul(text,&end,base);
    if(errno || *end || parsed>UINT_MAX)return false;
    *value=(unsigned)parsed;return true;
}
/** @brief 解析 /dev/i2c-N:十六进制地址，限制地址为 0x08～0x77。
 * @param text 总线与地址字符串。
 * @param s 成功时写入 bus 与 address，不打开设备。
 * @return true 语法有效，false 路径、长度或地址不合法。
 */
static bool parse_sensor(const char *text,Sensor *s) {
    const char *colon=strchr(text,':');
    if(!colon || (size_t)(colon-text)>=sizeof(s->bus) || strncmp(text,"/dev/i2c-",9) || colon<=text+9)return false;
    for(const char *p=text+9;p<colon;p++)if(*p<'0'||*p>'9')return false;
    if(!parse_unsigned(colon+1,16,&s->address) || s->address<8 || s->address>0x77)return false;
    memcpy(s->bus,text,(size_t)(colon-text));s->bus[colon-text]=0;return true;
}
/** @brief 将支持的波特率映射到 termios 常量；baud 单位 bit/s，speed 接收常量。
 * @return true 支持该波特率，false 未支持。
 */
static bool serial_speed(unsigned baud,speed_t *speed) {
    switch(baud) {
        case 57600:*speed=B57600;break;case 115200:*speed=B115200;break;case 230400:*speed=B230400;break;
        case 460800:*speed=B460800;break;case 921600:*speed=B921600;break;
        default:return false;
    }
    return true;
}
/** @brief 以只读、非阻塞方式打开 UART，配置原始数据模式并关闭硬件流控。
 * @param path 串口设备路径。
 * @param baud 支持的波特率，单位 bit/s。
 * @return 非负文件描述符，由调用方关闭；-1 打开或配置失败。
 */
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
/** @brief 有界地接收串口并逐字节解析 MAVLink，输出姿态与心跳记录。
 * @param fd 已配置的只读 UART 文件描述符。
 * @return 0 本次接收正常或暂无数据，EXIT_SERIAL 串口故障，EXIT_OUTPUT 输出故障。
 * @note 每次最多四批、每批 2048 字节，给 ToF 轮询留出执行机会；解析状态跨调用保留。
 */
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
                // 飞控启动毫秒转为 source_time_us；Nano 接收时间单独保存，两个时钟不能直接相减。
                // 姿态从 MAVLink 弧度换成度；不做坐标变换或生成控制指令。
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
/** @brief 解析参数、锁定单实例、初始化设备，并轮询采集直到停止或故障。
 * @param argc 参数数量。
 * @param argv 参数数组；至少指定一个 --sensor，可选第二颗与接收串口。
 * @return 0 正常结束；1 初始化失败，2 参数错误，3 锁失败，4/5/6 设备/串口/输出故障。
 * @note 默认 8×8、10 Hz；两颗传感器的总线和地址组合须不同，本入口不分配新 I²C 地址。
 */
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
    // 1. 单实例锁避免两个进程同时初始化传感器；输出背压作为故障处理。
    int lock=open("/tmp/drone-nano.lock",O_CREAT|O_RDWR|O_CLOEXEC,0600);
    if(lock<0||flock(lock,LOCK_EX|LOCK_NB)){fprintf(stderr,"Another collector is running (or lock unavailable)\n");if(lock>=0)close(lock);return 3;}
    if(prepare_output(&output_flags)){perror("stdout configure");result=EXIT_OUTPUT;goto cleanup;}
    if(serial_path){serial_fd=open_serial(serial_path,baud);if(serial_fd<0){result=EXIT_SERIAL;goto cleanup;}}
    // 2. 逐颗探测、下载 ST 固件、设置分辨率与频率，每颗初始化期限为 30 s。
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
        /* 启动命令可能已到达硬件后才报错，先标记以确保清理时尝试停止。 */
        s->started=true;
        if(vl53l5cx_start_ranging(&s->device)||!running){fprintf(stderr,"Sensor %d ranging start failed\n",n+1);goto cleanup;}
        p->deadline_us=0;
    }
    if(!running)goto cleanup;
    /* 两颗全部初始化后再启动失帧计时，避免第二颗固件下载导致第一颗误超时。 */
    for(int n=0;n<count;n++)sensors[n].last_frame=monotonic_us();
    fprintf(stderr,"Capture ready; polling timestamps are not exposure timestamps. UART is receive-only.\n");result=0;
    // 3. 就绪轮询后读取整帧，记录轮询、读取起止时间；连续 5 s 无新帧即退出。
    // 两颗顺序读取，不假定同步曝光；2 ms 等待只是轮询间隔，不是测距周期。
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
    // 4. 无论正常停止还是故障，都停止已尝试启动的设备并关闭句柄。
    for(int n=0;n<count;n++) {
        VL53L5CX_Platform *p=&sensors[n].device.platform;
        if(sensors[n].started) {
            /* 清理使用独立 250 ms 期限，解除采集取消标志与旧错误以便发出停止命令。 */
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
