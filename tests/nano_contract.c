/* Exercise the real JSON encoder and byte swapping without a physical sensor. */
#define main collector_main
#include "../nano/main.c"
#undef main
#include <assert.h>
int main(void) {
    uint8_t data[8]={1,2,3,4,5,6,7,8};
    SwapBuffer(data,8);
    assert(data[0]==4 && data[3]==1 && data[4]==8 && data[7]==5);
    Sensor sensor={0};
    for(int i=0;i<64;i++) {
        sensor.result.distance_mm[i]=(int16_t)(1000+i);
        sensor.result.target_status[i]=5;
        sensor.result.nb_target_detected[i]=1;
        sensor.result.signal_per_spad[i]=15;
        sensor.result.ambient_per_spad[i]=2;
        sensor.result.range_sigma_mm[i]=4;
    }
    sensor.result.target_status[5]=0;
    sensor.poll_start=1234500;
    sensor.read_start=1234567;sensor.read_end=1234667;
    assert(emit_tof(&sensor,1,1234567)==0);
    assert(sensor.sequence==1);
    return 0;
}
