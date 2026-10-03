#include "companion/vision_delay.hpp"
#include <cassert>
int main(){
 companion::VisionDelay<int> zero(0);zero.submit(1,100);assert(*zero.poll(100)==1);assert(!zero.poll(100));
 companion::VisionDelay<int> queue(100000);queue.submit(1,0);queue.submit(2,20000);queue.submit(3,40000);
 assert(!queue.poll(99999));assert(*queue.poll(100000)==1);assert(!queue.poll(100000));
 assert(!queue.poll(199999));assert(*queue.poll(200000)==3);assert(!queue.poll(300000));
 assert(queue.received==3&&queue.completed==2&&queue.dropped==1);
}
