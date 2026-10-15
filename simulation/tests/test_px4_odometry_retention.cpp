/** @file Native uORB publication retention and latest-sample subscription checks. */
#include <uORB/uORBDeviceNode.hpp>
#include <uORB/uORBManager.hpp>
#include <uORB/topics/vehicle_odometry.h>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace {
bool retained(uORB::DeviceNode& node, vehicle_odometry_s& sample, unsigned& generation) {
#ifdef RETENTION_CANDIDATE
  return uORB::Manager::orb_data_copy_retained(&node, &sample, generation);
#else
  return uORB::Manager::orb_data_copy(&node, &sample, generation, true);
#endif
}
void publish(uORB::DeviceNode& node, uint64_t stamp) {
  vehicle_odometry_s sample{};
  sample.timestamp=stamp;
  assert(node.write(nullptr,reinterpret_cast<const char*>(&sample),sizeof(sample))==sizeof(sample));
}
}

int main() {
  uORB::DeviceNode node(ORB_ID(vehicle_visual_odometry),0,"/obj/retention_contract");
  node.mark_as_advertised();
  vehicle_odometry_s sample{};
  unsigned logger=0,control=0,independent=0;
  assert(node.get_queue_size()==1);
  for(uint64_t stamp=1;stamp<=3;++stamp)publish(node,stamp);
  for(uint64_t expected=1;expected<=3;++expected) {
    assert(retained(node,sample,logger));
    assert(sample.timestamp==expected&&logger==expected);
    assert(uORB::Manager::orb_data_copy(&node,&sample,control,false)&&sample.timestamp==3&&control==3);
  }
  assert(!retained(node,sample,logger));
  for(uint64_t expected=1;expected<=3;++expected) {
    assert(retained(node,sample,independent)&&sample.timestamp==expected);
  }
  for(uint64_t stamp=4;stamp<=15;++stamp)publish(node,stamp);
  for(uint64_t expected=8;expected<=15;++expected) {
    const unsigned before=logger;
    assert(retained(node,sample,logger)&&sample.timestamp==expected&&logger==expected);
    if(expected==8)assert(logger!=before+1);
  }
  assert(!retained(node,sample,logger));
  unsigned late=node.get_initial_generation();
  assert(retained(node,sample,late)&&sample.timestamp==15);
  uORB::DeviceNode ordinary(ORB_ID(vehicle_odometry),0,"/obj/ordinary_contract");
  ordinary.mark_as_advertised();
  publish(ordinary,1);publish(ordinary,2);
  unsigned ordinary_reader=0;
  assert(retained(ordinary,sample,ordinary_reader)&&sample.timestamp==2);
  uORB::DeviceNode threaded(ORB_ID(vehicle_visual_odometry),0,"/obj/thread_contract");
  threaded.mark_as_advertised();
  std::atomic<bool> done{false};
  std::thread writer([&](){for(uint64_t n=1;n<=100000;++n)publish(threaded,n);done=true;});
  unsigned cursor=0;uint64_t previous=0,reads=0;
  while(!done||threaded.updates_available(cursor)) {
    if(retained(threaded,sample,cursor)) {
      assert(sample.timestamp>previous&&sample.timestamp<=100000);
      previous=sample.timestamp;++reads;
    }
  }
  writer.join();
  assert(previous==100000&&cursor==100000);
  std::printf("{\"native_publications\":100000,\"native_retained_reads\":%llu,\"burst_and_overflow_passed\":true,\"normal_readers_latest\":true}\n",static_cast<unsigned long long>(reads));
}
