/** @brief 实际算法分阶段计时；主机结果只代表运行该程序的机器。 */
#include "companion/estimator.hpp"
#include "companion/terrain.hpp"
#include "companion/armor_tracker.hpp"
#include "companion/rm_auto_aim.hpp"
#include "companion/vision.hpp"
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <sys/resource.h>
#include <sys/utsname.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>

int main(int argc,char**argv){try{
  if(argc!=4)throw std::runtime_error("benchmark IMAGE.ppm TERRAIN.f32 OUTPUT.json");
  cv::setNumThreads(1);
  auto image=companion::readPpm(argv[1]);if(!image)throw std::runtime_error("image unavailable");
  auto terrain=companion::TerrainMap::load(argv[2]);
  companion::CameraConfig camera;camera.fx_px=camera.fy_px=320;camera.cx_px=319.5;camera.cy_px=179.5;camera.strip_center_separation_m=.18409;
  companion::EstimatorConfig cfg;cfg.terrain=terrain;cfg.initial_pose={10.6,0,1.44,0,0,0};
  for(int i=0;i<3;++i){cfg.tof_mounts[i].yaw_rad=i*2*companion::kPi/3;cfg.tof_mounts[i].horizontal_fov_rad=cfg.tof_mounts[i].vertical_fov_rad=companion::kPi/4;}
  companion::Estimator estimator(cfg);companion::ArmorTracker tracker;
  nlohmann::json output;volatile double sink=0;
  auto measure=[&](const char*name,int n,auto fn){
    std::vector<double> samples;
    for(int i=0;i<n+20;++i){auto start=std::chrono::steady_clock::now();fn(i);auto end=std::chrono::steady_clock::now();if(i>=20)samples.push_back(std::chrono::duration<double,std::milli>(end-start).count());}
    std::sort(samples.begin(),samples.end());double sum=0;for(auto x:samples)sum+=x;
    output["stages"][name]={{"mean_ms",sum/n},{"p50_ms",samples[n/2]},{"p95_ms",samples[static_cast<size_t>(.95*(n-1))]},{"p99_ms",samples[static_cast<size_t>(.99*(n-1))]},{"max_ms",samples.back()},{"samples",n}};
  };
  int detected=0;
  measure("vision_detect_pnp",300,[&](int){auto r=companion::detectRmArmor(*image,camera,.093427,.108873);detected+=r.valid;sink=r.position_camera_m.norm();});
  measure("flow_height_predict",1000,[&](int i){uint64_t t=1000000+i*20000;estimator.fuseHeightAssist({t,1.44,"SIM_HEIGHT_ASSIST"});estimator.fuseOpticalFlow({t,0,0,.95,.0004});sink=estimator.estimate().pose.z_m;});
  measure("target_tracker",1000,[&](int i){uint64_t t=1000000+i*20000;tracker.observe(t,Eigen::Vector3d(11,.1*std::sin(i*.02),1.44));sink=tracker.at(t).position.y();});
  companion::TofFrame frame;frame.sensor_index=0;
  for(int cell=0;cell<64;++cell){companion::TerrainMap::Hit hit;auto ray=companion::Estimator::tofRayBody(cfg.tof_mounts[0],cell/8,cell%8);if(terrain->raycast({10.6,0,1.44},ray,4,hit)){frame.valid[cell]=1;frame.distance_m[cell]=hit.depth_m;}}
  companion::Estimator tof_estimator(cfg);
  measure("tof_frame",300,[&](int i){frame.sim_time_us=1000000+i*67000;sink=tof_estimator.fuseTof(frame).inliers;});
  struct rusage usage{};getrusage(RUSAGE_SELF,&usage);struct utsname host{};uname(&host);
  output["host_arch"]=host.machine;output["host_name"]=host.nodename;output["opencv"]=CV_VERSION;output["opencv_threads"]=cv::getNumThreads();
  output["peak_rss_kib"]=usage.ru_maxrss;output["terrain_storage_bytes"]=terrain->storageBytes();output["image_bytes"]=image->rgb.size();output["detected_iterations"]=detected;
  output["measurement_scope"]="host algorithm microbenchmark, not calibrated Nano timing";
  std::ofstream file(argv[3]);file<<output.dump(2)<<'\n';if(!file)throw std::runtime_error("output failed");std::cout<<output.dump(2)<<'\n';(void)sink;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
