#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include "companion/estimator.hpp"
#include "companion/protocol.hpp"
#include "companion/vision.hpp"
#include "companion/terrain.hpp"
#include <common/mavlink.h>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

using namespace companion;

static EstimatorConfig makeConfig() {
  EstimatorConfig c;
  c.vehicle_id=0; c.initial_pose.x_m=-3; c.initial_pose.y_m=-2; c.initial_pose.z_m=.2;
  c.initial_covariance.setIdentity(); c.initial_covariance*=.02; c.initial_covariance(2,2)=.25;
  c.map_lines.push_back({Eigen::Vector2d(-6,-5),Eigen::Vector2d(-6,5),0,4});
  c.map_lines.push_back({Eigen::Vector2d(6,-5),Eigen::Vector2d(6,5),0,4});
  c.map_lines.push_back({Eigen::Vector2d(-6,-5),Eigen::Vector2d(6,-5),0,4});
  c.map_lines.push_back({Eigen::Vector2d(-6,5),Eigen::Vector2d(6,5),0,4});
  for(int i=0;i<3;++i){c.tof_mounts[i].yaw_rad=(i==1?120:i==2?-120:0)*kPi/180;c.tof_mounts[i].pitch_down_rad=20*kPi/180;c.tof_mounts[i].horizontal_fov_rad=c.tof_mounts[i].vertical_fov_rad=kPi/4;}
  return c;
}

static void heightAndPrediction() {
  Estimator e(makeConfig()); e.reset(); ImuSample imu; imu.sim_time_us=1000000; imu.attitude_valid=true; imu.acceleration_valid=true; imu.acceleration_enu_mps2=Eigen::Vector3d(1,0,0); e.predict(imu);
  assert(!e.estimate().xy_valid&&e.estimate().localization_mode=="INITIALIZING");
  assert(e.fuseHeightAssist({1010000,1.0,"SIM_HEIGHT_ASSIST"}).accepted);
  assert(e.fuseHeightAssist({1020000,1.1,"SIM_HEIGHT_ASSIST"}).accepted);
  assert(e.estimate().z_valid&&e.estimate().vz_derived_from_height);
  assert(e.estimate().covariance.allFinite());
  assert(!e.fuseHeightAssist({1030000,1.1,"TRUTH"}).accepted);
  e.advanceTo(5000001); assert(!e.estimate().xy_valid);
}

static void peerGate() {
  Estimator e(makeConfig()); e.reset(1000000); PeerEstimate p; p.vehicle_id=1;p.sim_time_us=1000000;p.pose.x_m=-2;p.pose.y_m=-2;p.source="ESTIMATE";
  assert(!e.fuseUwb({1000000,1,1,0,true},p).accepted);
  p.xy_structural_valid=true;
  assert(e.fuseUwb({1000000,1,1,0,true},p).accepted);
  assert(e.estimate().covariance.allFinite());
  assert(e.fuseUwb({1100000,1,30,2,true},p).gated);
  p.source="TRUTH"; assert(!e.fuseUwb({1200000,1,1,0,true},p).accepted);
}

static void sensorAndVisionGates() {
  auto c=makeConfig(); auto ray=Estimator::tofRayBody(c.tof_mounts[0],3,3); assert(ray.x()>0&&ray.z()<0);
  PpmImage image; image.width=64; image.height=48; image.rgb.assign(64*48*3,0); CameraConfig camera; camera.fx_px=50; camera.cx_px=32; assert(!detectArmorStrips(image,camera).valid);
  const auto good=parseHeightAssistDatagram("{\"schema\":1,\"kind\":\"height_assist\",\"vehicle\":0,\"sim_time_us\":1,\"z_world_m\":2,\"source\":\"SIM_HEIGHT_ASSIST\"}",0); assert(good);
  // 滑轨观测样本报文：跨机共享用的是原始观测，必须能原样往返，并挡住自己的回声、
  // 截断报文和乱序样本（merge 是线性归并，时间轴乱了整条模板就废了）。
  const std::vector<PeriodicSample> samples{{1000000,.1},{1050000,-.2},{1100000,.3}};
  const auto text=serializePeriodicSamplesDatagram(samples,1);
  const auto back=parsePeriodicSamplesDatagram(text,0);
  assert(back&&back->size()==3&&std::abs((*back)[1].y+.2)<1e-9&&(*back)[2].stamp==1100000);
  assert((*back)[0].source==1&&(*back)[2].source==1);
  assert(!parsePeriodicSamplesDatagram(serializePeriodicSamplesDatagram(samples,-1),0));
  assert(!parsePeriodicSamplesDatagram(serializePeriodicSamplesDatagram(samples,256),0));
  assert(!parsePeriodicSamplesDatagram(R"({"schema":1,"kind":"periodic_samples","source":"ESTIMATE","vehicle":4294967297,"stamp_us":[1000000],"y_m":[0.1]})",0));
  assert(!parsePeriodicSamplesDatagram(text,1));
  assert(!parsePeriodicSamplesDatagram(text.substr(0,text.size()/2),0));
  const std::vector<PeriodicSample> unordered{{1100000,.3},{1000000,.1}};
  assert(!parsePeriodicSamplesDatagram(serializePeriodicSamplesDatagram(unordered,1),0));
  assert(!parsePeriodicSamplesDatagram(serializePeriodicSamplesDatagram({},1),0));
}

static void mavlinkCoordinateRoundTrip() {
  Eigen::Matrix3d enu_to_ned; enu_to_ned<<0,1,0,1,0,0,0,0,-1;
  const Eigen::Vector3d enu(-3.023,-2.018,0.012);
  const Eigen::Vector3d ned=(enu_to_ned*enu).eval();
  mavlink_odometry_t outbound{}; outbound.x=ned.x(); outbound.y=ned.y(); outbound.z=ned.z();
  mavlink_message_t message; mavlink_msg_odometry_encode(200,191,&message,&outbound);
  mavlink_odometry_t decoded{}; mavlink_msg_odometry_decode(&message,&decoded);
  assert(std::abs(decoded.x-ned.x())<1e-6 && std::abs(decoded.y-ned.y())<1e-6 && std::abs(decoded.z-ned.z())<1e-6);
}

static void terrainSurfaceTests() {
  using T = TerrainMap::Triangle;
  std::vector<T> triangles{
      T{Eigen::Vector3f(-2,-2,0), Eigen::Vector3f(2,-2,0), Eigen::Vector3f(2,2,0)},
      T{Eigen::Vector3f(-2,-2,0), Eigen::Vector3f(2,2,0), Eigen::Vector3f(-2,2,0)},
      // A nearer outlier surface must win the ray query.
      T{Eigen::Vector3f(-2,-2,1), Eigen::Vector3f(2,-2,1), Eigen::Vector3f(2,2,1)},
      T{Eigen::Vector3f(-2,-2,1), Eigen::Vector3f(2,2,1), Eigen::Vector3f(-2,2,1)},
      // Degenerate face is rejected while valid geometry remains usable.
      T{Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,1,1), Eigen::Vector3f(2,2,2)},
  };
  TerrainMap map(std::move(triangles));
  assert(map.triangleCount()==4);
  TerrainMap::Hit hit;
  assert(map.raycast(Eigen::Vector3d(0,0,2),Eigen::Vector3d(0,0,-1),4,hit));
  assert(std::abs(hit.depth_m-1.0)<1e-9 && std::abs(hit.point.z()-1.0)<1e-9);
  assert(hit.normal.z()>0.99);

  TerrainMap slope({T{Eigen::Vector3f(-2,-2,0), Eigen::Vector3f(2,-2,.8f), Eigen::Vector3f(2,2,.8f)},
                    T{Eigen::Vector3f(-2,-2,0), Eigen::Vector3f(2,2,.8f), Eigen::Vector3f(-2,2,0)}});
  assert(slope.raycast(Eigen::Vector3d(0,0,2),Eigen::Vector3d(0,0,-1),4,hit));
  assert(hit.point.z()>0.35 && hit.point.z()<0.45);
  assert(hit.normal.z()>0.9 && hit.normal.x()<0.0);
  assert(slope.storageBytes()>0);
  assert(!slope.raycast(Eigen::Vector3d(0,0,2),Eigen::Vector3d(0,0,1),4,hit));

  // Side and rear ToF mounts legitimately point away from body +X after mount yaw.
  TofMount side; side.yaw_rad=120*kPi/180; side.pitch_down_rad=20*kPi/180;
  TofMount rear; rear.yaw_rad=kPi; rear.pitch_down_rad=20*kPi/180;
  assert(Estimator::tofRayBody(side,3,3).x()<0.0);
  assert(Estimator::tofRayBody(rear,3,3).x()<0.0);
  // A vertical face is a valid structural surface even with a horizontal normal.
  TerrainMap wall({T{Eigen::Vector3f(1,-2,0), Eigen::Vector3f(1,2,0), Eigen::Vector3f(1,2,3)},
                   T{Eigen::Vector3f(1,-2,0), Eigen::Vector3f(1,2,3), Eigen::Vector3f(1,-2,3)}});
  assert(wall.raycast(Eigen::Vector3d(0,0,1),Eigen::Vector3d(1,0,0),4,hit));
  assert(std::abs(hit.point.x()-1.0)<1e-9 && std::abs(hit.normal.z())<1e-9);
}

static void terrainTraversalBudgetTests() {
  const TerrainMap::Triangle face{Eigen::Vector3f(2,-1,-1),Eigen::Vector3f(2,1,-1),Eigen::Vector3f(2,0,1)};
  // 重叠面不能由最近深度剪枝；适量重复面仍须返回交点。
  TerrainMap overlap(std::vector<TerrainMap::Triangle>(8192,face));
  TerrainMap::Hit hit;
  assert(overlap.raycast({0,0,0},{1,0,0},4,hit));
  assert(hit.depth_m==2&&(hit.point-Eigen::Vector3d(2,0,0)).norm()<1e-12);
  // 资源耗尽时即使已有交点，也不能将未完成的遍历当作最近命中。
  TerrainMap exhausted(std::vector<TerrainMap::Triangle>(32768,face));
  assert(!exhausted.raycast({0,0,0},{1,0,0},4,hit));
}

static void terrainEstimatorRegression() {
  using T=TerrainMap::Triangle;
  // Two finite vertical planes in each sensor's viewing frame. All winding signs
  // must work; association is two-sided and normal sign cancels in normal equations.
  for(double yaw:{0.0,120*kPi/180,-120*kPi/180,kPi})for(bool reversed:{false,true}) {
    const Eigen::Matrix3d rot=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    std::vector<T> faces{
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,-2),Eigen::Vector3f(2,3,2)},
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,2),Eigen::Vector3f(2,-3,2)},
      T{Eigen::Vector3f(0,.3,-2),Eigen::Vector3f(3,.3,-2),Eigen::Vector3f(3,.3,2)},
      T{Eigen::Vector3f(0,.3,-2),Eigen::Vector3f(3,.3,2),Eigen::Vector3f(0,.3,2)}};
    for(auto& f:faces){for(auto& v:f)v=(rot*v.cast<double>()).cast<float>();if(reversed)std::swap(f[1],f[2]);}
    EstimatorConfig c;c.initial_pose={};c.initial_covariance*=.02;
    c.terrain=std::make_shared<TerrainMap>(std::move(faces));c.tof_mounts[0].yaw_rad=yaw;
    c.tof_mounts[0].horizontal_fov_rad=c.tof_mounts[0].vertical_fov_rad=kPi/4;
    TofFrame frame;frame.sim_time_us=1000000;frame.sensor_index=0;
    for(int cell=0;cell<64;++cell){TerrainMap::Hit hit;const auto ray=Estimator::tofRayBody(c.tof_mounts[0],cell/8,cell%8);
      if(c.terrain->raycast(Eigen::Vector3d::Zero(),ray,4,hit)){frame.valid[cell]=1;frame.distance_m[cell]=hit.depth_m;}}
    const Eigen::Vector3d offset=rot*Eigen::Vector3d(.03,0,0);c.initial_pose.x_m=offset.x();c.initial_pose.y_m=offset.y();
    Estimator e(c);const auto report=e.fuseTof(frame);assert(report.accepted&&report.inliers>=8);
    assert(Eigen::Vector2d(e.estimate().pose.x_m,e.estimate().pose.y_m).norm()<.03);
    // Tightening the trust region must reject the same nonzero correction intact.
    c.terrain_max_translation_update_m=.001;Estimator gated(c);const auto rejection=gated.fuseTof(frame);
    assert(!rejection.accepted&&rejection.gated&&rejection.reason=="terrain_update_trust_region");
    assert(std::abs(gated.estimate().pose.x_m-c.initial_pose.x_m)<1e-12);
  }
}

static void tofExposureAttitude() {
  using T=TerrainMap::Triangle;
  const auto corner=[](float y){return std::make_shared<TerrainMap>(std::vector<T>{
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,-2),Eigen::Vector3f(2,3,2)},
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,2),Eigen::Vector3f(2,-3,2)},
      T{Eigen::Vector3f(0,y,-2),Eigen::Vector3f(3,y,-2),Eigen::Vector3f(3,y,2)},
      T{Eigen::Vector3f(0,y,-2),Eigen::Vector3f(3,y,2),Eigen::Vector3f(0,y,2)}});};
  EstimatorConfig c;c.initial_pose={0,0,1,0,0,0};c.initial_covariance*=.02;
  c.terrain=corner(.3f);
  auto& mount=c.tof_mounts[0];mount.translation_body_m={.03,-.02,.015};
  mount.roll_rad=.03;mount.pitch_down_rad=-.02;mount.yaw_rad=.04;
  mount.horizontal_fov_rad=mount.vertical_fov_rad=kPi/4;
  const auto render=[](const EstimatorConfig& config,const Eigen::Matrix3d& attitude,uint64_t stamp,int sensor=0){
    TofFrame frame;frame.sensor_index=sensor;frame.sim_time_us=stamp;
    const auto& mount=config.tof_mounts[sensor];
    const Eigen::Vector3d origin=Eigen::Vector3d(0,0,1)+attitude*mount.translation_body_m;
    for(int cell=0;cell<64;++cell){
      TerrainMap::Hit hit;const Eigen::Vector3d ray=attitude*Estimator::tofRayBody(mount,cell/8,cell%8);
      if(config.terrain->raycast(origin,ray,4,hit)){frame.valid[cell]=1;frame.distance_m[cell]=hit.depth_m;}
    }
    return frame;
  };
  const auto unchanged=[](const Estimate& before,const Estimate& after){
    assert(std::abs(after.pose.x_m-before.pose.x_m)<1e-6&&std::abs(after.pose.y_m-before.pose.y_m)<1e-6);
    assert(std::abs(wrapAngle(after.pose.yaw_rad-before.pose.yaw_rad))<1e-6);
  };
  const auto frame=render(c,Eigen::Matrix3d::Identity(),1000000);
  Estimator e(c);ImuSample imu;imu.attitude_valid=true;imu.sim_time_us=1000000;imu.yaw_enu_rad=2.7;e.predict(imu);
  imu.sim_time_us=1080000;imu.roll_flu_rad=.12;imu.pitch_flu_rad=-.10;imu.yaw_enu_rad=2.82;e.predict(imu);
  const auto before=e.estimate();const auto report=e.fuseTof(frame);const auto after=e.estimate();
  // 旧曝光的零残差深度不能抵消其后真实发生的三轴转动。
  assert(report.accepted&&report.inliers>=8);unchanged(before,after);
  // 恒定三维角速度的中点曝光，包含原始航向跨越 ±π 和非零地图航向偏置。
  const Eigen::Vector3d axis=Eigen::Vector3d(.3,1,.5).normalized();
  for(double offset:{2.7,3.1}){
    Estimator interpolated(c);ImuSample start;start.sim_time_us=1000000;start.attitude_valid=true;start.yaw_enu_rad=offset;interpolated.predict(start);
    const Eigen::Matrix3d raw=(Eigen::AngleAxisd(offset,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.18,axis)).toRotationMatrix();
    auto end=start;end.sim_time_us=1080000;end.roll_flu_rad=std::atan2(raw(2,1),raw(2,2));
    end.pitch_flu_rad=std::asin(-raw(2,0));end.yaw_enu_rad=std::atan2(raw(1,0),raw(0,0));interpolated.predict(end);
    const auto middle=render(c,Eigen::AngleAxisd(.09,axis).toRotationMatrix(),1040000);
    const auto prior=interpolated.estimate();assert(interpolated.fuseTof(middle).accepted);unchanged(prior,interpolated.estimate());
  }
  // 采样中断不插值；新曝光可继续使用最新姿态的零阶保持值。
  Estimator gap(c);ImuSample start;start.sim_time_us=1000000;start.attitude_valid=true;gap.predict(start);
  auto end=start;end.sim_time_us=1200000;end.yaw_enu_rad=.16;gap.predict(end);
  const auto missing=gap.fuseTof(render(c,Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitZ()).toRotationMatrix(),1100000));
  assert(!missing.accepted&&missing.stale&&missing.reason=="tof_attitude_history_missing");
  assert(gap.fuseTof(render(c,Eigen::AngleAxisd(.16,Eigen::Vector3d::UnitZ()).toRotationMatrix(),1201000)).accepted);
  // 环形缓冲覆盖外的曝光拒绝；reset 后不再保留旧姿态或消费记录。
  Estimator bounded(c);ImuSample fixed;fixed.attitude_valid=true;fixed.yaw_enu_rad=2.7;
  for(int i=0;i<200;++i){fixed.sim_time_us=1000000+i*1000;bounded.predict(fixed);}
  const auto expired=bounded.fuseTof(frame);assert(!expired.accepted&&expired.stale);
  bounded.reset(1000000);assert(bounded.fuseTof(frame).accepted);
  // 首个有效姿态只建立增量基准，不把此前无姿态 IMU 的默认航向当成测量。
  Estimator late(c);ImuSample invalid;invalid.sim_time_us=1000000;late.predict(invalid);
  fixed.sim_time_us=1010000;late.predict(fixed);assert(late.estimate().pose.yaw_rad==0);
  fixed.sim_time_us=1020000;fixed.yaw_enu_rad=2.8;late.predict(fixed);
  assert(std::abs(late.estimate().pose.yaw_rad-.1)<1e-12);
  fixed.sim_time_us=1015000;fixed.yaw_enu_rad=-1;late.predict(fixed);
  assert(std::abs(late.estimate().pose.yaw_rad-.1)<1e-12);
  // 两路互补墙面在不同姿态曝光，合并时仍保持当前位姿。
  auto combined=c;combined.terrain=corner(2);combined.tof_mounts[1]=combined.tof_mounts[0];combined.tof_mounts[1].yaw_rad=kPi/2;
  Estimator cached(combined);cached.predict(start);
  const auto first=cached.fuseTof(render(combined,Eigen::Matrix3d::Identity(),1000000));
  assert(!first.accepted&&first.reason=="terrain_normals_not_independent");
  end.sim_time_us=1080000;end.yaw_enu_rad=.15;cached.predict(end);
  const auto prior=cached.estimate();assert(cached.fuseTof(render(combined,Eigen::AngleAxisd(.15,Eigen::Vector3d::UnitZ()).toRotationMatrix(),1080000,1)).accepted);
  unchanged(prior,cached.estimate());
  Estimator old_static(c),current_static(c);fixed.yaw_enu_rad=0;
  for(auto* estimator:{&old_static,&current_static}){
    fixed.sim_time_us=1000000;estimator->predict(fixed);fixed.sim_time_us=1080000;estimator->predict(fixed);
  }
  auto fresh=frame;fresh.sim_time_us=1080000;
  assert(old_static.fuseTof(frame).accepted&&current_static.fuseTof(fresh).accepted);
  unchanged(old_static.estimate(),current_static.estimate());
  assert((old_static.estimate().covariance-current_static.estimate().covariance).cwiseAbs().maxCoeff()==0);
}

static int terrainBenchmark(const char* path) {
  const auto map=TerrainMap::load(path); TerrainMap::Hit hit; std::size_t hits=0;
  for(int i=0;i<1000;++i) { const double a=(i%37-18)*.015,b=(i%29-14)*.015; hits+=map->raycast(Eigen::Vector3d(-14+(i%101)*.27,-8+(i%61)*.26,5),Eigen::Vector3d(a,b,-1),8,hit); }
  const auto start=std::chrono::steady_clock::now(); hits=0;
  for(int i=0;i<100000;++i) { const double a=(i%37-18)*.015,b=(i%29-14)*.015; hits+=map->raycast(Eigen::Vector3d(-14+(i%101)*.27,-8+(i%61)*.26,5),Eigen::Vector3d(a,b,-1),8,hit); }
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cout<<"BENCH triangles="<<map->triangleCount()<<" sizeof="<<sizeof(TerrainMap)<<" storage_bytes="<<map->storageBytes()<<" hits="<<hits<<" rays_per_second="<<(100000/seconds)<<" avg_us="<<(seconds*1e6/100000)<<"\\n";
  return 0;
}

static int terrainDepthDiagnostic(const char* depth_path,const char* mesh_path) {
  using json=nlohmann::json; std::ifstream input(depth_path); if(!input)throw std::runtime_error("cannot read depth diagnostic");
  std::string text((std::istreambuf_iterator<char>(input)),std::istreambuf_iterator<char>()); size_t at=0;
  while((at=text.find("Infinity",at))!=std::string::npos){text.replace(at,8,"null");at+=4;}
  const auto root=json::parse(text);
  const auto map=TerrainMap::load(mesh_path); const double fov=kPi/4.0;
  const double yaws[3]={0,120*kPi/180,-120*kPi/180}; std::vector<double> all;
  for(int vehicle=0;vehicle<4;++vehicle) {
    const auto& pose=root.at("poses").at(std::to_string(vehicle)); const auto& xyz=pose.at("xyz"); const auto& q=pose.at("q_wxyz");
    const double qw=q[0].get<double>(),qx=q[1].get<double>(),qy=q[2].get<double>(),qz=q[3].get<double>();
    const Eigen::Quaterniond attitude(qw,qx,qy,qz);
    const Eigen::Vector3d position(xyz[0],xyz[1],xyz[2]);
    for(int sensor=0;sensor<3;++sensor) {
      std::vector<double> errors; int valid=0,hits=0;
      const auto& values=root.at("depth").at(std::to_string(vehicle)+"_"+std::to_string(sensor));
      const Eigen::Matrix3d mount=(Eigen::AngleAxisd(yaws[sensor],Eigen::Vector3d::UnitZ())).toRotationMatrix();
      const Eigen::Vector3d origin=position+attitude.toRotationMatrix()*Eigen::Vector3d(0,0,0.055);
      for(int cell=0;cell<64;++cell) {
        if(!values[cell].is_number())continue; const double actual=values[cell].get<double>(); if(!std::isfinite(actual))continue; ++valid;
        const int row=cell/8,col=cell%8; const double u=(col-3.5)/4*std::tan(fov/2),v=(row-3.5)/4*std::tan(fov/2);
        const Eigen::Vector3d ray=attitude.toRotationMatrix()*mount*Eigen::Vector3d(1,-u,-v); TerrainMap::Hit hit;
        if(map->raycast(origin,ray,8.0,hit)){++hits;errors.push_back(hit.depth_m-actual);all.push_back(hit.depth_m-actual);}
      }
      std::sort(errors.begin(),errors.end()); double mae=0,bias=0; for(double e:errors){mae+=std::abs(e);bias+=e;}
      const auto pct=[&](double p){return errors.empty()?0.0:errors[std::min(errors.size()-1,static_cast<size_t>(p*(errors.size()-1)))];};
      if(!errors.empty()){mae/=errors.size();bias/=errors.size();}
      std::cout<<"DEPTH vehicle="<<vehicle<<" sensor="<<sensor<<" valid="<<valid<<" hits="<<hits<<" bias_m="<<bias<<" mae_m="<<mae<<" p50_m="<<pct(.50)<<" p90_m="<<pct(.90)<<" p95_m="<<pct(.95)<<"\\n";
    }
  }
  std::sort(all.begin(),all.end()); double mae=0;for(double e:all)mae+=std::abs(e);if(!all.empty())mae/=all.size();
  std::cout<<"DEPTH aggregate samples="<<all.size()<<" mae_m="<<mae<<" p50_m="<<(all.empty()?0:all[all.size()/2])<<" p90_m="<<(all.empty()?0:all[static_cast<size_t>(.9*(all.size()-1))])<<"\\n";
  return 0;
}

int main(int argc,char**argv){if(argc==2)return terrainBenchmark(argv[1]);if(argc==4&&std::string(argv[1])=="--depth-diagnostic")return terrainDepthDiagnostic(argv[2],argv[3]);heightAndPrediction();peerGate();sensorAndVisionGates();mavlinkCoordinateRoundTrip();terrainSurfaceTests();terrainTraversalBudgetTests();terrainEstimatorRegression();tofExposureAttitude();std::cout<<"PASS companion tests\n";}
