#include "companion/estimator.hpp"
#include "companion/terrain.hpp"
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace companion;
using json = nlohmann::json;
constexpr double kStep = .01;
constexpr int kSteps = 2000;
constexpr double kStart[] = {-1.4, 1.4};

struct Truth { Eigen::Vector3d position, acceleration; double yaw; };
struct Input {
  uint64_t stamp;
  std::array<Truth,4> truth;
  std::array<ImuSample,4> imu;
  std::array<std::array<TofFrame,3>,4> tof;
  std::array<std::array<UwbPeerSample,4>,4> uwb;
};
struct Variant { std::string id; bool uwb, acceleration; int sensors; };

Eigen::Matrix3d rotate(double yaw) {
  return Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
Truth trajectory(int id,double t) {
  const double x=kStart[id%2], y=kStart[id/2];
  const double sx=x<0?1:-1,sy=y<0?1:-1;
  return {{x+sx*.35*(1-std::cos(.45*t)),y+sy*.30*(1-std::cos(.6*t)),1.2},
          {sx*.35*.45*.45*std::cos(.45*t),sy*.30*.6*.6*std::cos(.6*t),0},
          std::atan2(y,x)+.18*std::sin(.4*t)};
}
std::shared_ptr<const TerrainMap> terrain() {
  std::vector<TerrainMap::Triangle> triangles;
  auto quad=[&](Eigen::Vector3f a,Eigen::Vector3f b,Eigen::Vector3f c,Eigen::Vector3f d) {
    triangles.push_back({a,b,c});triangles.push_back({a,c,d});
  };
  quad({-3,-3,0},{-3,3,0},{-3,3,4},{-3,-3,4});
  quad({3,-3,0},{3,3,0},{3,3,4},{3,-3,4});
  quad({-3,-3,0},{3,-3,0},{3,-3,4},{-3,-3,4});
  quad({-3,3,0},{3,3,0},{3,3,4},{-3,3,4});
  quad({-3,-3,0},{3,-3,0},{3,3,0},{-3,3,0});
  return std::make_shared<const TerrainMap>(std::move(triangles));
}
EstimatorConfig config(int id,const std::shared_ptr<const TerrainMap>& map) {
  EstimatorConfig c;c.vehicle_id=id;auto initial=trajectory(id,0);
  c.initial_pose.x_m=initial.position.x();c.initial_pose.y_m=initial.position.y();
  c.initial_pose.z_m=initial.position.z();c.initial_pose.yaw_rad=initial.yaw;
  c.initial_covariance*=.02;c.terrain=map;c.use_baro_fallback=false;
  for(int s=0;s<3;++s) {
    auto&m=c.tof_mounts[s];m.yaw_rad=s*2*kPi/3;
    m.translation_body_m=rotate(m.yaw_rad)*Eigen::Vector3d(.055,0,0);
    m.horizontal_fov_rad=m.vertical_fov_rad=kPi/4;m.max_range_m=4;
  }
  return c;
}
// Independent plane intersections provide the observation oracle.
double planeDepth(const Eigen::Vector3d& p,const Eigen::Vector3d& ray,int& axis) {
  double best=4.0;axis=-1;
  for(int a=0;a<3;++a)for(double face: {-3.0,3.0}) {
    if(a==2&&face==3.0)continue;
    double location=a==2?0:face;
    if(std::abs(ray(a))<1e-10)continue;
    double d=(location-p(a))/ray(a);Eigen::Vector3d hit=p+d*ray;
    if(d<.05||d>=best||hit.z() < -1e-6||hit.z()>4)continue;
    if(a!=0&&std::abs(hit.x())>3.000001)continue;
    if(a!=1&&std::abs(hit.y())>3.000001)continue;
    best=d;axis=a;
  }
  return best;
}
std::vector<Input> inputs(int seed,const std::string& scenario,const std::shared_ptr<const TerrainMap>& map) {
  std::mt19937 rng(seed);std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform;
  std::vector<Input> result;result.reserve(kSteps+1);
  std::array<EstimatorConfig,4> configs{config(0,map),config(1,map),config(2,map),config(3,map)};
  for(int tick=0;tick<=kSteps;++tick) {
    Input f{};double t=tick*kStep;f.stamp=1000000+tick*10000;
    for(int i=0;i<4;++i)f.truth[i]=trajectory(i,t);
    for(int i=0;i<4;++i) {
      auto&m=f.imu[i];const auto&truth=f.truth[i];m.sim_time_us=f.stamp;
      m.attitude_valid=m.acceleration_valid=true;m.yaw_enu_rad=truth.yaw+.002*normal(rng);
      m.acceleration_enu_mps2=truth.acceleration+Eigen::Vector3d(.008+.008*normal(rng),-.006+.008*normal(rng),0);
      if(tick%5)continue;
      for(int s=0;s<3;++s) {
        const auto&mount=configs[i].tof_mounts[s];auto&frame=f.tof[i][s];
        frame.sim_time_us=f.stamp;frame.sensor_index=s;
        Eigen::Vector3d origin=truth.position+rotate(truth.yaw)*mount.translation_body_m;
        double common=.006*normal(rng);
        for(int cell=0;cell<64;++cell) {
          int row=cell/8,col=cell%8;
          Eigen::Vector3d ray=rotate(truth.yaw+mount.yaw_rad)*Eigen::Vector3d(1,-(col-3.5)/4*std::tan(kPi/8),-(row-3.5)/4*std::tan(kPi/8));
          int axis;double d=planeDepth(origin,ray,axis);
          frame.distance_m[cell]=d+common+.012*normal(rng);
          bool live=axis>=0&&uniform(rng)>.01;
          if(t>=8&&t<16) {
            if(scenario=="one_vehicle_single_axis"&&i==0&&axis==1)live=false;
            if(scenario=="all_tof_blackout")live=false;
          }
          frame.valid[cell]=live;
        }
      }
      for(int j=0;j<4;++j)if(i!=j) {
        Eigen::Vector3d delta=f.truth[j].position-truth.position;
        Eigen::Vector3d body=rotate(-truth.yaw)*delta;
        f.uwb[i][j]={f.stamp,j,delta.norm()+.15*normal(rng),wrapAngle(std::atan2(body.y(),body.x())+.08*normal(rng)),uniform(rng)>.02};
      }
    }
    result.push_back(std::move(f));
  }
  return result;
}
json run(const std::vector<Input>& frames,const Variant& variant,const std::shared_ptr<const TerrainMap>& map) {
  std::array<std::unique_ptr<Estimator>,4> estimators;
  for(int i=0;i<4;++i)estimators[i]=std::make_unique<Estimator>(config(i,map));
  double sq=0,maximum=0,commonSq=0,relativeSq=0;int count=0,valid=0,peerValid=0;
  int uwbCalls=0,uwbAccepted=0,tofCalls=0,tofAccepted=0;
  std::array<double,4> vehicleSq{};std::array<int,4> vehicleValid{};
  double degradedSq=0;int degradedCount=0;
  const auto begin=std::chrono::steady_clock::now();
  for(size_t k=0;k<frames.size();++k) {
    const auto&f=frames[k];
    for(int i=0;i<4;++i) {
      auto imu=f.imu[i];if(!variant.acceleration)imu.acceleration_valid=false;
      estimators[i]->predict(imu);
      estimators[i]->fuseHeightAssist({f.stamp,f.truth[i].position.z(),"SIM_HEIGHT_ASSIST"});
      if(k%5==0)for(int s=0;s<variant.sensors;++s) {
        ++tofCalls;tofAccepted+=estimators[i]->fuseTof(f.tof[i][s]).accepted;
      }
    }
    // Freeze all peer publications before processing this timestamp's links.
    std::array<PeerEstimate,4> peers;
    for(int i=0;i<4;++i) {
      auto e=estimators[i]->estimate();auto&p=peers[i];
      p.sim_time_us=e.sim_time_us;p.vehicle_id=i;p.pose=e.pose;
      p.velocity_enu_mps=e.velocity_enu_mps;p.pose_covariance=e.covariance.topLeftCorner<3,3>();
      p.xy_structural_valid=e.xy_structural_valid;p.xy_peer_valid=e.xy_peer_valid;
      p.height_assisted=e.height_assisted;p.source="ESTIMATE";
    }
    if(variant.uwb&&k%5==0)for(int i=0;i<4;++i)for(int j=0;j<4;++j)if(i!=j) {
      ++uwbCalls;uwbAccepted+=estimators[i]->fuseUwb(f.uwb[i][j],peers[j]).accepted;
    }
    if(k<300)continue;
    Eigen::Vector2d common=Eigen::Vector2d::Zero();std::array<Eigen::Vector2d,4> positions;
    for(int i=0;i<4;++i) {
      auto e=estimators[i]->estimate();positions[i]={e.pose.x_m,e.pose.y_m};
      Eigen::Vector2d error=positions[i]-f.truth[i].position.head<2>();
      if(!error.allFinite())throw std::runtime_error("nonfinite estimate");
      double squared=error.squaredNorm();sq+=squared;vehicleSq[i]+=squared;
      valid+=e.xy_valid;vehicleValid[i]+=e.xy_valid;peerValid+=e.xy_peer_valid;
      maximum=std::max(maximum,std::sqrt(squared));++count;common+=error/4;
      if(i==0&&k>=800&&k<1600){degradedSq+=squared;++degradedCount;}
    }
    commonSq+=common.squaredNorm();
    for(int i=0;i<4;++i)for(int j=i+1;j<4;++j) {
      double error=(positions[i]-positions[j]).norm()-(f.truth[i].position-f.truth[j].position).head<2>().norm();
      relativeSq+=error*error;
    }
  }
  json vehicles=json::array();for(int i=0;i<4;++i)vehicles.push_back({{"vehicle",i},{"xy_rmse_m",std::sqrt(vehicleSq[i]/(count/4))},{"valid_fraction",double(vehicleValid[i])/(count/4)}});
  return {{"variant",variant.id},{"xy_rmse_m",std::sqrt(sq/count)},{"max_xy_error_m",maximum},
          {"valid_fraction",double(valid)/count},{"peer_valid_fraction",double(peerValid)/count},
          {"vehicle0_window_rmse_m",std::sqrt(degradedSq/degradedCount)},
          {"common_drift_rmse_m",std::sqrt(commonSq/(count/4))},
          {"pair_distance_rmse_m",std::sqrt(relativeSq/(6*(count/4)))},
          {"samples",count},{"tof_calls",tofCalls},{"tof_accepted",tofAccepted},
          {"uwb_calls",uwbCalls},{"uwb_accepted",uwbAccepted},{"vehicles",vehicles},
          {"elapsed_seconds",std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()}};
}
int main(int argc,char**argv) {
  try {
    if(argc!=2)throw std::runtime_error("usage: algorithm_ablation OUTPUT.json");
    const auto map=terrain();json rows=json::array();
    std::vector<Variant> variants{{"full",true,true,3},{"no_uwb",false,true,3},{"no_tof",true,true,0},{"no_acceleration",true,false,3},{"one_tof",true,true,1},{"two_tof",true,true,2}};
    for(const std::string scenario:{"all_structure","one_vehicle_single_axis","all_tof_blackout"})for(int seed:{2027,4099,6151}) {
      auto frames=inputs(seed,scenario,map);
      for(const auto&v:variants) {
        auto row=run(frames,v,map);row["seed"]=seed;row["scenario"]=scenario;
        rows.push_back(row);std::cout<<scenario<<" "<<seed<<" "<<v.id<<" rmse="<<row["xy_rmse_m"]<<std::endl;
      }
    }
    json output{{"schema",1},{"input_model","analytic finite-wall sensor fixtures"},{"duration_s",kSteps*kStep},{"evaluation_start_s",3},{"degraded_window_s",{8,16}},{"height_assist",true},{"imu_hz",100},{"tof_uwb_hz",20},{"runs",rows}};
    std::ofstream file(argv[1]);if(!file)throw std::runtime_error("cannot open output");file<<output.dump(2)<<std::endl;
    return file.good()?0:2;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}
}
