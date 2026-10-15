/**
 * @file runner.cpp
 * @brief 单架伴随程序入口：接收传感器 → 定位融合 → 生成控制 → MAVLink 发送。
 * 每架启动一个进程，--vehicle 指定机号。Estimator 管本机估计，peer_cache 管邻机估计。
 * observation(16000+机号) 收测量，peers(17000+机号) 收邻机位置估计，
 * team(18000+机号) 收任务命令；18100 是协调器的状态接收端口。
 * 初读先按 main 的编号步骤看主流程，再读 configure、odometry 和 estimator.cpp。
 */
#include "companion/estimator.hpp"
#include "companion/protocol.hpp"
#include "companion/vision.hpp"
#include "companion/armor_tracker.hpp"
#include "companion/rail_motion.hpp"
#include "companion/intercept.hpp"
#include "companion/periodic_motion_worker.hpp"
#include "companion/vision_worker.hpp"
#include "companion/latest_tof.hpp"
#include "companion/vision_delay.hpp"
#include "companion/flight_feedback.hpp"
#include "companion/imu_attitude.hpp"
#ifdef COMPANION_WITH_OPENCV
#include "companion/rm_auto_aim.hpp"
#endif
#include <deque>
#include "companion/terrain.hpp"
#include <common/mavlink.h>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <thread>
#include <stdexcept>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

namespace {
using json=nlohmann::json;
using Clock=std::chrono::steady_clock;
volatile std::sig_atomic_t running=1;
void stop(int){running=0;}
double age(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
json readJson(const std::filesystem::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("cannot read "+p.string());json j;f>>j;return j;}
/**
 * @brief 非阻塞回环 UDP，当前进程拥有套接字，析构负责关闭。
 * @note receive 空串表示无包或接收失败；不等待下一包，避免阻塞其他数据源。
 * send 失败累加 send_drops 并保存 errno，主循环继续，停止日志提供诊断。
 */
struct Udp {
  int fd{-1};
  uint64_t send_drops{0};
  int last_send_errno{0};
  explicit Udp(int port){fd=socket(AF_INET,SOCK_DGRAM,0);if(fd<0)throw std::runtime_error("socket failed");sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(port);if(bind(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)))throw std::runtime_error("loopback port bind failed: "+std::to_string(port));if(fcntl(fd,F_SETFL,O_NONBLOCK)<0)throw std::runtime_error("nonblocking failed");}
  ~Udp(){if(fd>=0)close(fd);}
  std::string receive(){std::array<char,65536>b{};sockaddr_in a{};socklen_t n=sizeof(a);auto count=recvfrom(fd,b.data(),b.size(),0,reinterpret_cast<sockaddr*>(&a),&n);if(count<0)return {};if(a.sin_addr.s_addr!=htonl(INADDR_LOOPBACK))return {};return {b.data(),static_cast<size_t>(count)};}
  bool send(const void*data,size_t size,int port){sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(port);if(sendto(fd,data,size,0,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){last_send_errno=errno;++send_drops;return false;}return true;}
  bool send(const std::string& s,int port){return send(s.data(),s.size(),port);}
};
/** @brief 命令行配置；duration 为现实经过秒数，0 表示等停止信号。 */
struct Options {
  int vehicle=0,local=0,remote=0;
  double duration=0,takeoff_height=1.8;
  bool offboard=false,arm=false;
  int mission_vehicle=0;
  std::filesystem::path config="simulation/config.json",map="simulation/maps/calibration.json",runtime="simulation/runtime";
};
Options options(int argc,char**argv){Options o;for(int i=1;i<argc;++i){std::string a=argv[i];if(a=="--help"){std::cout<<"--vehicle N --config JSON --map JSON --runtime DIR --duration SECONDS [--offboard] [--arm] [--takeoff-height M] [--mavlink-local-port N] [--mavlink-remote-port N]\n";std::exit(0);}if(a=="--offboard"){o.offboard=true;continue;}if(a=="--arm"){o.arm=true;continue;}if(++i>=argc)throw std::runtime_error("missing value for "+a);std::string v=argv[i];if(a=="--vehicle")o.vehicle=std::stoi(v);else if(a=="--config")o.config=v;else if(a=="--map")o.map=v;else if(a=="--runtime")o.runtime=v;else if(a=="--duration")o.duration=std::stod(v);else if(a=="--takeoff-height")o.takeoff_height=std::stod(v);else if(a=="--mavlink-local-port")o.local=std::stoi(v);else if(a=="--mavlink-remote-port")o.remote=std::stoi(v);else throw std::runtime_error("unknown option "+a);}if(o.vehicle<0||o.vehicle>245||o.duration<0||o.takeoff_height<0.3||o.takeoff_height>3.0||o.arm&&!o.offboard)throw std::runtime_error("invalid CLI options");if(!o.local)o.local=14540+o.vehicle;if(!o.remote)o.remote=14580+o.vehicle;if(o.local<1024||o.local>65535||o.remote<1024||o.remote>65535)throw std::runtime_error("invalid UDP port");return o;}
/**
 * @brief 合并地图与传感器配置，建立本机的初始估计和安装关系。
 * @param o 输入路径和机号，spawns 必须包含该机。
 * @param camera 输出相机内参；定位配置通过返回值交给 Estimator。
 * @note 覆盖顺序为默认值、运行目录传感器契约、配置文件 companion 参数。
 * 初始位姿来自已知起点；三维地形只提供静态几何，不提供飞行中的真实位置。
 */
companion::EstimatorConfig configure(const Options&o,companion::CameraConfig&camera){
  const auto map=readJson(o.map);
  companion::EstimatorConfig c;c.vehicle_id=o.vehicle;c.initial_covariance.setIdentity();c.initial_covariance*=0.02;c.initial_covariance(3,3)=0.01;c.initial_covariance.block<3,3>(4,4)*=0.1;
  bool found=false;for(const auto&s:map.at("spawns"))if(s.at("id").get<int>()==o.vehicle){c.initial_pose.x_m=s.at("x");c.initial_pose.y_m=s.at("y");c.initial_pose.z_m=s.at("z");c.initial_pose.roll_rad=s.value("roll",0.0);c.initial_pose.pitch_rad=s.value("pitch",0.0);c.initial_pose.yaw_rad=s.value("yaw",0.0);found=true;}if(!found)throw std::runtime_error("vehicle spawn missing");
  for(const auto&w:map.at("walls")){companion::MapLineSegment l;l.start_m=Eigen::Vector2d(w.at("x1").get<double>(),w.at("y1").get<double>());l.end_m=Eigen::Vector2d(w.at("x2").get<double>(),w.at("y2").get<double>());l.z_min_m=w.at("z_min");l.z_max_m=w.at("z_max");c.map_lines.push_back(l);}c.floor_z_m=map.value("room",json::object()).value("floor_z",0.0);
  if(map.contains("uwb_anchors"))for(const auto&anchor:map.at("uwb_anchors")){
    const int id=anchor.at("id").get<int>();
    Eigen::Vector3d point(anchor.at("x").get<double>(),anchor.at("y").get<double>(),anchor.at("z").get<double>());
    if(id<0||!point.allFinite()||c.uwb_anchors.count(id))throw std::runtime_error("invalid UWB anchor configuration");
    c.uwb_anchors.emplace(id,point);
  }
  if(map.contains("terrain_triangles_file")) { c.terrain=companion::TerrainMap::load(o.map.parent_path()/map.at("terrain_triangles_file").get<std::string>()); }
  if(map.contains("safe_waypoints"))for(const auto&w:map.at("safe_waypoints")){companion::Pose2 p;p.x_m=w.at("x");p.y_m=w.at("y");p.yaw_rad=w.value("yaw",0.0);c.safe_waypoints.push_back(p);}
  if(map.contains("vehicle_waypoints")){
    auto key=std::to_string(o.vehicle);
    if(map.at("vehicle_waypoints").contains(key)){
      c.safe_waypoints.clear();
      for(const auto&w:map.at("vehicle_waypoints").at(key)){companion::Pose2 p;p.x_m=w.at("x");p.y_m=w.at("y");p.yaw_rad=w.value("yaw",0.0);c.safe_waypoints.push_back(p);}
    }
  }
  for(int s=0;s<3;++s){c.tof_mounts[s].yaw_rad=(s==1?120:s==2?-120:0)*companion::kPi/180;c.tof_mounts[s].pitch_down_rad=20*companion::kPi/180;c.tof_mounts[s].horizontal_fov_rad=c.tof_mounts[s].vertical_fov_rad=companion::kPi/4;c.tof_mounts[s].translation_body_m=Eigen::Vector3d(s==0?.4:-.2,s==0?0.0:(s==1?.3464101615:-.3464101615),.08);}
  camera.width=640;camera.height=360;camera.fx_px=camera.fy_px=320;camera.cx_px=319.5;camera.cy_px=179.5;
  const auto active_contract=o.runtime/"companion_config.json";
  if(std::filesystem::exists(active_contract)){
    const auto contract=readJson(active_contract);
    const auto sensor=contract.contains("sensor_contract")?contract.at("sensor_contract"):contract;
    if(sensor.contains("tof_hfov_rad"))for(int s=0;s<3;++s)c.tof_mounts[s].horizontal_fov_rad=sensor.at("tof_hfov_rad");
    if(sensor.contains("tof_vfov_rad"))for(int s=0;s<3;++s)c.tof_mounts[s].vertical_fov_rad=sensor.at("tof_vfov_rad");
    if(sensor.contains("tof_yaw_rad"))for(int s=0;s<3;++s)c.tof_mounts[s].yaw_rad=sensor.at("tof_yaw_rad").at(s);
    if(sensor.contains("tof_pitch_down_rad"))for(int s=0;s<3;++s)c.tof_mounts[s].pitch_down_rad=sensor.at("tof_pitch_down_rad");
    if(sensor.contains("tof_origins_body_m"))for(int s=0;s<3;++s)for(int a=0;a<3;++a)c.tof_mounts[s].translation_body_m(a)=sensor.at("tof_origins_body_m").at(s).at(a);
    if(sensor.contains("camera_width"))camera.width=sensor.at("camera_width");
    if(sensor.contains("camera_height"))camera.height=sensor.at("camera_height");
    if(sensor.contains("camera_hfov_rad"))camera.fx_px=camera.fy_px=0.5*camera.width/std::tan(sensor.at("camera_hfov_rad").get<double>()/2.0);
    camera.cx_px=(camera.width-1)*.5;camera.cy_px=(camera.height-1)*.5;
  }
  if(std::filesystem::exists(o.config)){auto root=readJson(o.config);auto j=root.value("companion",json::object());if(j.contains("tof_mounts")){const auto& mounts=j.at("tof_mounts");if(mounts.size()!=3)throw std::runtime_error("tof_mounts must have 3 entries");for(int s=0;s<3;++s){auto&m=c.tof_mounts[s];const auto&v=mounts[s];m.yaw_rad=v.value("yaw_rad",m.yaw_rad);m.roll_rad=v.value("roll_rad",0.0);m.pitch_down_rad=v.value("pitch_down_rad",m.pitch_down_rad);m.horizontal_fov_rad=v.value("horizontal_fov_rad",m.horizontal_fov_rad);m.vertical_fov_rad=v.value("vertical_fov_rad",m.vertical_fov_rad);m.min_range_m=v.value("min_range_m",m.min_range_m);m.max_range_m=v.value("max_range_m",m.max_range_m);if(v.contains("translation_body_m"))for(int a=0;a<3;++a)m.translation_body_m(a)=v.at("translation_body_m").at(a);}}auto cam=j.value("camera",json::object());camera.fx_px=cam.value("fx_px",camera.fx_px);camera.fy_px=cam.value("fy_px",camera.fy_px);camera.cx_px=cam.value("cx_px",camera.cx_px);camera.cy_px=cam.value("cy_px",camera.cy_px);camera.strip_center_separation_m=cam.value("strip_center_separation_m",0.135);}
  return c;
}
// 列向量先绕机体 X 滚转，再绕 Y 俯仰，最后绕世界 Z 偏航；角度均为 rad。
Eigen::Matrix3d rotation(double r,double p,double y){return (Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX())).toRotationMatrix();}
bool sendMav(Udp&socket,mavlink_message_t&msg,int port){uint8_t buf[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(buf,&msg);return socket.send(buf,n,port);}
void command(Udp&sock,int port,int sys,int comp,int id,float p1,float p2,float p3=0){mavlink_message_t msg;mavlink_msg_command_long_pack(200,191,&msg,sys,comp,id,0,p1,p2,p3,0,0,0,0);sendMav(sock,msg,port);}
/**
 * @brief 把定位估计转为 MAVLink ODOMETRY，供飞控外部定位融合。
 * @param sock MAVLink 输出套接字。
 * @param port 当前 PX4 接收端口。
 * @param e 世界 ENU 位姿和速度：位置 m、速度 m/s、角度 rad、时间 μs。
 * @param reliable_xy 最后可靠的世界系水平位置；水平定位有效时更新，失效时沿用。
 * @note a 交换世界 X/Y 并反转 Z；b 把机体前左上 FLU 转成前右下 FRD。
 * 姿态、速度与协方差都必须同样换坐标，不能只修改位置正负号。
 */
void odometry(Udp&sock,int port,const companion::Estimate&e,Eigen::Vector2d& reliable_xy){
  Eigen::Matrix3d a;a<<0,1,0,1,0,0,0,0,-1;Eigen::Matrix3d b=Eigen::Vector3d(1,-1,-1).asDiagonal();
  Eigen::Matrix3d r=rotation(e.pose.roll_rad,e.pose.pitch_rad,e.pose.yaw_rad);
  const Eigen::Matrix3d frd_rotation=(a*r*b).eval();
  const Eigen::Quaterniond q(frd_rotation);
  if(e.xy_valid)reliable_xy={e.pose.x_m,e.pose.y_m};
  // 接收端要求位置三轴均有限才能保留高度；水平失效时沿用最后可靠位置并放大方差。
  const Eigen::Vector3d p=(a*Eigen::Vector3d(reliable_xy.x(),reliable_xy.y(),e.pose.z_m)).eval();
  const Eigen::Vector3d v=(b*r.transpose()*e.velocity_enu_mps).eval();
  mavlink_odometry_t d{};d.time_usec=e.sim_time_us;d.frame_id=MAV_FRAME_LOCAL_FRD;d.child_frame_id=MAV_FRAME_BODY_FRD;d.x=p.x();d.y=p.y();d.z=p.z();d.q[0]=q.w();d.q[1]=q.x();d.q[2]=q.y();d.q[3]=q.z();d.vx=v.x();d.vy=v.y();d.vz=v.z();d.rollspeed=d.pitchspeed=d.yawspeed=0;d.estimator_type=MAV_ESTIMATOR_TYPE_VISION;d.reset_counter=0;d.quality=e.z_valid?(e.xy_valid?100:80):0;
  const bool velocity_valid=(e.xy_valid||e.flow_valid)&&e.velocity_enu_mps.allFinite();
  if(!velocity_valid)d.vx=d.vy=d.vz=std::numeric_limits<float>::quiet_NaN();
  Eigen::Matrix<double,6,6>pc=Eigen::Matrix<double,6,6>::Zero(),vc=pc;
  Eigen::Matrix3d velocity_covariance=e.covariance.block<3,3>(4,4);
  pc.block<3,3>(0,0)=a*e.covariance.block<3,3>(0,0)*a.transpose();
  // 水平定位无效时放大 XY 方差，降低飞控融合权重；高度方差仍正常保留。
  if(!e.xy_valid)for(int axis=0;axis<2;++axis){
    pc.row(axis).setZero();pc.col(axis).setZero();pc(axis,axis)=10000.0;
  }
  // 没有水平定位或新鲜光流时停止整组速度融合，避免惯性漂移通过机体系旋转污染高度。
  if(!velocity_valid)for(int axis=0;axis<2;++axis){
    velocity_covariance.row(axis).setZero();velocity_covariance.col(axis).setZero();velocity_covariance(axis,axis)=10000.0;
  }
  pc(3,3)=pc(4,4)=0.02;pc(5,5)=std::max(0.01,e.covariance(3,3));
  vc.block<3,3>(0,0)=b*r.transpose()*velocity_covariance*r*b;vc(3,3)=vc(4,4)=vc(5,5)=0.1;
  // 对称 6×6 协方差按行取上三角，打包成 MAVLink 规定的 21 个数。
  int k=0;for(int row=0;row<6;++row)for(int col=row;col<6;++col){d.pose_covariance[k]=pc(row,col);d.velocity_covariance[k]=vc(row,col);++k;}
  mavlink_message_t msg;mavlink_msg_odometry_encode(200,191,&msg,&d);sendMav(sock,msg,port);
}
}

/** @brief 单机主循环；正常结束返回 0，配置或运行异常返回 2。 */
int main(int argc,char**argv){try{
  // 1. 初始化本机配置与滤波状态；机群成员列表从地图读取。
  auto o=options(argc,argv);companion::CameraConfig camera;auto config=configure(o,camera);companion::Estimator estimator(config);
  if(std::filesystem::exists(o.config))o.mission_vehicle=readJson(o.config).value("mission",json::object()).value("explorer_vehicle",0);

  const auto mission=std::filesystem::exists(o.config)?readJson(o.config).value("mission",json::object()):json::object();
  const std::string mission_mode=mission.value("mode","localization");
  if(mission_mode!="localization"&&mission_mode!="armor"&&mission_mode!="team_contact")throw std::runtime_error("unknown mission.mode");
  const bool armor_enabled=mission_mode=="armor";
  const bool team_enabled=mission_mode=="team_contact";
  const auto map_settings=readJson(o.map);
  std::vector<int> vehicle_ids;for(const auto&spawn:map_settings.at("spawns"))vehicle_ids.push_back(spawn.at("id").get<int>());
  if(armor_enabled&&std::find(vehicle_ids.begin(),vehicle_ids.end(),o.mission_vehicle)==vehicle_ids.end())throw std::runtime_error("mission explorer is not in fleet");
  const auto team_settings=map_settings.value("team_mission",json::object());
  const bool visual_contact=team_enabled&&team_settings.value("visual_contact",false);
#ifndef COMPANION_WITH_OPENCV
  if(visual_contact)throw std::runtime_error("移动装甲视觉任务需要启用 COMPANION_WITH_OPENCV 构建");
#endif
  // 俯冲撞击参数：接触阶段不再用视觉伺服爬行，而是从高处按一条直线冲进装甲板。
  // 现场调参项一律在启动时报错，避免飞行中静默退化成默认值。
  const auto dive=team_settings.value("dive_attack",json::object());
  const bool dive_enabled=visual_contact&&dive.value("enabled",false);
  const double dive_impact_speed=dive.value("impact_speed_mps",3.0);
  const double dive_min_forward=dive.value("min_forward_speed_mps",1.2);
  const double dive_commit_range=dive.value("commit_range_m",0.9);
  const bool dive_plane_retreat=dive.value("passed_plane_retreat_enabled",false);
  const double dive_exit_margin=dive.value("exit_margin_m",0.25);
  const double dive_descent_limit=dive.value("descent_limit_mps",1.5);
  const double dive_lateral_limit=dive.value("lateral_limit_mps",1.5);
  const double dive_predict_horizon=dive.value("predict_horizon_s",0.6);
  const double dive_predict_fraction=dive.value("predict_horizon_fraction",0.25);
  const double dive_predict_confidence=dive.value("predict_confidence",0.7);
  const double dive_aim_lateral_limit=dive.value("aim_lateral_limit_m",0.35);
  const double dive_arrival_acceleration=dive.value("arrival_acceleration_mps2",1.5);
  const bool dive_acceleration_arrival=dive.value("arrival_acceleration_enabled",false);
  const double dive_aim_correction_rate=dive.value("aim_correction_rate_mps",0.0);
  const double dive_linear_speed_limit=dive.value("linear_speed_limit_mps",1.0);
  const double dive_reflection_half_width=dive.value("linear_reflection_half_width_m",0.0);
  const bool dive_entry_alignment=dive.value("entry_alignment_enabled",false);
  const bool dive_robust_linear=dive.value("robust_linear_enabled",false);
  const double dive_alignment_horizon=dive.value("entry_alignment_horizon_s",0.75);
  const double dive_alignment_tolerance=dive.value("entry_alignment_tolerance_m",0.06);
  const double dive_alignment_speed=dive.value("entry_alignment_speed_tolerance_mps",0.2);
  Eigen::Vector3d dive_entry=Eigen::Vector3d::Zero();
  if(dive_enabled){
    const auto target=team_settings.at("target");
    dive_entry={target.at("x").get<double>()-dive.value("entry_standoff_m",1.35),
        target.at("y").get<double>(),target.at("z").get<double>()+dive.value("entry_height_m",.55)};
    if(team_settings.contains("dive_entry")){
      const auto& entry=team_settings.at("dive_entry");
      dive_entry={entry.at("x").get<double>(),entry.at("y").get<double>(),entry.value("z",dive_entry.z())};
    }
    if(!dive_entry.allFinite())throw std::runtime_error("invalid dive entry position");
  }
  const double dive_weak_retreat_s=dive.value("weak_retreat_s",1.2);
  const double dive_timeout_s=dive.value("dive_timeout_s",1.2);
  const int dive_weak_limit=dive.value("max_weak_contacts",4);
  // 周期学习所需的最短连续观测窗：滑轨全周期 5.6 s（±0.28 m、0.2 m/s，单程 2.8 s），
  // 少于两个完整周期就折不出重复结构。单机视觉窗最长 9–10 s，所以这个门限只能靠
  // 合入队友广播的样本来满足。
  const double dive_learn_span=dive.value("learn_span_s",12.0);
  for(const auto& item:std::initializer_list<std::pair<const char*,double>>{{"impact_speed_mps",dive_impact_speed},{"commit_range_m",dive_commit_range},{"descent_limit_mps",dive_descent_limit},{"lateral_limit_mps",dive_lateral_limit},{"predict_horizon_s",dive_predict_horizon},{"predict_horizon_fraction",dive_predict_fraction},{"dive_timeout_s",dive_timeout_s}})
    if(!std::isfinite(item.second)||item.second<=0)throw std::runtime_error(std::string("invalid team_mission.dive_attack.")+item.first);
  if(!std::isfinite(dive_learn_span)||dive_learn_span<2.0)throw std::runtime_error("invalid team_mission.dive_attack.learn_span_s");
  for(const auto& item:std::initializer_list<std::pair<const char*,double>>{{"min_forward_speed_mps",dive_min_forward},{"weak_retreat_s",dive_weak_retreat_s}})
    if(!std::isfinite(item.second)||item.second<0)throw std::runtime_error(std::string("invalid team_mission.dive_attack.")+item.first);
  if(!std::isfinite(dive_predict_confidence)||dive_predict_confidence<0||dive_predict_confidence>1)throw std::runtime_error("invalid team_mission.dive_attack.predict_confidence");
  if(!std::isfinite(dive_aim_lateral_limit)||dive_aim_lateral_limit<=0)throw std::runtime_error("invalid team_mission.dive_attack.aim_lateral_limit_m");
  if(!std::isfinite(dive_arrival_acceleration)||dive_arrival_acceleration<=0||
     !std::isfinite(dive_aim_correction_rate)||dive_aim_correction_rate<0)
    throw std::runtime_error("invalid dive arrival acceleration or aim correction rate");
  if(dive_weak_limit<0||dive_weak_limit>64)throw std::runtime_error("invalid team_mission.dive_attack.max_weak_contacts");
  if(!std::isfinite(dive_exit_margin)||dive_exit_margin<0||dive_exit_margin>5)
    throw std::runtime_error("invalid team_mission.dive_attack.exit_margin_m");
  for(const auto& value:{dive_linear_speed_limit,dive_alignment_horizon,dive_alignment_tolerance,dive_alignment_speed})
    if(!std::isfinite(value)||value<=0)throw std::runtime_error("invalid dive linear prediction or entry alignment");
  if(!std::isfinite(dive_reflection_half_width)||dive_reflection_half_width<0||dive_reflection_half_width>dive_aim_lateral_limit)
    throw std::runtime_error("invalid dive linear_reflection_half_width_m");
  if(dive_entry_alignment&&(dive_alignment_horizon>dive_timeout_s||dive_alignment_horizon>10))
    throw std::runtime_error("entry alignment horizon exceeds terminal timeout");

  companion::ArmorDetectionConfig detection; detection.enemy_color=visual_contact?companion::ArmorColor::Blue:companion::ArmorColor::AnyBright;
  detection.max_pair_aspect=team_settings.value("visual_max_pair_aspect",4.0);
  companion::VisualGuidanceConfig visual_config;
  visual_config.max_forward_speed_mps=team_settings.value("contact_speed_mps",.18);
  visual_config.forward_gain=3.0;visual_config.lateral_gain=2.0;visual_config.vertical_gain=2.0;
  visual_config.yaw_gain=0.0;visual_config.max_observation_age_s=.25;visual_config.max_prediction_s=.3;
  companion::VisualTargetTracker visual_tracker(visual_config);
  companion::VisualGuidanceResult visual_result;
  bool visual_ready=false;
  uint64_t visual_lost_since=0;
  companion::ArmorTracker armor_track(team_settings.value("armor_process_noise_density",1.0),
      team_settings.value("armor_initial_velocity_variance",1.0));
  const double visual_pose_variance=team_settings.value("visual_pose_variance_m2",.0009);
  if(!std::isfinite(visual_pose_variance)||visual_pose_variance<=0)
    throw std::runtime_error("visual_pose_variance_m2 must be finite and positive");
  // 视觉跟踪结果在 COMPANION_WITH_OPENCV 块里产生，俯冲控制在外面使用，因此单独留一份。
  companion::ArmorTrack dive_track;
  companion::RailMotionFit rail_motion_fit(dive_linear_speed_limit);
  companion::PeriodicMotionWorker periodic_motion;
  companion::VisionWorker vision_worker;
  companion::LatestTof tof_queue;
  uint64_t requested_frame_seq=0;
  if(visual_contact&&team_settings.value("green_guide",json::object()).value("enabled",false)){
    const auto guide_config=team_settings.at("green_guide");
    for(const char* key:{"center_world_m","to_armor_world_m"}){
      const auto vector=guide_config.at(key).get<std::vector<double>>();
      if(vector.size()!=3||!std::all_of(vector.begin(),vector.end(),[](double v){return std::isfinite(v);}))
        throw std::runtime_error("green guide requires finite XYZ vectors");
    }
  }
  if(dive_enabled){periodic_motion.setHorizonCap(dive_predict_horizon);periodic_motion.setHorizonFraction(dive_predict_fraction);periodic_motion.setMinSpan(dive_learn_span);}
  std::deque<companion::Estimate> pose_history;
  uint64_t visual_pose_stamp=0;
  struct DelayedPose {uint64_t stamp;Eigen::Vector3d world;bool valid;double error;bool green{false};Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};};
  const double vision_delay_ms=team_settings.value("vision_delay_ms",0.0);
  if(!std::isfinite(vision_delay_ms)||vision_delay_ms<0||vision_delay_ms>2000)throw std::runtime_error("vision_delay_ms must be 0..2000");
  companion::VisionDelay<DelayedPose> vision_queue(static_cast<uint64_t>(vision_delay_ms*1000));


  for(const char* key:{"position_gain","cruise_speed_mps","climb_speed_mps","descent_speed_mps","landing_speed_mps","touchdown_descent_mps","yaw_gain","yaw_rate_limit_rad_s","impact_acceleration_mps2","impact_proximity_m"})
    if(team_settings.contains(key)&&(!team_settings[key].is_number()||!std::isfinite(team_settings[key].get<double>())||team_settings[key].get<double>()<=0))throw std::runtime_error(std::string("invalid team_mission.")+key);
  const double collision_force=team_settings.value("collision_specific_force_mps2",20.0);
  const double collision_proximity=team_settings.value("collision_proximity_m",.75);
  const double collision_grace_s=team_settings.value("collision_confirmation_grace_s",.06);
  if(!std::isfinite(collision_force)||collision_force<=9.80665||!std::isfinite(collision_proximity)||collision_proximity<=0)
    throw std::runtime_error("invalid collision force or proximity");
  if(!std::isfinite(collision_grace_s)||collision_grace_s<0||collision_grace_s>.2)
    throw std::runtime_error("collision confirmation grace must be 0..0.2 s");
  for(const char* key:{"velocity_damping","impact_settle_s","impact_recovery_s","impact_pitch_rad","impact_roll_rad","touchdown_thrust"})
    if(team_settings.contains(key)&&(!team_settings[key].is_number()||!std::isfinite(team_settings[key].get<double>())||team_settings[key].get<double>()<0))throw std::runtime_error(std::string("invalid team_mission.")+key);
  // 视觉撞击的现场调参项：必须在启动时报错而不是在飞行中静默退化成默认值。
  for(const char* key:{"intercept_horizon_s","visual_stale_age_s"})
    if(team_settings.contains(key)&&(!team_settings[key].is_number()||!std::isfinite(team_settings[key].get<double>())||team_settings[key].get<double>()<=0))throw std::runtime_error(std::string("invalid team_mission.")+key);
  for(const char* key:{"visual_approach_speed_mps","visual_no_reverse_range_m","visual_fresh_age_s","visual_ready_age_s","visual_lost_push_mps","visual_lost_coast_s","visual_search_speed_mps"})
    if(team_settings.contains(key)&&(!team_settings[key].is_number()||!std::isfinite(team_settings[key].get<double>())||team_settings[key].get<double>()<0))throw std::runtime_error(std::string("invalid team_mission.")+key);
  if(!std::isfinite(team_settings.value("visual_min_forward_scale",.3))||team_settings.value("visual_min_forward_scale",.3)<0||team_settings.value("visual_min_forward_scale",.3)>1)throw std::runtime_error("visual_min_forward_scale must be 0..1");
  if(team_settings.value("impact_pitch_rad",0.0)>=companion::kPi/2||team_settings.value("impact_roll_rad",0.0)>=companion::kPi/2||team_settings.value("touchdown_thrust",0.0)>1)throw std::runtime_error("invalid recovery attitude or thrust");
  // 持续时间过滤碰撞时短暂的着地标志，避免把接触后的反弹判为无法继续任务。
  const double stranded_s=team_settings.value("stranded_s",1.0);
  if(!std::isfinite(stranded_s)||stranded_s<.2||stranded_s>10.0)throw std::runtime_error("invalid team_mission.stranded_s");
  // 「该走却不动」的门限：目标点距离超过 pinned_range_m、水平速度低于 pinned_speed_mps
  // 并持续 stranded_s，就认定这架机推不动了。距离门限取在返航路线容差（0.5 m）之上，
  // 免得机体在航点附近正常收敛时被误判。
  const double pinned_range_m=team_settings.value("pinned_range_m",0.6);
  const double pinned_speed_mps=team_settings.value("pinned_speed_mps",0.15);
  if(!std::isfinite(pinned_range_m)||pinned_range_m<=0)throw std::runtime_error("invalid team_mission.pinned_range_m");
  if(!std::isfinite(pinned_speed_mps)||pinned_speed_mps<=0)throw std::runtime_error("invalid team_mission.pinned_speed_mps");
  std::unique_ptr<Udp> team;
  if(team_enabled)team=std::make_unique<Udp>(18000+o.vehicle);
  // 首条任务指令到达前，空对象允许 value() 返回默认值；null 会抛出 type_error.306。
  json team_command=json::object();Clock::time_point team_rx{};uint64_t team_sequence=0,impact_seq=0,last_impact_us=0,last_collision_us=0,contact_enter_us=0;
  companion::PendingInertialContact pending_collision;
  const auto sensor_disturbance=[&](){return std::max({last_impact_us,last_collision_us,pending_collision.since_us});};
  bool team_disarm=false,team_complete=false,team_restart_blocked=false,landing_recorded=false;
  // 趴窝判定：任务要求在空中飞行的阶段里持续落地超过 stranded_s 即认定无法复飞。
  // 一旦置位就不再清除——落在基地结构上的机体不会自己飞起来，而反复置位/清除只会
  // 让协调器在「失败」和「恢复」之间抖动。
  uint64_t stranded_since_us=0;bool unrecoverable=false;
  // 进入终端段后保持瞄准点的 X/Z，新的有效曝光仅有界修正横向位置。
  Eigen::Vector3d dive_aim=Eigen::Vector3d::Zero();
  bool dive_committed=false,dive_retry_entry=false;
  uint64_t dive_commit_us=0;
  uint64_t dive_aim_exposure_us=0;
  uint64_t dive_alignment_since_us=0,last_alignment_report_us=0,last_commit_gate_report_us=0;
  std::string dive_aim_source;
  uint64_t weak_until_us=0,weak_seq=0;int weak_contacts=0;
  uint64_t last_model_broadcast=0,last_learn_report=0;



  double start_delay_s=0.0;
  if(mission.contains("vehicle_start_delays_s")) { const auto key=std::to_string(o.vehicle); if(mission.at("vehicle_start_delays_s").contains(key)) start_delay_s=mission.at("vehicle_start_delays_s").at(key).get<double>(); if(!std::isfinite(start_delay_s)||start_delay_s<0)throw std::runtime_error("vehicle_start_delays_s must be finite and nonnegative"); }
  if(armor_enabled&&o.vehicle!=o.mission_vehicle)config.safe_waypoints.clear();
  if(!armor_enabled) {
    config.waypoint_radius_m=0.15;
    if(config.safe_waypoints.empty())
      config.safe_waypoints.push_back({config.initial_pose.x_m,config.initial_pose.y_m,config.initial_pose.yaw_rad});
  }
  std::filesystem::create_directories(o.runtime);std::ofstream log(o.runtime/("companion_"+std::to_string(o.vehicle)+".jsonl"));if(!log)throw std::runtime_error("cannot open runtime log");
  auto emit=[&](json j){j["vehicle"]=o.vehicle;j["source"]="ESTIMATE";log<<j.dump()<<'\n';log.flush();};
  emit({{"kind","startup"},{"simulation_only",true},{"height_assist_allowed",true},{"map",o.map.string()},{"gnss_position_ignored",true},{"mavlink_local_port",o.local},{"mavlink_remote_port",o.remote}});
  Udp pressure(19000+o.vehicle);uint64_t pressure_sequence=0;
  Udp observation(16000+o.vehicle),peers(17000+o.vehicle),mav(o.local);std::map<int,companion::PeerEstimate>peer_cache;
  Eigen::Vector2d reliable_odometry_xy(config.initial_pose.x_m,config.initial_pose.y_m);
  uint64_t sim_us=0,frame_seq=0,camera_us=0;int target_sys=0,target_comp=0;int waypoint_index=0;companion::FlightFeedback native_feedback;companion::ArmorObservation target;double yaw=0;Clock::time_point imu_rx{},attitude_rx{},obs_rx{},bias_start{};uint64_t first_imu=0,first_imu_sim=0,bias_samples=0;Eigen::Vector3d imu_linear_bias=Eigen::Vector3d::Zero();Eigen::Vector3d bias_sum=Eigen::Vector3d::Zero();bool bias_ready=false,yaw_aligned=false;double yaw_offset=0;
  companion::ImuAttitudeHistory attitude_history;
  uint64_t latest_imu_boot_us=0,attitude_stamp_rejected=0,imu_stamp_rejected=0,imu_attitude_missing=0;
  uint64_t max_attitude_extrapolation_us=0,max_attitude_interpolation_us=0;
  std::array<uint64_t,3> imu_attitude_counts{};
  auto start=Clock::now(),last_publish=start,last_heartbeat=start,last_command=start;bool intervals_requested=false;std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  auto fusion=[&](const companion::FusionReport&r,const char*kind,uint64_t stamp){emit({{"kind","fusion"},{"sensor_kind",kind},{"sim_time_us",stamp},{"accepted",r.accepted},{"gated",r.gated},{"stale",r.stale},{"degenerate",r.degenerate},{"inliers",r.inliers},{"reason",r.reason}});};
  // 单线程轮询：各通道每轮限制包数，避免繁忙通道使其他输入长期得不到处理。
  while(running && (o.duration==0||age(start)<o.duration)){
    auto flight=native_feedback.state(Clock::now());
    // 2. 接收协调器目标；只接受递增 sequence，记录进入接触阶段的时刻。
    if(team)for(int n=0;n<32;++n){
      auto packet=team->receive();if(packet.empty())break;
      try{auto cmd=json::parse(packet);
        if(cmd.value("kind","")!="team_command"||cmd.value("vehicle",-1)!=o.vehicle)continue;
        auto sequence=cmd.at("sequence").get<uint64_t>();if(sequence<=team_sequence)continue;
        bool finite=true;for(const char*key:{"target_x_m","target_y_m","target_z_m","target_yaw_rad","contact_speed_mps"})finite=finite&&cmd.contains(key)&&cmd.at(key).is_number()&&std::isfinite(cmd.at(key).get<double>());
        if(!finite)continue;
        if(cmd.contains("contact_deadline_us")&&(!cmd.at("contact_deadline_us").is_number_integer()||
            (!cmd.at("contact_deadline_us").is_number_unsigned()&&cmd.at("contact_deadline_us").get<int64_t>()<0)))continue;
        if(cmd.value("phase","")=="CONTACT"&&team_command.value("phase","")!="CONTACT"){contact_enter_us=cmd.at("sim_time_us").get<uint64_t>();dive_committed=false;dive_retry_entry=false;dive_commit_us=0;dive_alignment_since_us=0;last_alignment_report_us=0;last_commit_gate_report_us=0;weak_contacts=0;weak_until_us=0;dive_aim_source.clear();}
        // 返航或降落后禁止解锁及重新接管；HOLD 仍须检查全队阶段。
        for(const auto& phase:{cmd.value("phase",std::string{}),cmd.value("mission_phase",std::string{})})
          if(phase=="RETURN_ALL"||phase=="ABORT_RETURN"||phase=="LAND_ALL"||phase=="COMPLETE"||phase=="ABORTED")team_restart_blocked=true;
        team_command=std::move(cmd);team_sequence=sequence;team_rx=Clock::now();
      }catch(const std::exception&){}
    }
    // 板端压力开关只用于命中反馈，不参与视觉定位或轨迹预测。
    // 力度不足的事件不算命中，但会触发一次退避重来，避免机体一直顶在板上。
    if(visual_contact)for(int n=0;n<16;++n){
      auto packet=pressure.receive();if(packet.empty())break;
      try{auto j=json::parse(packet);const auto seq=j.at("sequence").get<uint64_t>();const auto stamp=j.at("sim_time_us").get<uint64_t>();
        if(j.value("kind","")!="armor_pressure"||j.value("source","")!="SIM_ARMOR_PRESSURE_SENSOR"||j.value("vehicle",-1)!=o.vehicle||!j.value("pressed",false)||seq<=pressure_sequence)continue;
        pressure_sequence=seq;
        const bool qualified=j.value("qualified",true);
        const double force=j.value("force_n",0.0);
        if(team_command.value("phase","")=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle&&stamp>=contact_enter_us&&stamp+500000>=sim_us){
          if(qualified){++impact_seq;last_impact_us=stamp;weak_until_us=0;pending_collision.clear();emit({{"kind","impact"},{"sim_time_us",stamp},{"impact_seq",impact_seq},{"force_n",force},{"confirmation","armor_pressure"}});}
          else{++weak_seq;emit({{"kind","weak_impact"},{"sim_time_us",stamp},{"weak_seq",weak_seq},{"force_n",force},{"limit",dive_weak_limit}});
            if(!dive_retry_entry)++weak_contacts;
            weak_until_us=std::max(weak_until_us,stamp+static_cast<uint64_t>(dive_weak_retreat_s*1e6));
            dive_committed=false;dive_retry_entry=true;dive_alignment_since_us=0;}
        }
      }catch(const std::exception&){}
    }
    // 3. 测量按 kind 分流。UWB 必须配合邻机估计；仅有距离不能直接得到世界位置。
    // 碰撞后 0.4 s 暂不使用 ToF，避免剧烈姿态变化时错误的结构匹配。
    for(int n=0;n<256;++n){auto s=observation.receive();if(s.empty())break;try{auto j=json::parse(s);uint64_t stamp=j.value("sim_time_us",uint64_t(0));if(j.value("vehicle",-1)!=o.vehicle||!stamp)continue;sim_us=std::max(sim_us,stamp);obs_rx=Clock::now();if(auto flow=companion::parseOpticalFlowDatagram(s,o.vehicle))fusion(estimator.fuseOpticalFlow(*flow),"optical_flow",stamp);else if(auto anchor=companion::parseAnchorRangeDatagram(s,o.vehicle))fusion(estimator.fuseAnchorRange(*anchor),"uwb_anchor",stamp);else if(auto t=companion::parseTofDatagram(s,o.vehicle)){const auto disturbance=sensor_disturbance();if(!team_enabled||!disturbance||stamp>disturbance+400000)tof_queue.submit(*t);}else if(auto h=companion::parseHeightAssistDatagram(s,o.vehicle))fusion(estimator.fuseHeightAssist(*h),"height_assist",stamp);else if(auto u=companion::parseUwbDatagram(s,o.vehicle)){auto p=peer_cache.find(u->peer_id);if(p!=peer_cache.end())fusion(estimator.fuseUwb(*u,p->second),"uwb",stamp);}}catch(const std::exception&){} }
    // 4. 更新邻机缓存；只保存配置机群成员，是否新鲜由融合器再次判断。
    //    同一通道还承载滑轨观测样本：滑轨全周期 5.6 s，学成一个模型需要两个以上完整
    //    周期（>11 s）的连续观测，而单机一次回合的视觉窗最长只有 9–10 s，谁都学不成，
    //    所以共享的是原始观测而不是学成的模型——合入队友样本后才够长。
    for(int n=0;n<64;++n){auto s=peers.receive();if(s.empty())break;
      if(auto p=companion::parsePeerEstimateDatagram(s,o.vehicle)){if(std::find(vehicle_ids.begin(),vehicle_ids.end(),p->vehicle_id)!=vehicle_ids.end())peer_cache[p->vehicle_id]=*p;}
      else if(dive_enabled){
        if(auto shared=companion::parsePeriodicSamplesDatagram(s,o.vehicle)){
          periodic_motion.merge(*shared);
        }
      }
    }
    // 5. MAVLink 是字节流协议，一包 UDP 可能含多个消息，解析器需跨包保留状态。
    for(int n=0;n<64;++n){auto bytes=mav.receive();if(bytes.empty())break;static mavlink_message_t msg{};static mavlink_status_t status{};for(unsigned char byte:bytes)if(mavlink_parse_char(MAVLINK_COMM_0,byte,&msg,&status)){
      if(msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){
        mavlink_heartbeat_t h;mavlink_msg_heartbeat_decode(&msg,&h);
        if(h.autopilot==MAV_AUTOPILOT_PX4&&msg.sysid!=0&&msg.compid==MAV_COMP_ID_AUTOPILOT1&&
            (!target_sys||(msg.sysid==target_sys&&msg.compid==target_comp))){
          target_sys=msg.sysid;target_comp=msg.compid;
          const unsigned main_mode=(h.custom_mode>>16)&255,sub_mode=(h.custom_mode>>24)&255;
          // PX4 custom_mode：主模式 AUTO=4，子模式 LAND=6。
          native_feedback.heartbeat(Clock::now(),h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED,
              main_mode==6,main_mode==4&&sub_mode==6,
              h.system_status==MAV_STATE_CRITICAL||h.system_status==MAV_STATE_EMERGENCY||h.system_status==MAV_STATE_FLIGHT_TERMINATION);
          flight=native_feedback.state(Clock::now());
        }
      }
      if(target_sys==0||msg.sysid!=target_sys||msg.compid!=target_comp)continue;
      if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
        mavlink_attitude_t a;mavlink_msg_attitude_decode(&msg,&a);
        const auto stamp=companion::attitudeBootTimeUs(a.time_boot_ms,
            latest_imu_boot_us?latest_imu_boot_us:attitude_history.latestTimeUs());
        if(stamp&&std::isfinite(a.roll)&&std::isfinite(a.pitch)&&std::isfinite(a.yaw)&&
            attitude_history.observe(*stamp,Eigen::Quaterniond(rotation(a.roll,-a.pitch,companion::kPi/2-a.yaw)))){
          yaw=a.yaw;attitude_rx=Clock::now();
        }else ++attitude_stamp_rejected;
      }
      else if(msg.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE){mavlink_extended_sys_state_t state;mavlink_msg_extended_sys_state_decode(&msg,&state);native_feedback.landed(Clock::now(),state.landed_state==MAV_LANDED_STATE_ON_GROUND);flight=native_feedback.state(Clock::now());}
      else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
        mavlink_highres_imu_t i; mavlink_msg_highres_imu_decode(&msg,&i);
        if(!i.time_usec||i.time_usec<=latest_imu_boot_us||i.time_usec>static_cast<uint64_t>(std::numeric_limits<int64_t>::max())){
          ++imu_stamp_rejected;continue;
        }
        latest_imu_boot_us=i.time_usec;
        // PX4 initializes and may reset its compass/yaw during early startup.
        // Do not interpret that estimator reset as physical angular motion.
        if(!sim_us||!attitude_history.latestTimeUs()||age(attitude_rx)>.25||age(start)<8.0) continue;
        std::optional<Eigen::Vector3d> body_rate;
        if((i.fields_updated&0x38)==0x38)body_rate=Eigen::Vector3d(i.xgyro,-i.ygyro,-i.zgyro);
        const auto timed_attitude=attitude_history.at(i.time_usec,body_rate);
        if(!timed_attitude){++imu_attitude_missing;continue;}
        if(!first_imu){
          estimator.reset(sim_us);first_imu=i.time_usec;first_imu_sim=sim_us;bias_start=Clock::now();
          emit({{"kind","imu_clock_alignment"},{"sim_time_us",first_imu_sim},{"imu_boot_time_us",first_imu},
              {"attitude_boot_time_us",attitude_history.latestTimeUs()}});
        }
        // 把飞控 IMU 的时间增量映射到首包对应的仿真时刻，拒绝偏差过大的映射。
        int64_t mapped=static_cast<int64_t>(first_imu_sim)+static_cast<int64_t>(i.time_usec)-static_cast<int64_t>(first_imu);
        if(mapped<=0||std::llabs(mapped-static_cast<int64_t>(sim_us))>500000) continue;
        ++imu_attitude_counts[static_cast<std::size_t>(timed_attitude->mode)];
        if(timed_attitude->mode==companion::ImuAttitudeMode::Extrapolated)
          max_attitude_extrapolation_us=std::max(max_attitude_extrapolation_us,i.time_usec-timed_attitude->begin_us);
        if(timed_attitude->mode==companion::ImuAttitudeMode::Interpolated)
          max_attitude_interpolation_us=std::max(max_attitude_interpolation_us,timed_attitude->end_us-timed_attitude->begin_us);
        const auto attitude_rotation=timed_attitude->body_to_world.toRotationMatrix();
        const double roll_flu=std::atan2(attitude_rotation(2,1),attitude_rotation(2,2));
        const double pitch_flu=std::atan2(-attitude_rotation(2,0),std::hypot(attitude_rotation(0,0),attitude_rotation(1,0)));
        // 起点航向补偿只在首次对齐时确定；姿态已采样到本条 IMU 的源时间。
        const double raw_yaw=std::atan2(attitude_rotation(1,0),attitude_rotation(0,0));
        if(!yaw_aligned){yaw_offset=companion::wrapAngle(config.initial_pose.yaw_rad-raw_yaw);yaw_aligned=true;}
        const double aligned_yaw=companion::wrapAngle(raw_yaw+yaw_offset);
        companion::ImuSample sample; sample.sim_time_us=static_cast<uint64_t>(mapped); sample.attitude_valid=true;
        sample.roll_flu_rad=roll_flu; sample.pitch_flu_rad=pitch_flu; sample.yaw_enu_rad=aligned_yaw;
        // 加速度计给的是比力：先 FRD→FLU，再旋到世界系并补重力，得到平移加速度。
        Eigen::Vector3d specific_flu(i.xacc,-i.yacc,-i.zacc);
        const double acceleration_yaw=estimator.estimate().pose.yaw_rad;
        Eigen::Vector3d raw_linear=rotation(roll_flu,pitch_flu,acceleration_yaw)*specific_flu+Eigen::Vector3d(0,0,-9.80665);
        // 冲击识别：必须是当前撞击机、接近基地、超过阶段切换屏蔽期并出现减速度。
        // 这仍是传感器判断；Gazebo 接触日志由独立评估器核验，未作为这里的输入。
        if(team_enabled&&team_command.value("phase","")=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle&&flight.armed&&sim_us>last_impact_us+1000000&&sim_us>contact_enter_us+150000){
          const auto estimate=estimator.estimate();
          const double tx=team_settings.value("target",json::object()).value("x",0.0),ty=team_settings.value("target",json::object()).value("y",0.0);
          const double heading=team_settings.value("approach",json::object()).value("yaw",0.0);
          const double deceleration=-raw_linear.x()*std::cos(heading)-raw_linear.y()*std::sin(heading);
          const double impulse=team_settings.value("directional_impact",false)?deceleration:raw_linear.head<2>().norm();
          const bool visual_near=visual_result.tracking_valid&&visual_result.range_valid&&visual_result.range_m>0&&visual_result.range_m<.16&&visual_result.observation_age_s<.45;
          const bool close_to_target=visual_contact?visual_near:std::hypot(estimate.pose.x_m-tx,estimate.pose.y_m-ty)<team_settings.value("impact_proximity_m",.35);
          if(visual_contact&&impulse>1.0)emit({{"kind","impact_candidate"},{"sim_time_us",sim_us},{"impulse",impulse},{"range_m",visual_result.range_m},{"age_s",visual_result.observation_age_s},{"tracking",visual_result.tracking_valid}});
          // 近目标冲击等待压力反馈；合格压力取消未确认冲击的退出请求。
          if(visual_contact&&dive_committed&&!unrecoverable&&(i.fields_updated&7)==7&&sim_us>last_collision_us+1000000&&
             std::llabs(static_cast<int64_t>(sim_us)-static_cast<int64_t>(estimate.sim_time_us))<=150000&&
             companion::inertialContactCandidate(specific_flu,dive_aim-Eigen::Vector3d(estimate.pose.x_m,
                 estimate.pose.y_m,estimate.pose.z_m),collision_force,collision_proximity)){
            if(pending_collision.observe(sim_us,specific_flu.norm()))
              emit({{"kind","collision_candidate"},{"sim_time_us",sim_us},{"imu_time_us",sample.sim_time_us},
                  {"specific_force_mps2",specific_flu.norm()}});
          }
          if(!visual_contact&&close_to_target&&impulse>(visual_contact?8.0:team_settings.value("impact_acceleration_mps2",2.0))){++impact_seq;last_impact_us=sim_us;if(!visual_contact)estimator.fuseMapContact(std::max(sim_us,estimator.estimate().sim_time_us),Eigen::Vector3d::UnitX(),tx-team_settings.value("contact_radius_m",.041));emit({{"kind","impact"},{"sim_time_us",sim_us},{"impact_seq",impact_seq},{"acceleration_mps2",raw_linear.head<2>().norm()},{"normal_deceleration_mps2",deceleration}});}
        }
        // 未解锁静置期间统计加速度偏置；样本数量和经过时间同时满足才认为可用。
        if(!bias_ready&&!flight.armed&&flight.landed&&age(bias_start)<2.0&&raw_linear.allFinite()&&raw_linear.norm()<40){
          bias_sum+=raw_linear; ++bias_samples; imu_linear_bias=bias_sum/static_cast<double>(bias_samples);
          if(bias_samples>=80&&age(bias_start)>.8) bias_ready=true;
        }
        sample.acceleration_enu_mps2=raw_linear-(bias_ready?imu_linear_bias:Eigen::Vector3d::Zero());
        sample.acceleration_valid=bias_ready&&(i.fields_updated&7)==7&&sample.acceleration_enu_mps2.allFinite()&&sample.acceleration_enu_mps2.norm()<15;
        sample.pressure_valid=i.fields_updated&(1<<9); sample.absolute_pressure_hpa=i.abs_pressure;
        const auto disturbance=sensor_disturbance();
        if(!flight.armed||(team_enabled&&disturbance&&sim_us<disturbance+400000))sample.acceleration_valid=false;
        estimator.predict(sample); estimator.fuseBaroPressure(sample);
        if(!flight.armed&&flight.landed&&std::abs(estimator.estimate().pose.z_m-team_settings.value("home_ground_z_m",config.initial_pose.z_m-.0375)-team_settings.value("body_clearance_m",.0125))<.08)estimator.resetInertialVelocityWhileLanded(sample.sim_time_us);
        imu_rx=Clock::now();
      }      else if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK){mavlink_command_ack_t a;mavlink_msg_command_ack_decode(&msg,&a);emit({{"kind","command_ack"},{"command",a.command},{"result",a.result},{"sim_time_us",sim_us}});}
      // No positions, GPS, simulator states, or PX4 velocity estimates are consumed.
    }}
    // 已解锁失效保护、心跳或控制流中断均终止任务；排队的健康心跳不能恢复原任务。
    flight=native_feedback.state(Clock::now());
    if((flight.failsafe_latched||flight.heartbeat_gap_latched||flight.setpoint_gap_latched)&&!unrecoverable){
      unrecoverable=true;team_restart_blocked=true;
      emit({{"kind","unrecoverable"},{"sim_time_us",sim_us},{"phase",team_command.value("phase","")},
            {"reason",flight.failsafe_latched?"PX4_FAILSAFE":(flight.heartbeat_gap_latched?"HEARTBEAT_GAP":"OFFBOARD_STREAM_GAP")},
            {"native_failsafe_latched",flight.failsafe_latched},{"heartbeat_gap_latched",flight.heartbeat_gap_latched},
            {"setpoint_gap_latched",flight.setpoint_gap_latched}});
    }
    const bool collision_attempt=visual_contact&&dive_committed&&!unrecoverable&&flight.armed&&
        team_command.value("phase","")=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle&&
        (!last_impact_us||last_impact_us<dive_commit_us);
    if(!collision_attempt)pending_collision.clear();
    else if(pending_collision.mature(sim_us,static_cast<uint64_t>(collision_grace_s*1e6))){
      const auto estimate=estimator.estimate();
      emit({{"kind","collision_recovery"},{"sim_time_us",sim_us},{"candidate_time_us",pending_collision.since_us},
          {"specific_force_mps2",pending_collision.peak_specific_force_mps2},{"retry_count",weak_contacts+1},
          {"self_x_m",estimate.pose.x_m},{"self_y_m",estimate.pose.y_m},{"self_z_m",estimate.pose.z_m}});
      last_collision_us=sim_us;pending_collision.clear();dive_committed=false;dive_retry_entry=true;dive_alignment_since_us=0;
      ++weak_contacts;weak_until_us=sim_us+static_cast<uint64_t>(dive_weak_retreat_s*1e6);
    }
    // 6. 请求姿态/IMU/落地状态发送间隔，单位 μs；请求值不等于链路保证的实际频率。
    if(target_sys&&!intervals_requested){command(mav,o.remote,target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,MAVLINK_MSG_ID_ATTITUDE,20000);command(mav,o.remote,target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,MAVLINK_MSG_ID_HIGHRES_IMU,10000);command(mav,o.remote,target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,MAVLINK_MSG_ID_EXTENDED_SYS_STATE,100000);intervals_requested=true;}
    if(dive_enabled){
      const bool was_ready=periodic_motion.ready();
      periodic_motion.service(sim_us);
      if(!was_ready&&periodic_motion.ready())
        emit({{"kind","periodic_shared"},{"sim_time_us",sim_us},{"samples",periodic_motion.sample_count()},{"merged",periodic_motion.merged_count()},{"span_s",periodic_motion.window_span()},{"period_s",periodic_motion.period_estimate()},{"learning_ms",periodic_motion.learningStats().last_ms}});
    }
    auto now=Clock::now();if(std::chrono::duration<double>(now-last_heartbeat).count()>=1){mavlink_message_t msg;mavlink_msg_heartbeat_pack(200,191,&msg,MAV_TYPE_ONBOARD_CONTROLLER,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);sendMav(mav,msg,o.remote);last_heartbeat=now;}
    // 7. 按现实时间的 20 ms 槽位发布；仿真时间取测量时戳，晚到时跳过积压槽位。
    const bool navigation_due=std::chrono::duration<double>(now-last_publish).count()>=.02;
    if(navigation_due){
      const auto slots=std::chrono::duration_cast<std::chrono::milliseconds>(now-last_publish).count()/20;
      last_publish+=std::chrono::milliseconds(slots*20);estimator.advanceTo(sim_us);auto e=estimator.estimate();
      pose_history.push_back(e);while(pose_history.size()>120)pose_history.pop_front();
      // 相机 PPM 与元数据序号/时间一致才检测；移动装甲接触任务使用蓝色灯条像素闭环。
      auto vision_done=vision_worker.poll(sim_us,camera.timeout_us);
      if(vision_done){
        target=vision_done->pixel;frame_seq=vision_done->sequence;camera_us=vision_done->stamp;
        emit({{"kind","vision"},{"sim_time_us",camera_us},{"frame_seq",frame_seq},{"pixel_only",true},{"detected",target.valid},{"reason",vision_done->error.empty()?target.rejection_reason:vision_done->error},{"bright_pixels",target.bright_pixels},{"components",target.candidate_components},{"strips",target.accepted_strips},{"pairs",target.tested_pairs},{"pair_aspect_rejected",target.rejected_pair_aspect},{"max_pair_aspect",target.max_observed_pair_aspect},{"centroid_x_px",target.centroid_x_px},{"centroid_y_px",target.centroid_y_px},{"angular_error_rad",target.angular_error_rad},{"range_m",target.range_m},{"frame_age_ms",(sim_us-camera_us)*.001}});
      }
      auto camera_dir=o.runtime; if(std::filesystem::exists(o.runtime/"camera"))camera_dir=o.runtime/"camera";
      try {
        const auto meta=readJson(camera_dir/("camera_"+std::to_string(o.vehicle)+".json"));
        const auto seq=meta.at("frame_seq").get<uint64_t>(),stamp=meta.at("sim_time_us").get<uint64_t>();
        if(armor_enabled||visual_contact){
          if(seq!=requested_frame_seq&&stamp<=sim_us){
            companion::VisionRequest request;
            request.stamp=stamp;request.sequence=seq;
            request.image_path=(camera_dir/("camera_"+std::to_string(o.vehicle)+".ppm")).string();
            request.camera=camera;request.detection=detection;request.pnp=visual_contact;
            request.pnp_variance_m2=visual_pose_variance;
            request.strip_height_m=team_settings.value("visual_strip_height_m",.055);
            request.bottom_separation_m=team_settings.value("visual_bottom_separation_m",0.0);
            if(!pose_history.empty()){
              auto nearest=std::min_element(pose_history.begin(),pose_history.end(),[&](const auto&a,const auto&b){return std::llabs(static_cast<int64_t>(a.sim_time_us)-static_cast<int64_t>(stamp))<std::llabs(static_cast<int64_t>(b.sim_time_us)-static_cast<int64_t>(stamp));});
              if(std::llabs(static_cast<int64_t>(nearest->sim_time_us)-static_cast<int64_t>(stamp))<80000){
                request.exposure_pose=nearest->pose;request.exposure_covariance=nearest->covariance.topLeftCorner<4,4>();request.pose_valid=true;
              }
            }
            const auto guide_config=team_settings.value("green_guide",json::object());
            if(visual_contact&&request.pose_valid&&guide_config.value("enabled",false)){
              const auto center=guide_config.at("center_world_m").get<std::vector<double>>();
              const auto offset=guide_config.at("to_armor_world_m").get<std::vector<double>>();
              request.guide_center={center[0],center[1],center[2]};request.guide_offset={offset[0],offset[1],offset[2]};
              auto expected=request.guide_center;
              const auto prior=armor_track.at(stamp);if(prior.valid)expected=prior.position-request.guide_offset;
              const auto&pose=request.exposure_pose;
              const Eigen::Vector3d relative=rotation(pose.roll_rad,pose.pitch_rad,pose.yaw_rad).transpose()*(expected-Eigen::Vector3d(pose.x_m,pose.y_m,pose.z_m))-Eigen::Vector3d(.052,0,0);
              if(relative.x()>.1){
                request.guide_mode=companion::GuideMode::Fallback;
                request.guide_x_px=camera.cx_px-camera.fx_px*relative.y()/relative.x();
                request.guide_y_px=camera.cy_px-camera.fy_px*relative.z()/relative.x();
                request.guide_radius_px=std::clamp(camera.fx_px*.4/relative.x(),25.0,220.0);
                request.guide_y_gate_m=.4;request.guide_z_gate_m=.10;
              }
            }
            if(vision_worker.submit(std::move(request)))requested_frame_seq=seq;
          }
        }else if(seq!=frame_seq){frame_seq=seq;camera_us=stamp;emit({{"kind","camera_metadata"},{"sim_time_us",camera_us},{"frame_seq",frame_seq},{"detection_skipped",true}});}
      }catch(const std::exception&){}
      if(!camera_us||camera_us>sim_us||sim_us-camera_us>camera.timeout_us)target.valid=false;
      // 像素闭环结果留作退化参考：PnP 在近距会因灯条轮廓不再是四边形而连续失锁。
      companion::VisualGuidanceResult pixel_result;
      bool pixel_ready=false;
      if(visual_contact){
        pixel_result=visual_tracker.update(sim_us,target,camera_us);
        visual_result=pixel_result;
        pixel_ready=visual_result.target_valid&&visual_result.range_valid&&std::abs(target.angular_error_rad)<.15&&std::abs(target.vertical_error_rad)<.15;
        visual_ready=pixel_ready;
      }

#ifdef COMPANION_WITH_OPENCV
      if(visual_contact){
        if(vision_done&&vision_done->stamp!=visual_pose_stamp){
          visual_pose_stamp=vision_done->stamp;
          DelayedPose task{vision_done->stamp,vision_done->world,vision_done->world_valid,
            vision_done->green?0.0:vision_done->pnp.reprojection_error_px,vision_done->green,vision_done->world_covariance};
          if(vision_done->green_ran)emit({{"kind","green_guide"},{"sim_time_us",task.stamp},{"detected",vision_done->guide.valid},{"accepted",task.green},{"candidates",vision_done->guide.candidates},{"area_px",vision_done->guide.area}});
          vision_queue.submit(task,sim_us);
        }
        if(auto done=vision_queue.poll(sim_us)){
          emit({{"kind","vision_service"},{"sim_time_us",done->stamp},{"completed_sim_us",sim_us},{"frame_age_ms",(sim_us-done->stamp)*.001},{"valid",done->valid},{"received",vision_queue.received},{"completed",vision_queue.completed},{"dropped",vision_queue.dropped}});
          const bool accepted=done->valid&&armor_track.observe(done->stamp,done->world,done->covariance);
          if(dive_robust_linear&&accepted&&std::abs(done->world.y()-team_settings.at("target").value("y",0.0))<=dive_aim_lateral_limit+.08)
            rail_motion_fit.observe(done->stamp,done->world.y());
          if(done->valid)emit({{"kind","visual_pose"},{"sim_time_us",done->stamp},{"completed_sim_us",sim_us},{"frame_age_ms",(sim_us-done->stamp)*.001},{"vision_delay_ms",vision_delay_ms},{"dropped",vision_queue.dropped},{"accepted",accepted},{"pnp",!done->green},{"measurement",done->green?"green_rail_plane":"blue_pnp"},{"x_m",done->world.x()},{"y_m",done->world.y()},{"z_m",done->world.z()},{"reprojection_error_px",done->error},
              {"observation_covariance",{done->covariance(0,0),done->covariance(0,1),done->covariance(0,2),done->covariance(1,0),done->covariance(1,1),done->covariance(1,2),done->covariance(2,0),done->covariance(2,1),done->covariance(2,2)}}});
        }
        auto tracked=armor_track.at(sim_us);
        dive_track=tracked;
        // 周期学习每次曝光只接收一次关联后的原始位置，避免连续外推放大单帧权重。
        // 延迟曝光按原时刻入窗；局部相位的重尾噪声由跨周期折叠与中位数处理。
        if(dive_enabled&&tracked.valid)periodic_motion.observeMeasurement(
            tracked.measurement_time_us,tracked.measurement_position);
        const auto periodic=periodic_motion.predict(sim_us);
        // 周期模型只使用通过关联的视觉观测，且独立周期验证合格才覆盖短时预测。
        if(periodic.valid&&periodic.confidence>=.85){
          tracked.position=periodic.position;tracked.velocity=periodic.velocity;
          emit({{"kind","periodic_prediction"},{"sim_time_us",sim_us},{"confidence",periodic.confidence},{"horizon_s",periodic.horizon},{"y_m",periodic.position.y()}});
        }
        if(tracked.valid){
          visual_result={};visual_ready=false;
          Eigen::Vector3d relative=rotation(0,0,-e.pose.yaw_rad)*(tracked.position-Eigen::Vector3d(e.pose.x_m,e.pose.y_m,e.pose.z_m));
          Eigen::Vector3d feedforward=rotation(0,0,-e.pose.yaw_rad)*tracked.velocity;
          visual_result.tracking_valid=tracked.age_s<=.35||relative.x()<.3;visual_result.target_valid=tracked.measured;visual_result.range_valid=true;
          visual_result.range_m=relative.x();visual_result.observation_age_s=tracked.age_s;
          visual_result.yaw_error_rad=std::atan2(relative.y(),relative.x());
          visual_result.velocity_flu_mps.y()=std::clamp(2.0*relative.y()+feedforward.y(),-.65,.65);
          visual_result.velocity_flu_mps.z()=std::clamp(2.0*relative.z()+feedforward.z(),-.4,.4);
          // 板很近时允许最多0.8s的已锁定轨迹延拓，解决灯条离开视野到机体接触间的盲区。
          // 就绪帧龄单独配置，默认 150 ms，与 ArmorTracker 的 measured 门限一致。
          const double ready_age=team_settings.value("visual_ready_age_s",.15);
          const bool aligned=std::abs(relative.y())<team_settings.value("visual_lateral_gate_m",.10)&&std::abs(relative.z())<team_settings.value("visual_vertical_gate_m",.075);
          if(aligned&&visual_result.tracking_valid&&(tracked.measured||relative.x()<.3))visual_result.velocity_flu_mps.x()=team_settings.value("contact_speed_mps",.18);
          visual_ready=tracked.valid&&tracked.age_s<=ready_age&&std::abs(relative.y())<.12&&std::abs(relative.z())<.1;
          if(team_settings.value("intercept_enabled",false)&&visual_result.tracking_valid){
            // 滑轨只沿世界 Y 轴移动；相遇预测排除 X/Z 的速度估计噪声。
            const Eigen::Vector3d rail_velocity(0.0,tracked.velocity.y(),0.0);
            const double horizon=std::max(.05,team_settings.value("intercept_horizon_s",.30)-tracked.age_s);
            const auto plan=companion::planIntercept({e.pose.x_m,e.pose.y_m,e.pose.z_m},e.velocity_enu_mps,tracked.position,rail_velocity,
                team_settings.value("intercept_speed_mps",1.0),team_settings.value("intercept_accel_mps2",1.5),horizon);
            if(plan.valid){
              // 用具名 Vector3d 立即求值，避免 Eigen 惰性乘积持有临时旋转矩阵的悬垂引用。
              const Eigen::Vector3d body=rotation(0,0,-e.pose.yaw_rad)*plan.velocity;
              visual_result.velocity_flu_mps=body;
              visual_result.velocity_flu_mps.z()=std::clamp(body.z(),-.4,.4);
              // 贴近目标后不再允许反向：撞击任务里倒车只会把目标推远、让画面更小，
              // 从而制造更多失锁。距离门由视觉进入距离保证不会太远。
              if(relative.x()<=team_settings.value("visual_no_reverse_range_m",1.2))
                visual_result.velocity_flu_mps.x()=std::max(0.0,body.x());
              // 随帧龄降低前向速度，保留配置的推进比例下限。
              const double fresh=team_settings.value("visual_fresh_age_s",.10);
              const double stale=team_settings.value("visual_stale_age_s",.40);
              const double scale=std::clamp((stale-tracked.age_s)/std::max(1e-3,stale-fresh),0.0,1.0);
              visual_result.velocity_flu_mps.x()*=std::max(team_settings.value("visual_min_forward_scale",.3),scale);
              // 就绪检查航向与帧龄，横向追赶交给接触阶段的视觉闭环。
              visual_ready=tracked.valid&&tracked.age_s<=ready_age&&relative.x()>0&&std::abs(visual_result.yaw_error_rad)<.5;
              emit({{"kind","intercept_plan"},{"sim_time_us",sim_us},{"arrival_s",plan.arrival_s},{"reachable",plan.reachable},{"aim_x_m",plan.aim.x()},{"aim_y_m",plan.aim.y()},{"aim_z_m",plan.aim.z()}});
            }
          }

        }else{
          // 近距灯条轮廓可能不满足 PnP 四角条件；无可用跟踪时保留像素闭环与就绪判据。
          visual_result=pixel_result;visual_ready=pixel_ready;
        }
        // 视觉门限诊断：把像素闭环与 PnP 两条路各自的判决分开记录，
        // 否则「卡在进入距离内不动」无法区分是检测失败、测距失败还是就绪判据否决。
        double gate_rel_y=0;
        if(tracked.valid){
          const Eigen::Vector3d gate_rel=rotation(0,0,-e.pose.yaw_rad)*(tracked.position-Eigen::Vector3d(e.pose.x_m,e.pose.y_m,e.pose.z_m));
          gate_rel_y=gate_rel.y();
        }
        emit({{"kind","visual_gate"},{"sim_time_us",sim_us},
              {"pixel_detected",target.valid},{"pixel_clipped",target.image_clipped},{"pixel_conf",target.confidence},
              {"pixel_ang",target.angular_error_rad},{"pixel_vert",target.vertical_error_rad},
              {"pixel_range_valid",target.range_valid},{"pixel_range_m",target.range_m},
              {"pixel_ready",pixel_ready},{"pixel_age_s",pixel_result.observation_age_s},
              {"pnp_valid",tracked.valid},{"pnp_measured",tracked.measured},{"pnp_age_s",tracked.age_s},
              {"rel_y_m",gate_rel_y},
              {"visual_ready",visual_ready},
              {"vis_vx",visual_result.velocity_flu_mps.x()},{"vis_vy",visual_result.velocity_flu_mps.y()}});
      }
#endif
      if(armor_enabled&&!visual_contact&&o.vehicle!=o.mission_vehicle)target.valid=false;
      std::vector<companion::PeerEstimate>p;for(auto&entry:peer_cache)p.push_back(entry.second);const bool schedule_active=sim_us>=static_cast<uint64_t>(start_delay_s*1e6);auto intention=companion::makeControlIntention(sim_us,e,target,camera,p,schedule_active?config.safe_waypoints:std::vector<companion::Pose2>{},&waypoint_index,config.waypoint_radius_m,armor_enabled&&schedule_active);if(!schedule_active){intention.mode="HOLD_DELAY";intention.velocity_flu_mps.setZero();intention.target_valid=false;intention.waypoint_valid=false;}
      flight=native_feedback.state(Clock::now());
      const bool live=sim_us&&age(obs_rx)<1.0&&age(imu_rx)<.5&&flight.heartbeat_fresh&&age(attitude_rx)<.5;intention.safe_to_arm=intention.safe_to_arm&&live&&!flight.failsafe_active&&!flight.failsafe_latched&&!flight.heartbeat_gap_latched&&!flight.setpoint_gap_latched;
      if(live&&e.z_valid&&!team_enabled){
        if(e.pose.z_m<o.takeoff_height-.15){intention.mode="TAKEOFF";intention.velocity_flu_mps.head<2>().setZero();intention.velocity_flu_mps.z()=std::clamp((o.takeoff_height-e.pose.z_m)*.7,0.,.6);}
        else if(!intention.target_valid)intention.velocity_flu_mps.z()=std::clamp((o.takeoff_height-e.pose.z_m)*.7,-.4,.4);
      }
      // 8. 队伍任务覆盖普通航点/视觉控制：协调器管阶段，本机把目标误差转成速度。
      if(team_enabled){
        team_disarm=false;team_complete=false;
        const bool command_fresh=!team_command.empty()&&age(team_rx)<1.0;
        intention.mode=command_fresh?team_command.value("phase","WAIT_READY"):"WAIT_TEAM";
        intention.velocity_flu_mps.setZero();intention.yaw_rate_ref_rad_s=0;
        intention.target_valid=false;intention.waypoint_valid=false;intention.avoidance_active=false;
        const bool team_ready=live&&e.z_valid&&(e.xy_valid||e.flow_valid)&&!flight.failsafe_active&&!flight.failsafe_latched&&!flight.heartbeat_gap_latched&&!flight.setpoint_gap_latched;
        intention.safe_to_arm=team_ready&&command_fresh;
        // 飞行阶段持续贴地或卡住时判为无法继续任务。
        // 阶段指令过期时保留计时器，等待有效指令后再检查持续时间。
        if(command_fresh){
          const std::string phase_now=team_command.value("phase","");
          const bool airborne_phase=phase_now=="TRANSIT"||phase_now=="ASSEMBLE"||phase_now=="APPROACH"||phase_now=="CONTACT"||phase_now=="RETREAT"||phase_now=="RETURN_ALL"||phase_now=="ABORT_RETURN";
          const double ground_ceiling=team_settings.value("home_ground_z_m",.5)+team_settings.value("body_clearance_m",.016)+.15;
          // 估计高度补充飞控着地标志，以覆盖机体压在结构件上的情况。
          const bool down=flight.landed||(e.z_valid&&e.pose.z_m<ground_ceiling);
          // 目标仍远而水平速度持续接近零，覆盖高于地面门限的结构卡住情况。
          const double dx=team_command.value("target_x_m",e.pose.x_m)-e.pose.x_m;
          const double dy=team_command.value("target_y_m",e.pose.y_m)-e.pose.y_m;
          const double speed=std::hypot(e.velocity_enu_mps.x(),e.velocity_enu_mps.y());
          const bool pinned=airborne_phase&&e.xy_valid&&std::hypot(dx,dy)>pinned_range_m&&speed<pinned_speed_mps;
          if((down||pinned)&&airborne_phase){if(!stranded_since_us)stranded_since_us=sim_us;}
          else stranded_since_us=0;
          if(!unrecoverable&&stranded_since_us&&sim_us>=stranded_since_us+static_cast<uint64_t>(stranded_s*1e6)){
            unrecoverable=true;
            emit({{"kind","unrecoverable"},{"sim_time_us",sim_us},{"phase",phase_now},{"landed_s",(sim_us-stranded_since_us)*1e-6},{"z_m",e.pose.z_m},{"ceiling_m",ground_ceiling},{"down",down},{"pinned",pinned},{"target_range_m",std::hypot(dx,dy)},{"speed_mps",speed}});
          }
        }
        if(command_fresh){
          team_disarm=team_command.value("disarm",false);team_complete=team_command.value("complete",false);
          Eigen::Vector3d delta(team_command.at("target_x_m").get<double>()-e.pose.x_m,team_command.at("target_y_m").get<double>()-e.pose.y_m,team_command.at("target_z_m").get<double>()-e.pose.z_m);
          // v_ref = Kp*(目标位置-估计位置) - Kd*估计速度。阻尼减少高速到点后的摆动。
          Eigen::Vector3d desired=delta*team_settings.value("position_gain",.65);
          desired.head<2>()-=team_settings.value("velocity_damping",0.0)*e.velocity_enu_mps.head<2>();
          double max_speed=team_settings.value("cruise_speed_mps",.35);if(!e.xy_valid)max_speed=std::min(max_speed,.12);
          double horizontal=desired.head<2>().norm();if(horizontal>max_speed)desired.head<2>()*=max_speed/horizontal;
          desired.z()=std::clamp(desired.z(),-team_settings.value("descent_speed_mps",.20),team_settings.value("climb_speed_mps",.35));
          if(intention.mode=="LAND_ALL")desired.z()=flight.landed?0.0:-team_settings.value("landing_speed_mps",.12);
          const bool touchdown_zone=intention.mode=="LAND_ALL"&&e.pose.z_m<team_settings.value("home_ground_z_m",.5)+team_settings.value("body_clearance_m",.016)+.03;
          if(touchdown_zone)desired.head<2>().setZero();
          if(touchdown_zone&&!flight.landed)desired.z()=-team_settings.value("touchdown_descent_mps",team_settings.value("landing_speed_mps",.12));
          // 接触阶段用固定航向速度推进，防止位置环在到达接近点时提前停止。
          if(!visual_contact&&intention.mode=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle){
            const double heading=team_command.at("target_yaw_rad").get<double>();
            const double speed=std::max(0.0,team_command.at("contact_speed_mps").get<double>());
            desired.x()=std::cos(heading)*speed;desired.y()=std::sin(heading)*speed;
          }
          // 世界系速度参考转为水平机体系；高度仍向上为正，发 MAVLink 时再转符号。
          intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
          double heading=team_command.at("target_yaw_rad").get<double>();
          const double yaw_limit=team_settings.value("yaw_rate_limit_rad_s",.5);
          intention.yaw_rate_ref_rad_s=std::clamp(companion::wrapAngle(heading-e.pose.yaw_rad)*team_settings.value("yaw_gain",.8),-yaw_limit,yaw_limit);
          if(touchdown_zone)intention.yaw_rate_ref_rad_s=0;
          // 终端接近仅使用相机角误差和视觉深度，绝不查询滑轨相位或板世界位置。
          if(visual_contact&&team_command.value("active_vehicle",-1)==o.vehicle&&
             (intention.mode=="APPROACH"||intention.mode=="CONTACT")){
            intention.velocity_flu_mps=visual_result.velocity_flu_mps;
            if(intention.mode=="APPROACH"){
              // 接近阶段前向由任务位置环驱动到搜索点，横向与垂向仍由视觉闭环对准。
              // visual_approach_speed_mps 限制前向推进速度；0 表示不前推。
              const double cap=team_settings.value("visual_approach_speed_mps",.8);
              const Eigen::Vector3d to_waypoint=rotation(0,0,-e.pose.yaw_rad)*desired;
              intention.velocity_flu_mps.x()=std::clamp(to_waypoint.x(),-cap,cap);
            }
            // 接近阶段由任务偏航控制对准搜索点，接触阶段使用视觉偏航参考。
            if(intention.mode=="CONTACT")intention.yaw_rate_ref_rad_s=visual_result.yaw_rate_ref_rad_s;
            intention.target_valid=visual_result.target_valid;
            intention.target_range_m=visual_result.range_m;
            intention.target_yaw_error_rad=visual_result.yaw_error_rad;
            if(visual_result.tracking_valid)visual_lost_since=0;
            if(!visual_result.tracking_valid){
              if(intention.mode=="APPROACH"){
                // 未锁定前到达配置搜索点；一旦锁定，仅视觉驱动接近。搜索点不是装甲瞬时位置。
                // visual_search_speed_mps 限制无视觉跟踪时的搜索速度。
                intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
                const double horizontal=intention.velocity_flu_mps.head<2>().norm();
                const double search_speed=team_settings.value("visual_search_speed_mps",.35);
                if(horizontal>search_speed)intention.velocity_flu_mps.head<2>()*=search_speed/horizontal;
              }else {
                // 短暂失锁时保持小幅前推；超过滑行时限后使用任务位置参考。
                if(!visual_lost_since)visual_lost_since=sim_us;
                const double lost_s=static_cast<double>(sim_us-visual_lost_since)*1e-6;
                if(lost_s<team_settings.value("visual_lost_coast_s",1.0)){
                  // 沿用最后一次已知的推进方向，横向与垂向归零，避免用陈旧误差左右乱摆。
                  intention.velocity_flu_mps.setZero();
                  intention.velocity_flu_mps.x()=team_settings.value("visual_lost_push_mps",.18);
                }else{
                  intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
                  // 接触阶段一律不倒车：这里回退到的是静态任务目标，而无人机可能已经
                  // 越过它，于是指令变成倒退。倒车会让目标更小、更难重新捕获，形成正反馈。
                  if(intention.mode=="CONTACT")intention.velocity_flu_mps.x()=std::max(0.0,intention.velocity_flu_mps.x());
                  const double speed=intention.velocity_flu_mps.head<2>().norm();
                  if(speed>.3)intention.velocity_flu_mps.head<2>()*=.3/speed;
                }
              }
              // 只有接触阶段才冻结偏航；接近阶段失去视觉时仍要能转向搜索点。
              if(intention.mode=="CONTACT")intention.yaw_rate_ref_rad_s=0;
            }
          }
          // 俯冲模式的接近阶段：入口是固定航点，整条三维轨迹都交给任务位置环。
          // 入口高于板面，接近阶段不能由视觉高度对准覆盖爬升；视觉只提供接触落点。
          if(dive_enabled&&intention.mode=="APPROACH"&&team_command.value("active_vehicle",-1)==o.vehicle){
            intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
            const double cap=team_settings.value("visual_approach_speed_mps",.8);
            const double horizontal=intention.velocity_flu_mps.head<2>().norm();
            if(horizontal>cap)intention.velocity_flu_mps.head<2>()*=cap/horizontal;
            intention.yaw_rate_ref_rad_s=std::clamp(companion::wrapAngle(heading-e.pose.yaw_rad)*team_settings.value("yaw_gain",.8),-yaw_limit,yaw_limit);
          }
          // 终端段按世界系落点推进；接触、过板或超时后退出。
          if(dive_enabled&&intention.mode=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle){
            const Eigen::Vector3d self(e.pose.x_m,e.pose.y_m,e.pose.z_m);
            const double heading=team_command.at("target_yaw_rad").get<double>();
            const bool passed_plane=dive_committed&&dive_plane_retreat&&team_ready&&
                companion::passedTargetPlane(self,dive_aim,heading,dive_exit_margin);
            const bool contact_confirmed=last_impact_us&&last_impact_us>=dive_commit_us;
            const bool unconfirmed_pass=passed_plane&&!contact_confirmed;
            // 已越过板面且未确认接触时及时退避，时间上限覆盖位置估计不可用的情况。
            if(dive_committed&&(unconfirmed_pass||sim_us>dive_commit_us+static_cast<uint64_t>(dive_timeout_s*1e6))){
              ++weak_seq;emit({{"kind","dive_miss"},{"sim_time_us",sim_us},{"weak_seq",weak_seq},{"limit",dive_weak_limit},
                  {"reason",unconfirmed_pass?"passed_plane":"timeout"},{"elapsed_s",(sim_us-dive_commit_us)*1e-6},
                  {"forward_overrun_m",(self.x()-dive_aim.x())*std::cos(heading)+(self.y()-dive_aim.y())*std::sin(heading)}});
              dive_committed=false;
              dive_retry_entry=true;
              dive_alignment_since_us=0;
              ++weak_contacts;weak_until_us=sim_us+static_cast<uint64_t>(dive_weak_retreat_s*1e6);
            }
            if(dive_retry_entry&&sim_us>=weak_until_us&&weak_contacts<=dive_weak_limit&&flight.armed&&team_ready&&
                (self-dive_entry).head<2>().norm()<=dive.value("entry_tolerance_m",.2)&&
                std::abs(self.z()-dive_entry.z())<=dive.value("entry_height_tolerance_m",.12)&&
                e.velocity_enu_mps.norm()<=.3&&
                std::abs(companion::wrapAngle(heading-e.pose.yaw_rad))<=dive.value("entry_yaw_tolerance_rad",.35)){
              dive_retry_entry=false;
              emit({{"kind","dive_retry_entry"},{"sim_time_us",sim_us},{"weak_contacts",weak_contacts},
                  {"self_x_m",self.x()},{"self_y_m",self.y()},{"self_z_m",self.z()},
                  {"speed_mps",e.velocity_enu_mps.norm()}});
            }
            if(sim_us<weak_until_us||dive_retry_entry){
              // 退避时间结束后仍需到达入口并减速；次数用尽则保持入口等待轮次结束。
              intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
              dive_committed=false;
              dive_alignment_since_us=0;
            }else{
              bool aligning=false;
              Eigen::Vector3d alignment_velocity=Eigen::Vector3d::Zero();
              const auto arrival_time=[&](const Eigen::Vector3d& aim){
                const Eigen::Vector3d direction=aim-self;
                const double distance=direction.norm();
                if(!dive_acceleration_arrival){
                  const double speed=std::clamp(std::hypot(e.velocity_enu_mps.x(),e.velocity_enu_mps.y()),
                      dive_min_forward,dive_impact_speed);
                  return distance/speed;
                }
                const double radial=distance>1e-6?e.velocity_enu_mps.dot(direction/distance):0.0;
                return companion::estimateArrivalTime(distance,radial,dive_impact_speed,
                    dive_arrival_acceleration).value_or(0.0);
              };
              const auto robust=rail_motion_fit.at(sim_us);
              if(!dive_committed){
                // 装甲板只沿世界 Y 轴平移；X/Z 使用结构中心，避免外推视觉噪声。
                const auto target=team_settings.value("target",json::object());
                const Eigen::Vector3d anchor(target.value("x",0.0),target.value("y",0.0),target.value("z",0.0));
                // 默认用有界实测水平速度；可配置为沿目标方向的加速、限速到达模型。
                const double lead=dive_entry_alignment?dive_alignment_horizon:arrival_time(anchor);
                // 落点预测优先用学到的滑轨周期模板，其次用跟踪速度线性外推；
                // 无有效预测时回到入口等待观测。
                const auto predicted=periodic_motion.predict(sim_us+static_cast<uint64_t>(lead*1e6));
                std::string source="nominal";
                double lateral=anchor.y();
                double target_velocity=0;
                bool linear_valid=false;
                const bool periodic_valid=predicted.valid&&predicted.confidence>=dive_predict_confidence;
                if(periodic_valid){lateral=predicted.position.y();target_velocity=predicted.velocity.y();source="periodic";}
                else if((dive_robust_linear&&robust)||(!dive_robust_linear&&dive_track.valid)){
                  const double observed_position=robust&&dive_robust_linear?robust->position_m:dive_track.position.y();
                  const double observed_velocity=robust&&dive_robust_linear?robust->velocity_mps:dive_track.velocity.y();
                  const auto rail=companion::predictRailLateral(observed_position,observed_velocity,
                      lead,anchor.y(),dive_linear_speed_limit,dive_reflection_half_width);
                  if(rail){
                    linear_valid=true;
                    lateral=*rail;source=dive_reflection_half_width>0?"linear_reflected":"linear";
                    const auto next=companion::predictRailLateral(observed_position,observed_velocity,
                        lead+.02,anchor.y(),dive_linear_speed_limit,dive_reflection_half_width);
                    target_velocity=next?(*next-*rail)/.02:0;
                  }
                }
                // 将预测 Y 限制在配置的滑轨横向范围内。
                const double lateral_limit=dive_aim_lateral_limit;
                const double clamped=std::clamp(lateral,anchor.y()-lateral_limit,anchor.y()+lateral_limit);
                if(clamped!=lateral)source+="_clamped";
                const Eigen::Vector3d aim(anchor.x(),clamped,anchor.z());
                dive_aim=aim;
                const bool prediction_usable=(periodic_valid||linear_valid)&&dive_track.measured;
                const auto contact_deadline=team_command.value("contact_deadline_us",uint64_t(0));
                const bool attempt_window=companion::diveAttemptFitsWindow(sim_us,contact_deadline,dive_timeout_s);
                const Eigen::Vector3d body_displacement=rotation(0,0,-e.pose.yaw_rad)*(aim-self);
                const Eigen::Vector3d body_velocity=rotation(0,0,-e.pose.yaw_rad)*e.velocity_enu_mps;
                const bool terminal_available=companion::terminalEntryVelocityCompatible(body_displacement,
                    body_velocity,dive_impact_speed,dive_lateral_limit,dive_descent_limit,dive_min_forward);
                const bool entry_eligible=flight.armed&&team_ready&&prediction_usable&&attempt_window&&
                    companion::diveCommitEligible(self,e.velocity_enu_mps,aim,e.pose.yaw_rad,heading,
                        dive_entry.z(),dive_track.measurement_time_us,sim_us,
                        dive.value("entry_height_tolerance_m",.12),dive.value("entry_yaw_tolerance_rad",.35));
                const bool commit_eligible=entry_eligible&&terminal_available;
                if(dive_entry_alignment){
                  target_velocity=std::clamp(target_velocity,-dive_linear_speed_limit,dive_linear_speed_limit);
                  const Eigen::Vector3d entry(dive_entry.x(),aim.y(),dive_entry.z());
                  const Eigen::Vector3d feedforward(0,target_velocity,0);
                  alignment_velocity=(entry-self)*team_settings.value("position_gain",.65)+feedforward+
                      team_settings.value("velocity_damping",0.0)*(feedforward-e.velocity_enu_mps);
                  alignment_velocity.x()=std::clamp(alignment_velocity.x(),-dive_impact_speed,dive_impact_speed);
                  alignment_velocity.y()=std::clamp(alignment_velocity.y(),-dive_lateral_limit,dive_lateral_limit);
                  alignment_velocity.z()=std::clamp(alignment_velocity.z(),-dive_descent_limit,dive_descent_limit);
                  const bool ready=commit_eligible&&
                      std::abs(entry.x()-self.x())<=dive.value("entry_tolerance_m",.2)&&
                      std::abs(entry.z()-self.z())<=dive.value("entry_height_tolerance_m",.12)&&
                      std::abs(entry.y()-self.y())<=dive_alignment_tolerance&&
                      e.velocity_enu_mps.x()*e.velocity_enu_mps.x()+e.velocity_enu_mps.z()*e.velocity_enu_mps.z()<=dive_alignment_speed*dive_alignment_speed&&
                      std::abs(e.velocity_enu_mps.y()-target_velocity)<=dive_alignment_speed&&
                      std::abs(companion::wrapAngle(heading-e.pose.yaw_rad))<=dive.value("entry_yaw_tolerance_rad",.35);
                  if(!ready)dive_alignment_since_us=0;
                  else if(!dive_alignment_since_us)dive_alignment_since_us=sim_us;
                  dive_committed=ready&&sim_us>=dive_alignment_since_us+100000;
                  aligning=!dive_committed;
                  if(dive_committed||sim_us>=last_alignment_report_us+200000){
                    last_alignment_report_us=sim_us;
                    emit({{"kind","dive_alignment"},{"sim_time_us",sim_us},{"ready",ready},{"committed",dive_committed},
                        {"error_y_m",entry.y()-self.y()},{"vy_mps",e.velocity_enu_mps.y()},{"target_vy_mps",target_velocity},
                        {"error_x_m",entry.x()-self.x()},{"error_z_m",entry.z()-self.z()},{"aim_source",source},{"lead_s",lead},
                        {"rail_fit_valid",bool(robust)},{"rail_fit_y_m",robust?robust->position_m:0.0},
                        {"rail_fit_vy_mps",robust?robust->velocity_mps:0.0}});
                  }
                }else{
                  const bool in_commit_range=std::hypot(aim.x()-self.x(),aim.y()-self.y())<=dive_commit_range;
                  dive_committed=commit_eligible&&in_commit_range;
                  if(!entry_eligible||(in_commit_range&&!terminal_available)){aligning=true;alignment_velocity=desired;}
                }
                if(!commit_eligible&&sim_us>=last_commit_gate_report_us+200000){
                  last_commit_gate_report_us=sim_us;
                  emit({{"kind","dive_commit_gate"},{"sim_time_us",sim_us},{"eligible",false},
                      {"attempt_window_available",attempt_window},{"contact_deadline_us",contact_deadline},
                      {"terminal_motion_available",terminal_available},
                      {"prediction_usable",prediction_usable},{"track_measured",dive_track.measured},
                      {"measurement_time_us",dive_track.measurement_time_us},{"aim_source",source},
                      {"self_x_m",self.x()},{"self_z_m",self.z()},
                      {"vertical_speed_mps",e.velocity_enu_mps.z()},
                      {"forward_speed_mps",e.velocity_enu_mps.x()*std::cos(heading)+e.velocity_enu_mps.y()*std::sin(heading)}});
                }
                if(dive_committed){dive_commit_us=sim_us;dive_aim_exposure_us=dive_track.valid?dive_track.measurement_time_us:sim_us;}
                // 记录来源变化和终端段提交；新曝光修正单独记录。
                if(source!=dive_aim_source||dive_committed){
                  dive_aim_source=source;
                  emit({{"kind","dive_aim"},{"sim_time_us",sim_us},{"aim_source",source},{"lead_s",lead},{"committed",dive_committed},{"aim_x_m",aim.x()},{"aim_y_m",aim.y()},{"aim_z_m",aim.z()},{"predicted_confidence",predicted.valid?predicted.confidence:0.0},
                      {"entry_alignment",dive_entry_alignment},{"self_x_m",self.x()},{"self_y_m",self.y()},{"self_z_m",self.z()},
                      {"self_vx_mps",e.velocity_enu_mps.x()},{"self_vy_mps",e.velocity_enu_mps.y()},{"track_y_m",dive_track.position.y()},{"track_vy_mps",dive_track.velocity.y()},{"track_valid",dive_track.valid},
                      {"track_measured",dive_track.measured},{"measurement_time_us",dive_track.measurement_time_us},{"commit_eligible",commit_eligible}});
                }
              }
              if(dive_aim_correction_rate>0&&dive_committed&&dive_track.valid&&dive_track.measured){
                const double lead=arrival_time(dive_aim);
                const auto predicted=periodic_motion.predict(sim_us+static_cast<uint64_t>(lead*1e6));
                const double center=team_settings.at("target").value("y",0.0);
                double lateral=companion::predictRailLateral(dive_track.position.y(),dive_track.velocity.y(),
                    lead,center,dive_linear_speed_limit,dive_reflection_half_width).value_or(center);
                if(predicted.valid&&predicted.confidence>=dive_predict_confidence)lateral=predicted.position.y();
                lateral=std::clamp(lateral,center-dive_aim_lateral_limit,center+dive_aim_lateral_limit);
                if(companion::correctLateralAim(dive_aim.y(),dive_aim_exposure_us,lateral,
                    dive_track.measurement_time_us,sim_us,dive_aim_correction_rate))
                  emit({{"kind","dive_aim_correction"},{"sim_time_us",sim_us},
                    {"measurement_time_us",dive_aim_exposure_us},{"aim_y_m",dive_aim.y()},{"lead_s",lead}});
              }
              // 水平朝落点飞，垂向按「到达落点 x 时正好到落点 z」配比：
              // vz = dz * vx / dx，使用实测水平速度匹配受推力和倾角限制的到达速度。
              if(aligning){
                intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*alignment_velocity;
                intention.target_valid=dive_track.measured;
              }else{
              const double dx=dive_aim.x()-self.x();
              const double dz=dive_aim.z()-self.z();
              const Eigen::Vector3d horizontal(dx,dive_aim.y()-self.y(),0.0);
              Eigen::Vector3d body_velocity=Eigen::Vector3d::Zero();
              if(horizontal.norm()>1e-3)body_velocity=rotation(0,0,-e.pose.yaw_rad)*(horizontal.normalized()*dive_impact_speed);
              body_velocity.y()=std::clamp(body_velocity.y(),-dive_lateral_limit,dive_lateral_limit);
              // 前向速度下限：越过瞄准点后 delta 反向，钳位保证继续前推而不是倒车停住。
              body_velocity.x()=std::max(body_velocity.x(),dive_min_forward);
              const double closing=std::max(.2,std::hypot(e.velocity_enu_mps.x(),e.velocity_enu_mps.y()));
              body_velocity.z()=std::clamp(dx>.05?dz*closing/dx:0.0,-dive_descent_limit,dive_descent_limit);
              intention.velocity_flu_mps=body_velocity;
              intention.target_valid=true;
              }
            }
            intention.yaw_rate_ref_rad_s=std::clamp(companion::wrapAngle(heading-e.pose.yaw_rad)*team_settings.value("yaw_gain",.8),-yaw_limit,yaw_limit);
          }

          // 无法继续任务的成员交给飞控自动降落，不再接受任务速度参考。
          if(unrecoverable){intention.mode="RECOVER_LAND";intention.velocity_flu_mps.setZero();intention.yaw_rate_ref_rad_s=0;intention.target_valid=false;}
          const auto disturbance=std::max(last_impact_us,last_collision_us);
          if(intention.mode=="WAIT_READY"||intention.mode=="HOLD"||(disturbance&&sim_us<disturbance+static_cast<uint64_t>(team_settings.value("impact_settle_s",.4)*1e6))||team_disarm||team_complete){intention.velocity_flu_mps.setZero();intention.yaw_rate_ref_rad_s=0;}
        }
        team->send(json{{"schema",1},{"kind","team_status"},{"vehicle",o.vehicle},{"sim_time_us",sim_us},{"x_m",e.pose.x_m},{"y_m",e.pose.y_m},{"z_m",e.pose.z_m},{"yaw_rad",e.pose.yaw_rad},{"vx_mps",e.velocity_enu_mps.x()},{"vy_mps",e.velocity_enu_mps.y()},{"vz_mps",e.velocity_enu_mps.z()},{"xy_valid",e.xy_valid||e.flow_valid},{"z_valid",e.z_valid},{"ready",team_ready},{"xy_reason",e.localization_mode},{"flow_valid",e.flow_valid},{"visual_ready",visual_ready},{"position_confidence",e.position_confidence},{"armed",flight.armed},{"landed",flight.landed},{"heartbeat_fresh",flight.heartbeat_fresh},{"landed_fresh",flight.landed_fresh},{"native_failsafe_active",flight.failsafe_active},{"native_failsafe_latched",flight.failsafe_latched},{"heartbeat_gap_latched",flight.heartbeat_gap_latched},{"setpoint_gap_latched",flight.setpoint_gap_latched},{"unrecoverable",unrecoverable},{"impact_seq",impact_seq}}.dump(),18100);
      }
      if(team_enabled&&flight.landed&&!landing_recorded&&(team_command.value("phase","")=="LAND_ALL"||team_complete)){emit({{"kind","contact_final"},{"sim_time_us",sim_us},{"landed",true},{"phase","LANDED"}});landing_recorded=true;}
      if(unrecoverable)intention.safe_to_arm=false;
      if(!intention.safe_to_arm){intention.mode=unrecoverable?"RECOVER_LAND":(flight.armed?"RECOVER_ODOMETRY":"WAIT_SENSORS");intention.velocity_flu_mps.setZero();intention.yaw_rate_ref_rad_s=0;}
      auto message=companion::serializeEstimateDatagram(e);for(int i:vehicle_ids)if(i!=o.vehicle)peers.send(message,17000+i);log<<message<<'\n';
      // 本机的滑轨观测按 2 Hz 广播给队友：滑轨全周期 5.6 s，学成一个模型需要两个以上
      // 完整周期（>11 s）的连续观测，而单机一次回合的视觉窗最长只有 9–10 s，自己凑不满。
      // 四机各广播自己的窗口，任一台都能合到约 30 s 的样本，够折出周期和模板。
      // 还没学成时按 1 Hz 记录一次学习状态，用来区分「窗口不够」和「周期不显著」。
      if(dive_enabled&&!periodic_motion.ready()&&sim_us>last_learn_report+1000000){
        last_learn_report=sim_us;
        emit({{"kind","periodic_learn"},{"sim_time_us",sim_us},{"span_s",periodic_motion.window_span()},{"samples",periodic_motion.sample_count()},{"merged",periodic_motion.merged_count()},{"period_s",periodic_motion.period_estimate()},{"explained",periodic_motion.explained()},{"template_error_m",periodic_motion.template_error()},{"reject",periodic_motion.learn_reject()}});
      }
      if(dive_enabled&&sim_us>last_model_broadcast+500000){
        const auto samples=periodic_motion.exportSamples(192);
        if(!samples.empty()){
          last_model_broadcast=sim_us;
          const auto shared=companion::serializePeriodicSamplesDatagram(samples,o.vehicle);
          for(int i:vehicle_ids)if(i!=o.vehicle)peers.send(shared,17000+i);
          // 广播日志保留周期、解释度与模板误差，供质量核验。
          emit({{"kind","periodic_broadcast"},{"sim_time_us",sim_us},{"samples",samples.size()},{"span_s",periodic_motion.window_span()},{"ready",periodic_motion.ready()},{"period_s",periodic_motion.period_estimate()},{"explained",periodic_motion.explained()},{"template_error_m",periodic_motion.template_error()}});
        }
      }auto control=json::parse(companion::serializeControlDatagram(intention));control["armed"]=flight.armed;control["landed"]=flight.landed;control["heartbeat_fresh"]=flight.heartbeat_fresh;control["landed_fresh"]=flight.landed_fresh;control["native_failsafe_active"]=flight.failsafe_active;control["native_failsafe_latched"]=flight.failsafe_latched;control["heartbeat_gap_latched"]=flight.heartbeat_gap_latched;control["setpoint_gap_latched"]=flight.setpoint_gap_latched;control["offboard_active"]=flight.offboard;control["auto_land_active"]=flight.auto_land;control["imu_bias_ready"]=bias_ready;control["imu_bias_samples"]=bias_samples;emit(control);
      if(target_sys&&live&&e.z_valid)odometry(mav,o.remote,e,reliable_odometry_xy);
      // 9. 常规发送速度参考；碰撞恢复、定位失效保持或着地低推力阶段改发姿态参考。
      if(o.offboard&&target_sys&&live&&!unrecoverable&&!native_feedback.state(Clock::now()).setpoint_gap_latched){
        mavlink_message_t msg;
        const auto disturbance=std::max(last_impact_us,last_collision_us);
        const bool impact_recovery=team_enabled&&flight.armed&&disturbance&&sim_us<disturbance+static_cast<uint64_t>(team_settings.value("impact_recovery_s",0.0)*1e6);
        const bool touchdown=team_enabled&&flight.armed&&team_command.value("phase","")=="LAND_ALL"&&team_settings.contains("touchdown_thrust")&&e.z_valid&&
            e.pose.z_m<team_settings.value("home_ground_z_m",.5)+team_settings.value("body_clearance_m",.016)+.015&&std::abs(e.velocity_enu_mps.z())<.25;
        const bool attitude_hold=team_enabled&&flight.armed&&((!e.xy_valid&&!e.flow_valid)||impact_recovery||touchdown);
        if(attitude_hold){
          const Eigen::Vector3d body_velocity=rotation(0,0,-e.pose.yaw_rad)*e.velocity_enu_mps;
          double recovery_roll=0.0;
          if(impact_recovery)for(const auto&point:team_settings.value("assembly",json::array()))if(point.value("id",-1)==o.vehicle){
            const double heading=team_settings.value("approach",json::object()).value("yaw",0.0);
            const double side=(point.value("y",e.pose.y_m)-e.pose.y_m)*std::cos(heading)-(point.value("x",e.pose.x_m)-e.pose.x_m)*std::sin(heading);
            recovery_roll=-std::copysign(team_settings.value("impact_roll_rad",0.0),side);
          }
          if(impact_recovery&&team_settings.value("contact_exit_side",0.0)!=0)recovery_roll=-std::copysign(team_settings.value("impact_roll_rad",0.0),team_settings.at("contact_exit_side").get<double>());
          const double brake_roll=touchdown?0.0:(impact_recovery?recovery_roll:std::clamp(.6*body_velocity.y()/9.80665,-.06,.06));
          const double brake_pitch=touchdown?0.0:(impact_recovery?team_settings.value("impact_pitch_rad",.2):std::clamp(.6*body_velocity.x()/9.80665,-.06,.06));
          // 直接姿态控制沿用飞控原生 yaw；不能混入已经起点对齐的地图 yaw。
          const double search_yaw=yaw;
          const Eigen::Quaterniond q(rotation(brake_roll,brake_pitch,search_yaw));float quaternion[4]={static_cast<float>(q.w()),static_cast<float>(q.x()),static_cast<float>(q.y()),static_cast<float>(q.z())};
          double zref=team_command.value("target_z_m",e.pose.z_m);
          if(dive_entry_alignment&&!dive_committed&&team_command.value("phase","")=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle)
            zref=dive_entry.z();
          // 0.454545 是模型悬停归一化指令；倾斜时除以 cos 补偿竖直分量，再加高度反馈。
          double thrust=std::clamp(.454545/(std::cos(brake_pitch)*std::cos(brake_roll))+.12*(zref-e.pose.z_m)-.08*e.velocity_enu_mps.z(),.25,.65);
          if(touchdown)thrust=team_settings.at("touchdown_thrust").get<double>();
          const float body_thrust[3]={0,0,0};
          mavlink_msg_set_attitude_target_pack(200,191,&msg,static_cast<uint32_t>(sim_us/1000),target_sys,target_comp,7,quaternion,0,0,0,thrust,body_thrust);
        }else{
          auto v=intention.velocity_flu_mps;
          // 位掩码忽略位置/加速度/yaw，只使用速度和偏航角速度；FLU 的 Y/Z 在此反号。
          mavlink_msg_set_position_target_local_ned_pack(200,191,&msg,static_cast<uint32_t>(sim_us/1000),target_sys,target_comp,MAV_FRAME_BODY_NED,1|2|4|64|128|256|1024,0,0,0,v.x(),-v.y(),-v.z(),0,0,0,0,-intention.yaw_rate_ref_rad_s);
        }
        if(sendMav(mav,msg,o.remote))native_feedback.setpointSent(Clock::now());
      }
      // 10. 命令请求至少间隔 1 s；持续发送参考与请求解锁是不同职责。落地确认后才解除武装。
      if(o.offboard&&team_enabled&&unrecoverable&&flight.armed&&target_sys&&flight.heartbeat_fresh&&(flight.landed||!flight.auto_land)&&std::chrono::duration<double>(now-last_command).count()>1){
        if(flight.landed)command(mav,o.remote,target_sys,target_comp,MAV_CMD_COMPONENT_ARM_DISARM,0,0);
        else command(mav,o.remote,target_sys,target_comp,MAV_CMD_NAV_LAND,0,0);
        emit({{"kind","command_request"},{"command",flight.landed?"DISARM":"LAND"},{"sim_time_us",sim_us},{"reason","unrecoverable"}});
        last_command=now;
      }
      if(o.offboard&&team_enabled&&team_disarm&&flight.armed&&target_sys&&flight.heartbeat_fresh&&flight.landed&&std::chrono::duration<double>(now-last_command).count()>1){command(mav,o.remote,target_sys,target_comp,MAV_CMD_COMPONENT_ARM_DISARM,0,0);emit({{"kind","command_request"},{"command","DISARM"},{"sim_time_us",sim_us},{"reason","team_disarm"}});last_command=now;}
      const bool initial_takeoff=!team_enabled||team_command.value("mission_phase",team_command.value("phase",""))=="TAKEOFF";
      if(o.offboard&&o.arm&&!unrecoverable&&!team_restart_blocked&&(flight.armed||initial_takeoff)&&intention.safe_to_arm&&!team_disarm&&!team_complete&&(!team_enabled||intention.mode!="WAIT_READY")&&age(start)>2&&std::chrono::duration<double>(now-last_command).count()>1){if(!flight.offboard){command(mav,o.remote,target_sys,target_comp,MAV_CMD_DO_SET_MODE,1,6);emit({{"kind","command_request"},{"command","OFFBOARD"},{"sim_time_us",sim_us}});}else if(!flight.armed&&flight.landed){command(mav,o.remote,target_sys,target_comp,MAV_CMD_COMPONENT_ARM_DISARM,1,0);emit({{"kind","command_request"},{"command","ARM"},{"sim_time_us",sim_us}});}last_command=now;}
    }
    // 只在导航发布后启动一帧融合，给计算留出完整的 20 ms 槽位。
    if(navigation_due)if(auto frame=tof_queue.take())
      if(!team_enabled||!sensor_disturbance()||frame->sim_time_us>sensor_disturbance()+400000)
        fusion(estimator.fuseTof(*frame),"tof",frame->sim_time_us);
    std::this_thread::sleep_until(std::min(Clock::now()+std::chrono::milliseconds(2),
        last_publish+std::chrono::milliseconds(20)));
  }
  const auto final_feedback=native_feedback.state(Clock::now());
  emit({{"kind","stop"},{"sim_time_us",sim_us},{"armed_at_stop",final_feedback.armed},{"heartbeat_fresh",final_feedback.heartbeat_fresh},{"landed_fresh",final_feedback.landed_fresh},{"peer_send_drops",peers.send_drops},{"peer_last_errno",peers.last_send_errno},{"mav_send_drops",mav.send_drops},{"mav_last_errno",mav.last_send_errno},
      {"imu_attitude",{{"exact",imu_attitude_counts[0]},{"interpolated",imu_attitude_counts[1]},{"extrapolated",imu_attitude_counts[2]},
          {"missing",imu_attitude_missing},{"attitude_stamp_rejected",attitude_stamp_rejected},{"imu_stamp_rejected",imu_stamp_rejected},
          {"max_extrapolation_us",max_attitude_extrapolation_us},{"max_interpolation_gap_us",max_attitude_interpolation_us}}}});return 0;
}catch(const std::exception&e){std::cerr<<"companion: "<<e.what()<<'\n';return 2;}}


