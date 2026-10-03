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
#include "companion/periodic_motion.hpp"
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
void sendMav(Udp&socket,mavlink_message_t&msg,int port){uint8_t buf[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(buf,&msg);socket.send(buf,n,port);}
void command(Udp&sock,int port,int sys,int comp,int id,float p1,float p2,float p3=0){mavlink_message_t msg;mavlink_msg_command_long_pack(200,191,&msg,sys,comp,id,0,p1,p2,p3,0,0,0,0);sendMav(sock,msg,port);}
/**
 * @brief 把定位估计转为 MAVLink ODOMETRY，供飞控外部定位融合。
 * @param sock MAVLink 输出套接字。
 * @param port 当前 PX4 接收端口。
 * @param e 世界 ENU 位姿和速度：位置 m、速度 m/s、角度 rad、时间 μs。
 * @note a 交换世界 X/Y 并反转 Z；b 把机体前左上 FLU 转成前右下 FRD。
 * 姿态、速度与协方差都必须同样换坐标，不能只修改位置正负号。
 */
void odometry(Udp&sock,int port,const companion::Estimate&e){
  Eigen::Matrix3d a;a<<0,1,0,1,0,0,0,0,-1;Eigen::Matrix3d b=Eigen::Vector3d(1,-1,-1).asDiagonal();
  Eigen::Matrix3d r=rotation(e.pose.roll_rad,e.pose.pitch_rad,e.pose.yaw_rad);
  const Eigen::Matrix3d frd_rotation=(a*r*b).eval();
  const Eigen::Quaterniond q(frd_rotation);
  const Eigen::Vector3d p=(a*Eigen::Vector3d(e.pose.x_m,e.pose.y_m,e.pose.z_m)).eval();
  const Eigen::Vector3d v=(b*r.transpose()*e.velocity_enu_mps).eval();
  mavlink_odometry_t d{};d.time_usec=e.sim_time_us;d.frame_id=MAV_FRAME_LOCAL_FRD;d.child_frame_id=MAV_FRAME_BODY_FRD;d.x=p.x();d.y=p.y();d.z=p.z();d.q[0]=q.w();d.q[1]=q.x();d.q[2]=q.y();d.q[3]=q.z();d.vx=v.x();d.vy=v.y();d.vz=v.z();d.rollspeed=d.pitchspeed=d.yawspeed=0;d.estimator_type=MAV_ESTIMATOR_TYPE_VISION;d.reset_counter=0;d.quality=e.z_valid?(e.xy_valid?100:80):0;
  Eigen::Matrix<double,6,6>pc=Eigen::Matrix<double,6,6>::Zero(),vc=pc;
  Eigen::Matrix3d velocity_covariance=e.covariance.block<3,3>(4,4);
  pc.block<3,3>(0,0)=a*e.covariance.block<3,3>(0,0)*a.transpose();
  // 水平定位无效时放大 XY 方差，降低飞控融合权重；高度方差仍正常保留。
  if(!e.xy_valid)for(int axis=0;axis<2;++axis){
    pc.row(axis).setZero();pc.col(axis).setZero();pc(axis,axis)=10000.0;
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

  companion::ArmorDetectionConfig detection; detection.enemy_color=visual_contact?companion::ArmorColor::Blue:companion::ArmorColor::AnyBright;
  companion::VisualGuidanceConfig visual_config;
  visual_config.max_forward_speed_mps=team_settings.value("contact_speed_mps",.18);
  visual_config.forward_gain=3.0;visual_config.lateral_gain=2.0;visual_config.vertical_gain=2.0;
  visual_config.yaw_gain=0.0;visual_config.max_observation_age_s=.25;visual_config.max_prediction_s=.3;
  companion::VisualTargetTracker visual_tracker(visual_config);
  companion::VisualGuidanceResult visual_result;
  bool visual_ready=false;
  uint64_t visual_lost_since=0;
  companion::ArmorTracker armor_track;
  companion::PeriodicMotion periodic_motion;
  std::deque<companion::Estimate> pose_history;
  uint64_t visual_pose_stamp=0;

  for(const char* key:{"position_gain","cruise_speed_mps","climb_speed_mps","descent_speed_mps","landing_speed_mps","touchdown_descent_mps","yaw_gain","yaw_rate_limit_rad_s","impact_acceleration_mps2","impact_proximity_m"})
    if(team_settings.contains(key)&&(!team_settings[key].is_number()||!std::isfinite(team_settings[key].get<double>())||team_settings[key].get<double>()<=0))throw std::runtime_error(std::string("invalid team_mission.")+key);
  for(const char* key:{"velocity_damping","impact_settle_s","impact_recovery_s","impact_pitch_rad","impact_roll_rad","touchdown_thrust"})
    if(team_settings.contains(key)&&(!team_settings[key].is_number()||!std::isfinite(team_settings[key].get<double>())||team_settings[key].get<double>()<0))throw std::runtime_error(std::string("invalid team_mission.")+key);
  if(team_settings.value("impact_pitch_rad",0.0)>=companion::kPi/2||team_settings.value("impact_roll_rad",0.0)>=companion::kPi/2||team_settings.value("touchdown_thrust",0.0)>1)throw std::runtime_error("invalid recovery attitude or thrust");
  std::unique_ptr<Udp> team;
  if(team_enabled)team=std::make_unique<Udp>(18000+o.vehicle);
  json team_command;Clock::time_point team_rx{};uint64_t team_sequence=0,impact_seq=0,last_impact_us=0,contact_enter_us=0;
  bool team_disarm=false,team_complete=false,landed=false,landing_recorded=false;



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
  Udp observation(16000+o.vehicle),peers(17000+o.vehicle),mav(o.local);std::map<int,companion::PeerEstimate>peer_cache;
  uint64_t sim_us=0,frame_seq=0,camera_us=0;int target_sys=0,target_comp=0;int waypoint_index=0;bool armed=false,mode_offboard=false;companion::ArmorObservation target;double roll=0,pitch=0,yaw=0;bool attitude=false;Clock::time_point imu_rx{},attitude_rx{},obs_rx{},hb_rx{},bias_start{};uint64_t first_imu=0,first_imu_sim=0,bias_samples=0;Eigen::Vector3d imu_linear_bias=Eigen::Vector3d::Zero();Eigen::Vector3d bias_sum=Eigen::Vector3d::Zero();bool bias_ready=false,yaw_aligned=false;double yaw_offset=0;
  auto start=Clock::now(),last_publish=start,last_heartbeat=start,last_command=start;bool intervals_requested=false;std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  auto fusion=[&](const companion::FusionReport&r,const char*kind,uint64_t stamp){emit({{"kind","fusion"},{"sensor_kind",kind},{"sim_time_us",stamp},{"accepted",r.accepted},{"gated",r.gated},{"stale",r.stale},{"degenerate",r.degenerate},{"inliers",r.inliers},{"reason",r.reason}});};
  // 单线程轮询：各通道每轮限制包数，避免繁忙通道使其他输入长期得不到处理。
  while(running && (o.duration==0||age(start)<o.duration)){
    // 2. 接收协调器目标；只接受递增 sequence，记录进入接触阶段的时刻。
    if(team)for(int n=0;n<32;++n){
      auto packet=team->receive();if(packet.empty())break;
      try{auto cmd=json::parse(packet);
        if(cmd.value("kind","")!="team_command"||cmd.value("vehicle",-1)!=o.vehicle)continue;
        auto sequence=cmd.at("sequence").get<uint64_t>();if(sequence<=team_sequence)continue;
        bool finite=true;for(const char*key:{"target_x_m","target_y_m","target_z_m","target_yaw_rad","contact_speed_mps"})finite=finite&&cmd.contains(key)&&cmd.at(key).is_number()&&std::isfinite(cmd.at(key).get<double>());
        if(!finite)continue;
        if(cmd.value("phase","")=="CONTACT"&&team_command.value("phase","")!="CONTACT")contact_enter_us=cmd.at("sim_time_us").get<uint64_t>();
        team_command=std::move(cmd);team_sequence=sequence;team_rx=Clock::now();
      }catch(const std::exception&){}
    }
    // 3. 测量按 kind 分流。UWB 必须配合邻机估计；仅有距离不能直接得到世界位置。
    // 碰撞后 0.4 s 暂不使用 ToF，避免剧烈姿态变化时错误的结构匹配。
    for(int n=0;n<256;++n){auto s=observation.receive();if(s.empty())break;try{auto j=json::parse(s);uint64_t stamp=j.value("sim_time_us",uint64_t(0));if(j.value("vehicle",-1)!=o.vehicle||!stamp)continue;sim_us=std::max(sim_us,stamp);obs_rx=Clock::now();if(auto flow=companion::parseOpticalFlowDatagram(s,o.vehicle))fusion(estimator.fuseOpticalFlow(*flow),"optical_flow",stamp);else if(auto anchor=companion::parseAnchorRangeDatagram(s,o.vehicle))fusion(estimator.fuseAnchorRange(*anchor),"uwb_anchor",stamp);else if(auto t=companion::parseTofDatagram(s,o.vehicle)){if(!team_enabled||!last_impact_us||stamp>last_impact_us+400000)fusion(estimator.fuseTof(*t),"tof",stamp);}else if(auto h=companion::parseHeightAssistDatagram(s,o.vehicle))fusion(estimator.fuseHeightAssist(*h),"height_assist",stamp);else if(auto u=companion::parseUwbDatagram(s,o.vehicle)){auto p=peer_cache.find(u->peer_id);if(p!=peer_cache.end())fusion(estimator.fuseUwb(*u,p->second),"uwb",stamp);}}catch(const std::exception&){} }
    // 4. 更新邻机缓存；只保存配置机群成员，是否新鲜由融合器再次判断。
    for(int n=0;n<64;++n){auto s=peers.receive();if(s.empty())break;if(auto p=companion::parsePeerEstimateDatagram(s,o.vehicle))if(std::find(vehicle_ids.begin(),vehicle_ids.end(),p->vehicle_id)!=vehicle_ids.end())peer_cache[p->vehicle_id]=*p;}
    // 5. MAVLink 是字节流协议，一包 UDP 可能含多个消息，解析器需跨包保留状态。
    for(int n=0;n<64;++n){auto bytes=mav.receive();if(bytes.empty())break;static mavlink_message_t msg{};static mavlink_status_t status{};for(unsigned char byte:bytes)if(mavlink_parse_char(MAVLINK_COMM_0,byte,&msg,&status)){
      if(msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t h;mavlink_msg_heartbeat_decode(&msg,&h);if(h.autopilot==MAV_AUTOPILOT_PX4){target_sys=msg.sysid;target_comp=msg.compid;armed=h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED;mode_offboard=((h.custom_mode>>16)&255)==6;hb_rx=Clock::now();}}
      if(target_sys==0||msg.sysid!=target_sys)continue;
      if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a;mavlink_msg_attitude_decode(&msg,&a);if(std::isfinite(a.roll)&&std::isfinite(a.pitch)&&std::isfinite(a.yaw)){roll=a.roll;pitch=a.pitch;yaw=a.yaw;attitude=true;attitude_rx=Clock::now();}}
      else if(msg.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE){mavlink_extended_sys_state_t state;mavlink_msg_extended_sys_state_decode(&msg,&state);landed=state.landed_state==MAV_LANDED_STATE_ON_GROUND;}
      else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
        mavlink_highres_imu_t i; mavlink_msg_highres_imu_decode(&msg,&i);
        // PX4 initializes and may reset its compass/yaw during early startup.
        // Do not interpret that estimator reset as physical angular motion.
        if(!sim_us||!attitude||age(attitude_rx)>.25||age(start)<8.0) continue;
        if(!first_imu){estimator.reset(sim_us);first_imu=i.time_usec;first_imu_sim=sim_us;bias_start=Clock::now();}
        // 把飞控 IMU 的时间增量映射到首包对应的仿真时刻，拒绝偏差过大的映射。
        int64_t mapped=static_cast<int64_t>(first_imu_sim)+static_cast<int64_t>(i.time_usec)-static_cast<int64_t>(first_imu);
        if(mapped<=0||std::llabs(mapped-static_cast<int64_t>(sim_us))>500000) continue;
        // 飞控 NED 航向转 ENU：π/2-yaw；起点航向补偿只在首次对齐时确定。
        const double raw_yaw=companion::wrapAngle(companion::kPi/2-yaw);
        if(!yaw_aligned){yaw_offset=companion::wrapAngle(config.initial_pose.yaw_rad-raw_yaw);yaw_aligned=true;}
        const double aligned_yaw=companion::wrapAngle(raw_yaw+yaw_offset);
        companion::ImuSample sample; sample.sim_time_us=static_cast<uint64_t>(mapped); sample.attitude_valid=true;
        sample.roll_flu_rad=roll; sample.pitch_flu_rad=-pitch; sample.yaw_enu_rad=aligned_yaw;
        // 加速度计给的是比力：先 FRD→FLU，再旋到世界系并补重力，得到平移加速度。
        Eigen::Vector3d specific_flu(i.xacc,-i.yacc,-i.zacc);
        const double acceleration_yaw=estimator.estimate().pose.yaw_rad;
        Eigen::Vector3d raw_linear=rotation(roll,-pitch,acceleration_yaw)*specific_flu+Eigen::Vector3d(0,0,-9.80665);
        // 冲击识别：必须是当前撞击机、接近基地、超过阶段切换屏蔽期并出现减速度。
        // 这仍是传感器判断；Gazebo 接触日志由独立评估器核验，未作为这里的输入。
        if(team_enabled&&team_command.value("phase","")=="CONTACT"&&team_command.value("active_vehicle",-1)==o.vehicle&&armed&&sim_us>last_impact_us+1000000&&sim_us>contact_enter_us+150000){
          const auto estimate=estimator.estimate();
          const double tx=team_settings.value("target",json::object()).value("x",0.0),ty=team_settings.value("target",json::object()).value("y",0.0);
          const double heading=team_settings.value("approach",json::object()).value("yaw",0.0);
          const double deceleration=-raw_linear.x()*std::cos(heading)-raw_linear.y()*std::sin(heading);
          const double impulse=team_settings.value("directional_impact",false)?deceleration:raw_linear.head<2>().norm();
          const bool visual_near=visual_result.tracking_valid&&visual_result.range_valid&&visual_result.range_m>0&&visual_result.range_m<.16&&visual_result.observation_age_s<.45;
          const bool close_to_target=visual_contact?visual_near:std::hypot(estimate.pose.x_m-tx,estimate.pose.y_m-ty)<team_settings.value("impact_proximity_m",.35);
          if(visual_contact&&impulse>1.0)emit({{"kind","impact_candidate"},{"sim_time_us",sim_us},{"impulse",impulse},{"range_m",visual_result.range_m},{"age_s",visual_result.observation_age_s},{"tracking",visual_result.tracking_valid}});
          if(close_to_target&&impulse>(visual_contact?8.0:team_settings.value("impact_acceleration_mps2",2.0))){++impact_seq;last_impact_us=sim_us;if(!visual_contact)estimator.fuseMapContact(std::max(sim_us,estimator.estimate().sim_time_us),Eigen::Vector3d::UnitX(),tx-team_settings.value("contact_radius_m",.041));emit({{"kind","impact"},{"sim_time_us",sim_us},{"impact_seq",impact_seq},{"acceleration_mps2",raw_linear.head<2>().norm()},{"normal_deceleration_mps2",deceleration}});}
        }
        // 未解锁静置期间统计加速度偏置；样本数量和经过时间同时满足才认为可用。
        if(!bias_ready&&!armed&&age(bias_start)<2.0&&raw_linear.allFinite()&&raw_linear.norm()<40){
          bias_sum+=raw_linear; ++bias_samples; imu_linear_bias=bias_sum/static_cast<double>(bias_samples);
          if(bias_samples>=80&&age(bias_start)>.8) bias_ready=true;
        }
        sample.acceleration_enu_mps2=raw_linear-(bias_ready?imu_linear_bias:Eigen::Vector3d::Zero());
        sample.acceleration_valid=bias_ready&&(i.fields_updated&7)==7&&sample.acceleration_enu_mps2.allFinite()&&sample.acceleration_enu_mps2.norm()<15;
        sample.pressure_valid=i.fields_updated&(1<<9); sample.absolute_pressure_hpa=i.abs_pressure;
        if(!armed||(team_enabled&&last_impact_us&&sim_us<last_impact_us+400000))sample.acceleration_valid=false;
        estimator.predict(sample); estimator.fuseBaroPressure(sample);
        if(!armed&&std::abs(estimator.estimate().pose.z_m-team_settings.value("home_ground_z_m",config.initial_pose.z_m-.0375)-team_settings.value("body_clearance_m",.0125))<.08)estimator.resetInertialVelocityWhileLanded(sample.sim_time_us);
        imu_rx=Clock::now();
      }      else if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK){mavlink_command_ack_t a;mavlink_msg_command_ack_decode(&msg,&a);emit({{"kind","command_ack"},{"command",a.command},{"result",a.result},{"sim_time_us",sim_us}});}
      // No positions, GPS, simulator states, or PX4 velocity estimates are consumed.
    }}
    // 6. 请求姿态/IMU/落地状态发送间隔，单位 μs；请求值不等于链路保证的实际频率。
    if(target_sys&&!intervals_requested){command(mav,o.remote,target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,MAVLINK_MSG_ID_ATTITUDE,20000);command(mav,o.remote,target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,MAVLINK_MSG_ID_HIGHRES_IMU,10000);command(mav,o.remote,target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,MAVLINK_MSG_ID_EXTENDED_SYS_STATE,100000);intervals_requested=true;}
    auto now=Clock::now();if(std::chrono::duration<double>(now-last_heartbeat).count()>=1){mavlink_message_t msg;mavlink_msg_heartbeat_pack(200,191,&msg,MAV_TYPE_ONBOARD_CONTROLLER,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);sendMav(mav,msg,o.remote);last_heartbeat=now;}
    // 7. 按现实时间至少间隔 20 ms 发布；仿真时间仍取测量时戳，不随电脑运行速度伪造。
    if(std::chrono::duration<double>(now-last_publish).count()>=.02){last_publish=now;estimator.advanceTo(sim_us);auto e=estimator.estimate();
      pose_history.push_back(e);while(pose_history.size()>120)pose_history.pop_front();
      // 相机 PPM 与元数据序号/时间一致才检测；移动装甲接触任务使用蓝色灯条像素闭环。
      auto camera_dir=o.runtime; if(std::filesystem::exists(o.runtime/"camera"))camera_dir=o.runtime/"camera";
      try{auto meta=readJson(camera_dir/("camera_"+std::to_string(o.vehicle)+".json"));auto seq=meta.at("frame_seq").get<uint64_t>();const auto stamp=meta.at("sim_time_us").get<uint64_t>();if(seq!=frame_seq){if(armor_enabled||visual_contact){auto image=companion::readPpm((camera_dir/("camera_"+std::to_string(o.vehicle)+".ppm")).string());if(image && image->frame_seq==seq && image->sim_time_us==stamp){target=companion::detectArmorStrips(*image,camera,detection);frame_seq=seq;camera_us=stamp;emit({{"kind","vision"},{"sim_time_us",camera_us},{"frame_seq",frame_seq},{"pixel_only",true},{"detected",target.valid},{"centroid_x_px",target.centroid_x_px},{"centroid_y_px",target.centroid_y_px},{"angular_error_rad",target.angular_error_rad},{"range_m",target.range_m}});}}else{frame_seq=seq;camera_us=stamp;emit({{"kind","camera_metadata"},{"sim_time_us",camera_us},{"frame_seq",frame_seq},{"detection_skipped",true}});}}}catch(const std::exception&){}
      if(!camera_us||camera_us>sim_us||sim_us-camera_us>camera.timeout_us)target.valid=false;
      if(visual_contact){
        visual_result=visual_tracker.update(sim_us,target,camera_us);
        visual_ready=visual_result.target_valid&&visual_result.range_valid&&std::abs(target.angular_error_rad)<.15&&std::abs(target.vertical_error_rad)<.15;
      }

#ifdef COMPANION_WITH_OPENCV
      if(visual_contact){
        if(camera_us&&camera_us!=visual_pose_stamp&&camera_us<=sim_us){
          auto image=companion::readPpm((camera_dir/("camera_"+std::to_string(o.vehicle)+".ppm")).string());
          if(image&&image->sim_time_us==camera_us){
            visual_pose_stamp=camera_us;
            const auto detected=companion::detectRmArmor(*image,camera,team_settings.value("visual_strip_height_m",.055),team_settings.value("visual_bottom_separation_m",0.0));
            if(detected.valid&&!pose_history.empty()){
              auto nearest=std::min_element(pose_history.begin(),pose_history.end(),[&](const auto&a,const auto&b){return std::llabs(static_cast<int64_t>(a.sim_time_us)-static_cast<int64_t>(camera_us))<std::llabs(static_cast<int64_t>(b.sim_time_us)-static_cast<int64_t>(camera_us));});
              if(std::llabs(static_cast<int64_t>(nearest->sim_time_us)-static_cast<int64_t>(camera_us))<80000){
                const auto&pose=nearest->pose;
                // 相机右下前 → 机体前左上，补相机安装平移，再用曝光时刻自身位姿转世界系。
                Eigen::Vector3d body(detected.position_camera_m.z()+.052,-detected.position_camera_m.x(),-detected.position_camera_m.y());
                Eigen::Vector3d world=Eigen::Vector3d(pose.x_m,pose.y_m,pose.z_m)+rotation(pose.roll_rad,pose.pitch_rad,pose.yaw_rad)*body;
                if(armor_track.observe(camera_us,world))periodic_motion.observe(camera_us,world);
                emit({{"kind","visual_pose"},{"sim_time_us",camera_us},{"pnp",true},{"x_m",world.x()},{"y_m",world.y()},{"z_m",world.z()},{"reprojection_error_px",detected.reprojection_error_px}});
              }
            }
          }
        }
        auto tracked=armor_track.at(sim_us);
        const auto periodic=periodic_motion.predict(sim_us);
        // 周期模型只使用通过关联的视觉观测，且独立周期验证合格才覆盖短时预测。
        if(periodic.valid&&periodic.confidence>=.85){
          tracked.position=periodic.position;tracked.velocity=periodic.velocity;
          emit({{"kind","periodic_prediction"},{"sim_time_us",sim_us},{"confidence",periodic.confidence},{"horizon_s",periodic.horizon},{"y_m",periodic.position.y()}});
        }
        visual_result={};visual_ready=false;
        if(tracked.valid){
          Eigen::Vector3d relative=rotation(0,0,-e.pose.yaw_rad)*(tracked.position-Eigen::Vector3d(e.pose.x_m,e.pose.y_m,e.pose.z_m));
          Eigen::Vector3d feedforward=rotation(0,0,-e.pose.yaw_rad)*tracked.velocity;
          visual_result.tracking_valid=tracked.age_s<=.35||relative.x()<.3;visual_result.target_valid=tracked.measured;visual_result.range_valid=true;
          visual_result.range_m=relative.x();visual_result.observation_age_s=tracked.age_s;
          visual_result.yaw_error_rad=std::atan2(relative.y(),relative.x());
          visual_result.velocity_flu_mps.y()=std::clamp(2.0*relative.y()+feedforward.y(),-.65,.65);
          visual_result.velocity_flu_mps.z()=std::clamp(2.0*relative.z()+feedforward.z(),-.4,.4);
          // 板很近时允许最多0.8s的已锁定轨迹延拓，解决灯条离开视野到机体接触间的盲区。
          const bool aligned=std::abs(relative.y())<.07&&std::abs(relative.z())<.05;
          if(aligned&&visual_result.tracking_valid&&(tracked.measured||relative.x()<.3))visual_result.velocity_flu_mps.x()=team_settings.value("contact_speed_mps",.18);
          visual_ready=tracked.measured&&std::abs(relative.y())<.12&&std::abs(relative.z())<.1;
        }
      }
#endif
      if(armor_enabled&&!visual_contact&&o.vehicle!=o.mission_vehicle)target.valid=false;
      std::vector<companion::PeerEstimate>p;for(auto&entry:peer_cache)p.push_back(entry.second);const bool schedule_active=sim_us>=static_cast<uint64_t>(start_delay_s*1e6);auto intention=companion::makeControlIntention(sim_us,e,target,camera,p,schedule_active?config.safe_waypoints:std::vector<companion::Pose2>{},&waypoint_index,config.waypoint_radius_m,armor_enabled&&schedule_active);if(!schedule_active){intention.mode="HOLD_DELAY";intention.velocity_flu_mps.setZero();intention.target_valid=false;intention.waypoint_valid=false;}
      const bool live=sim_us&&age(obs_rx)<1.0&&age(imu_rx)<.5&&age(hb_rx)<3&&age(attitude_rx)<.5;intention.safe_to_arm=intention.safe_to_arm&&live;
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
        const bool team_ready=live&&e.z_valid&&(e.xy_valid||e.flow_valid);
        intention.safe_to_arm=team_ready&&command_fresh;
        if(command_fresh){
          team_disarm=team_command.value("disarm",false);team_complete=team_command.value("complete",false);
          Eigen::Vector3d delta(team_command.at("target_x_m").get<double>()-e.pose.x_m,team_command.at("target_y_m").get<double>()-e.pose.y_m,team_command.at("target_z_m").get<double>()-e.pose.z_m);
          // v_ref = Kp*(目标位置-估计位置) - Kd*估计速度。阻尼减少高速到点后的摆动。
          Eigen::Vector3d desired=delta*team_settings.value("position_gain",.65);
          desired.head<2>()-=team_settings.value("velocity_damping",0.0)*e.velocity_enu_mps.head<2>();
          double max_speed=team_settings.value("cruise_speed_mps",.35);if(!e.xy_valid)max_speed=std::min(max_speed,.12);
          double horizontal=desired.head<2>().norm();if(horizontal>max_speed)desired.head<2>()*=max_speed/horizontal;
          desired.z()=std::clamp(desired.z(),-team_settings.value("descent_speed_mps",.20),team_settings.value("climb_speed_mps",.35));
          if(intention.mode=="LAND_ALL")desired.z()=landed?0.0:-team_settings.value("landing_speed_mps",.12);
          const bool touchdown_zone=intention.mode=="LAND_ALL"&&e.pose.z_m<team_settings.value("home_ground_z_m",.5)+team_settings.value("body_clearance_m",.016)+.03;
          if(touchdown_zone)desired.head<2>().setZero();
          if(touchdown_zone&&!landed)desired.z()=-team_settings.value("touchdown_descent_mps",team_settings.value("landing_speed_mps",.12));
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
            if(intention.mode=="APPROACH")intention.velocity_flu_mps.x()=0;
            intention.yaw_rate_ref_rad_s=visual_result.yaw_rate_ref_rad_s;
            intention.target_valid=visual_result.target_valid;
            intention.target_range_m=visual_result.range_m;
            intention.target_yaw_error_rad=visual_result.yaw_error_rad;
            if(visual_result.tracking_valid)visual_lost_since=0;
            if(!visual_result.tracking_valid){
              if(intention.mode=="APPROACH"){
                // 未锁定前到达配置搜索点；一旦锁定，仅视觉驱动接近。搜索点不是装甲瞬时位置。
                intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
                const double horizontal=intention.velocity_flu_mps.head<2>().norm();
                if(horizontal>.35)intention.velocity_flu_mps.head<2>()*=.35/horizontal;
              }else {
                if(!visual_lost_since)visual_lost_since=sim_us;
                if(sim_us-visual_lost_since<800000){
                  intention.velocity_flu_mps.setZero();intention.velocity_flu_mps.x()=-.15;
                }else{
                  intention.velocity_flu_mps=rotation(0,0,-e.pose.yaw_rad)*desired;
                  const double speed=intention.velocity_flu_mps.head<2>().norm();
                  if(speed>.3)intention.velocity_flu_mps.head<2>()*=.3/speed;
                }
              }
              intention.yaw_rate_ref_rad_s=0;
            }
          }

          if(intention.mode=="WAIT_READY"||intention.mode=="HOLD"||(last_impact_us&&sim_us<last_impact_us+static_cast<uint64_t>(team_settings.value("impact_settle_s",.4)*1e6))||team_disarm||team_complete){intention.velocity_flu_mps.setZero();intention.yaw_rate_ref_rad_s=0;}
        }
        team->send(json{{"schema",1},{"kind","team_status"},{"vehicle",o.vehicle},{"sim_time_us",sim_us},{"x_m",e.pose.x_m},{"y_m",e.pose.y_m},{"z_m",e.pose.z_m},{"yaw_rad",e.pose.yaw_rad},{"vx_mps",e.velocity_enu_mps.x()},{"vy_mps",e.velocity_enu_mps.y()},{"vz_mps",e.velocity_enu_mps.z()},{"xy_valid",e.xy_valid||e.flow_valid},{"z_valid",e.z_valid},{"ready",team_ready},{"xy_reason",e.localization_mode},{"flow_valid",e.flow_valid},{"visual_ready",visual_ready},{"position_confidence",e.position_confidence},{"armed",armed},{"landed",landed},{"impact_seq",impact_seq}}.dump(),18100);
      }
      if(team_enabled&&landed&&!landing_recorded&&(team_command.value("phase","")=="LAND_ALL"||team_complete)){emit({{"kind","contact_final"},{"sim_time_us",sim_us},{"landed",true},{"phase","LANDED"}});landing_recorded=true;}
      if(!intention.safe_to_arm){intention.mode=armed?"RECOVER_ODOMETRY":"WAIT_SENSORS";intention.velocity_flu_mps.setZero();intention.yaw_rate_ref_rad_s=0;}
      auto message=companion::serializeEstimateDatagram(e);for(int i:vehicle_ids)if(i!=o.vehicle)peers.send(message,17000+i);log<<message<<'\n';auto control=json::parse(companion::serializeControlDatagram(intention));control["armed"]=armed;control["offboard_active"]=mode_offboard;control["imu_bias_ready"]=bias_ready;control["imu_bias_samples"]=bias_samples;emit(control);
      if(target_sys&&live&&e.z_valid)odometry(mav,o.remote,e);
      // 9. 常规发送速度参考；碰撞恢复、定位失效保持或着地低推力阶段改发姿态参考。
      if(o.offboard&&target_sys&&live){
        mavlink_message_t msg;
        const bool impact_recovery=team_enabled&&armed&&last_impact_us&&sim_us<last_impact_us+static_cast<uint64_t>(team_settings.value("impact_recovery_s",0.0)*1e6);
        const bool touchdown=team_enabled&&armed&&team_command.value("phase","")=="LAND_ALL"&&team_settings.contains("touchdown_thrust")&&e.z_valid&&
            e.pose.z_m<team_settings.value("home_ground_z_m",.5)+team_settings.value("body_clearance_m",.016)+.015&&std::abs(e.velocity_enu_mps.z())<.25;
        const bool attitude_hold=team_enabled&&armed&&((!e.xy_valid&&!e.flow_valid)||impact_recovery||touchdown);
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
        sendMav(mav,msg,o.remote);
      }
      // 10. 命令请求至少间隔 1 s；持续发送参考与请求解锁是不同职责。落地确认后才解除武装。
      if(team_enabled&&team_disarm&&armed&&std::chrono::duration<double>(now-last_command).count()>1){command(mav,o.remote,target_sys,target_comp,MAV_CMD_COMPONENT_ARM_DISARM,0,0);last_command=now;}
      if(o.offboard&&o.arm&&intention.safe_to_arm&&!team_disarm&&!team_complete&&(!team_enabled||intention.mode!="WAIT_READY")&&age(start)>2&&std::chrono::duration<double>(now-last_command).count()>1){if(!mode_offboard){command(mav,o.remote,target_sys,target_comp,MAV_CMD_DO_SET_MODE,1,6);emit({{"kind","command_request"},{"command","OFFBOARD"},{"sim_time_us",sim_us}});}else if(!armed){command(mav,o.remote,target_sys,target_comp,MAV_CMD_COMPONENT_ARM_DISARM,1,0);emit({{"kind","command_request"},{"command","ARM"},{"sim_time_us",sim_us}});}last_command=now;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  emit({{"kind","stop"},{"sim_time_us",sim_us},{"armed_at_stop",armed},{"peer_send_drops",peers.send_drops},{"peer_last_errno",peers.last_send_errno},{"mav_send_drops",mav.send_drops},{"mav_last_errno",mav.last_send_errno}});return 0;
}catch(const std::exception&e){std::cerr<<"companion: "<<e.what()<<'\n';return 2;}}


