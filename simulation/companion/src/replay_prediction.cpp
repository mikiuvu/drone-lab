/** @brief 按日志时间顺序重放视觉观测，在观察未来帧之前记录预测，避免未来数据泄漏。 */
#include "companion/periodic_motion.hpp"
#include "companion/armor_tracker.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <deque>
#include <cmath>

int main(int argc,char**argv){try{
  if(argc!=3)throw std::runtime_error("replay_prediction companion.jsonl output.json");
  std::ifstream input(argv[1]);if(!input)throw std::runtime_error("input unavailable");
  companion::PeriodicMotion periodic;companion::ArmorTracker track;
  struct Pending{uint64_t target;Eigen::Vector3d point;double confidence;};
  std::deque<Pending> pending;
  size_t frames=0,accepted=0,eligible=0,high=0,matched=0;double sq=0,max_error=0,max_confidence=0;
  uint64_t first=0,last=0;std::string line;
  while(std::getline(input,line)){
    auto j=nlohmann::json::parse(line,nullptr,false);
    if(j.is_discarded()||j.value("kind","")!="visual_pose")continue;
    uint64_t stamp=j.at("sim_time_us").get<uint64_t>();Eigen::Vector3d point(j.at("x_m"),j.at("y_m"),j.at("z_m"));
    if(stamp<=last||!point.allFinite())continue;
    if(!first)first=stamp;last=stamp;++frames;
    while(!pending.empty()&&pending.front().target<=stamp){
      auto p=pending.front();pending.pop_front();
      if(stamp-p.target<=40000){double error=(point-p.point).norm();sq+=error*error;max_error=std::max(max_error,error);++matched;}
    }
    if(!track.observe(stamp,point))continue;
    ++accepted;periodic.observe(stamp,point);
    const auto forecast=periodic.predict(stamp+100000);
    if(forecast.valid){++eligible;max_confidence=std::max(max_confidence,forecast.confidence);}
    if(forecast.valid&&forecast.confidence>=.85){++high;pending.push_back({stamp+100000,forecast.position,forecast.confidence});}
  }
  nlohmann::json out={{"source",argv[1]},{"visual_frames",frames},{"associated_frames",accepted},
    {"span_s",(last-first)*1e-6},{"forecast_horizon_s",.1},{"model_valid_forecasts",eligible},
    {"high_confidence_forecasts",high},{"matched_future_visual_frames",matched},
    {"max_confidence",max_confidence},{"future_visual_rmse_m",matched?nlohmann::json(std::sqrt(sq/matched)):nlohmann::json(nullptr)},
    {"max_future_visual_error_m",matched?nlohmann::json(max_error):nlohmann::json(nullptr)},
    {"evaluation_reference","future visual measurements, not independent physical truth"}};
  std::ofstream output(argv[2]);output<<out.dump(2)<<'\n';if(!output)throw std::runtime_error("output failed");std::cout<<out.dump(2)<<'\n';
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
