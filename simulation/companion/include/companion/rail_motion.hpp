#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace companion {
struct RailMotionEstimate {
  double position_m{0},velocity_mps{0};
  uint64_t measurement_us{0};
};

/** @brief 对最近 0.8 s 的独立曝光拟合世界 Y 运动，使用间隔至少 160 ms 的斜率中位数。 */
class RailMotionFit {
 public:
  explicit RailMotionFit(double speed_limit):speed_limit_(speed_limit){
    if(!std::isfinite(speed_limit)||speed_limit<=0)throw std::invalid_argument("invalid rail speed limit");
  }
  bool observe(uint64_t stamp,double position){
    if(!stamp||!std::isfinite(position)||(size_&&stamp<=samples_[size_-1].stamp))return false;
    size_t keep=0;
    for(size_t i=0;i<size_;++i)if(stamp-samples_[i].stamp<=800000)samples_[keep++]=samples_[i];
    size_=keep;
    if(size_==samples_.size()){
      std::move(samples_.begin()+1,samples_.end(),samples_.begin());--size_;
    }
    samples_[size_++]={stamp,position};valid_=false;
    if(size_<4||stamp-samples_[0].stamp<200000)return true;
    std::array<double,276> slopes{};size_t count=0;
    for(size_t i=0;i<size_;++i)for(size_t j=i+1;j<size_;++j){
      const uint64_t dt=samples_[j].stamp-samples_[i].stamp;
      if(dt<160000)continue;
      const double slope=(samples_[j].position-samples_[i].position)/(dt*1e-6);
      if(std::isfinite(slope))slopes[count++]=slope;
    }
    if(!count)return true;
    velocity_=std::clamp(median(slopes.data(),count),-speed_limit_,speed_limit_);
    std::array<double,24> positions{};count=0;
    for(size_t i=0;i<size_;++i){
      const double predicted=samples_[i].position+velocity_*((stamp-samples_[i].stamp)*1e-6);
      if(std::isfinite(predicted))positions[count++]=predicted;
    }
    if(count<4)return true;
    position_=median(positions.data(),count);stamp_=stamp;valid_=true;return true;
  }
  std::optional<RailMotionEstimate> at(uint64_t now)const{
    if(!valid_||now<stamp_||now-stamp_>150000)return std::nullopt;
    return RailMotionEstimate{position_+velocity_*((now-stamp_)*1e-6),velocity_,stamp_};
  }
 private:
  struct Sample{uint64_t stamp;double position;};
  static double median(double* values,size_t size){
    const size_t middle=size/2;
    std::nth_element(values,values+middle,values+size);
    if(size%2)return values[middle];
    return .5*values[middle]+.5*(*std::max_element(values,values+middle));
  }
  std::array<Sample,24> samples_{};
  size_t size_{0};
  double speed_limit_,position_{0},velocity_{0};
  uint64_t stamp_{0};
  bool valid_{false};
};
}
