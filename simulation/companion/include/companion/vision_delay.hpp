/** @brief 单线程虚拟处理时延：一个在途任务，一个最新等待任务，原曝光时戳由载荷保留。 */
#pragma once
#include <cstdint>
#include <optional>
#include <utility>
namespace companion {
template<class T> class VisionDelay {
 public:
  explicit VisionDelay(uint64_t duration_us):duration_(duration_us){}
  void submit(T payload,uint64_t now){
    ++received;
    if(!active_){active_=std::move(payload);ready_=now+duration_;}
    else {if(waiting_)++dropped;waiting_=std::move(payload);}
  }
  std::optional<T> poll(uint64_t now){
    if(!active_||now<ready_)return std::nullopt;
    auto result=std::move(active_);active_.reset();++completed;
    if(waiting_){active_=std::move(waiting_);waiting_.reset();ready_=now+duration_;}
    return result;
  }
  uint64_t received{0},dropped{0},completed{0};
 private:
  uint64_t duration_,ready_{0};
  std::optional<T> active_,waiting_;
};
}
