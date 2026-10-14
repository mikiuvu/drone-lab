#include "companion/periodic_motion_worker.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#ifdef __linux__
#include <sys/resource.h>
#endif

namespace companion {
struct PeriodicMotionWorker::Impl {
  struct Job { PeriodicMotion model; uint64_t generation; };
  struct Result { std::unique_ptr<Job> job; double milliseconds; bool failed; };
  std::mutex mutex;
  std::condition_variable wake;
  std::unique_ptr<Job> pending;
  std::unique_ptr<Result> result;
  bool stopping{false}, busy{false};
  std::atomic<int> nice{-1};
  std::thread thread;

  Impl() : thread([this] { run(); }) {}
  ~Impl() {
    { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
    wake.notify_one();
    thread.join();
  }
  void run() {
#ifdef __linux__
    // Linux 的 PRIO_PROCESS/0 指向当前线程，控制线程的 nice 值保持不变。
    if (setpriority(PRIO_PROCESS, 0, 10) == 0) nice.store(10);
#endif
    for (;;) {
      std::unique_ptr<Job> job;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [&] { return stopping || pending; });
        if (stopping) return;
        job = std::move(pending);
      }
      const auto begin = std::chrono::steady_clock::now();
      bool failed = false;
      try { job->model.learn(job->model.last_stamp_); }
      catch (...) { failed = true; }
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count();
      auto completed = std::make_unique<Result>(Result{std::move(job), elapsed, failed});
      { std::lock_guard<std::mutex> lock(mutex); result = std::move(completed); }
    }
  }
};

PeriodicMotionWorker::PeriodicMotionWorker(bool asynchronous) {
  if (asynchronous) {
    model_.automatic_learning_ = false;
    impl_ = std::make_unique<Impl>();
  }
}
PeriodicMotionWorker::~PeriodicMotionWorker() = default;

void PeriodicMotionWorker::setHorizonCap(double value) { model_.setHorizonCap(value); ++generation_; }
void PeriodicMotionWorker::setHorizonFraction(double value) { model_.setHorizonFraction(value); ++generation_; }
void PeriodicMotionWorker::setMinSpan(double value) { model_.setMinSpan(value); ++generation_; }
bool PeriodicMotionWorker::observe(uint64_t stamp, const Eigen::Vector3d& point) {
  const bool accepted=model_.observe(stamp,point);
  if(accepted)checkMotion(stamp,point.y(),local_consistency_,.12,stamp);
  return accepted;
}
bool PeriodicMotionWorker::observeMeasurement(uint64_t stamp,const Eigen::Vector3d& point) {
  if(stamp<stats_.last_motion_reset_us)return false;
  const bool accepted=model_.observeDelayed(stamp,point);
  if(accepted)checkMotion(stamp,point.y(),local_consistency_,.20,stamp);
  return accepted;
}
std::size_t PeriodicMotionWorker::merge(const std::vector<PeriodicSample>& incoming) {
  // 广播携带整段历史，重置前的数据不能在下一包中重新进入学习窗口。
  const auto first=std::lower_bound(incoming.begin(),incoming.end(),stats_.last_motion_reset_us,
      [](const PeriodicSample& sample,uint64_t time){return sample.stamp<time;});
  const auto added=first==incoming.begin()?model_.merge(incoming):
      model_.merge(std::vector<PeriodicSample>(first,incoming.end()));
  for(auto it=first;it!=incoming.end();++it){
    const auto& sample=*it;
    if(!sample.stamp||!std::isfinite(sample.y)||sample.source< -1||sample.source>255)continue;
    auto& latest=shared_latest_[sample.source];
    if(sample.stamp>latest.stamp){latest=sample;shared_dirty_=true;}
  }
  return added;
}

void PeriodicMotionWorker::checkShared(){
  if(!shared_dirty_)return;
  shared_dirty_=false;
  if(!model_.learn_ok_)return;
  uint64_t newest=0;
  for(const auto& [source,sample]:shared_latest_)newest=std::max(newest,sample.stamp);
  if(newest<=shared_consistency_.checked||newest<=model_.learn_ok_)return;
  std::vector<double> errors;errors.reserve(shared_latest_.size());
  for(const auto& [source,sample]:shared_latest_){
    // 相位突变后，较早的正常观测不能证明当前仍正常；只合并同刻的独立来源。
    if(sample.stamp==newest)
      errors.push_back(sample.y-model_.replay(sample.stamp).position.y());
  }
  std::sort(errors.begin(),errors.end());
  // 偶数来源取中位区间内最接近零的残差；只有严格多数同向超限才触发变化。
  const double low=errors[(errors.size()-1)/2],high=errors[errors.size()/2];
  const double error=low>0?low:high<0?high:0;
  checkMotion(newest,model_.replay(newest).position.y()+error,shared_consistency_,.20,newest);
}

void PeriodicMotionWorker::checkMotion(uint64_t stamp,double y,Consistency& stream,double minimum_gate,uint64_t measurement_stamp){
  if(!stamp||!measurement_stamp||measurement_stamp>stamp||measurement_stamp<=stream.checked||!std::isfinite(y))return;
  stream.checked=measurement_stamp;
  // 已发布模板即使暂停预测，也要继续积累运动变化证据，才能切断旧学习窗口。
  if(!model_.learn_ok_||measurement_stamp<=model_.learn_ok_)return;
  const double error=y-model_.replay(stamp).position.y();
  const double gate=std::max(minimum_gate,model_.template_error_*3);
  if(std::abs(error)>gate){
    if(!stream.blocked)++stats_.motion_suspensions;
    stream.blocked=true;stream.good=0;
    if(stream.bad&&error*stream.last_error<0)stream.bad=0;
    if(!stream.bad)stream.bad_since=stamp;
    stream.last_error=error;
    ++stream.bad;
    // 分开检查本机跟踪与队友原始点，避免一个通道的正常点掩盖另一个通道的异常。
    // 三次独立曝光/共享采样的同向异常跨越 40 ms 才切断学习窗口；单点只临时暂停。
    if(stream.bad>=3&&stamp-stream.bad_since>=40000){
      const uint64_t boundary=stream.bad_since;
      while(!model_.samples_.empty()&&model_.samples_.front().stamp<boundary)model_.samples_.pop_front();
      model_.merged_=std::count_if(model_.samples_.begin(),model_.samples_.end(),[](const auto& s){return !s.local;});
      model_.ready_=false;model_.learn_ok_=0;model_.next_learn_=0;
      model_.reject_="运动变化，重新积累观测";
      ++generation_;++stats_.motion_resets;stats_.last_motion_reset_us=boundary;
      stats_.last_motion_reset_error_m=error;
      stats_.last_motion_reset_shared=&stream==&shared_consistency_;
      const auto local_checked=local_consistency_.checked,shared_checked=shared_consistency_.checked;
      local_consistency_={};shared_consistency_={};
      local_consistency_.checked=local_checked;shared_consistency_.checked=shared_checked;
    }
  }else{
    stream.bad=0;stream.bad_since=0;
    if(++stream.good>=2)stream.blocked=false;
  }
}

void PeriodicMotionWorker::apply(const PeriodicMotion& fitted, uint64_t now) {
  model_.next_learn_ = fitted.next_learn_;
  model_.reject_ = fitted.reject_;
  if(fitted.ready_&&fitted.learn_ok_!=model_.learn_ok_){
    // 搜索期间到达的最新观测作为留出集；最多检查最后 256 条，避免发布旧运动模型。
    double squared=0;std::size_t used=0;
    uint64_t later=fitted.last_stamp_;
    bool gap=false;
    for(auto sample=model_.samples_.rbegin();sample!=model_.samples_.rend()&&used<256;++sample){
      if(sample->stamp<=fitted.last_stamp_)break;
      if(later!=fitted.last_stamp_&&PeriodicMotion::seconds(later,sample->stamp)>PeriodicMotion::kMaxGap)gap=true;
      later=sample->stamp;
      const double error=sample->point.y()-fitted.replay(sample->stamp).position.y();
      squared+=error*error;++used;
    }
    const double noise=fitted.template_error_*std::sqrt(std::max(1.0,
        static_cast<double>(fitted.samples_.size())/PeriodicMotion::kBins));
    if(now<fitted.learn_ok_||PeriodicMotion::seconds(now,fitted.learn_ok_)>2*fitted.period_||gap||
        (used>=16&&std::sqrt(squared/used)>std::max(fitted.tolerance_*3,noise*1.8))){
      ++stats_.publication_rejected;model_.ready_=false;
      model_.reject_="后台模板与最新观测不符或已过期";return;
    }
  }
  // 未通过验证的候选只更新诊断，不覆盖仍在有效期内的已发布模板。
  if (fitted.learn_ok_ != model_.learn_ok_) {
    model_.anchor_ = fitted.anchor_; model_.learn_ok_ = fitted.learn_ok_;
    model_.shape_ = fitted.shape_; model_.fixed_ = fitted.fixed_;
    model_.phase_valid_ = fitted.phase_valid_;
    model_.period_ = fitted.period_; model_.tolerance_ = fitted.tolerance_;
    model_.confidence_ = fitted.confidence_; model_.horizon_ = fitted.horizon_;
    model_.explained_ = fitted.explained_; model_.template_error_ = fitted.template_error_;
  }
  model_.ready_ = fitted.ready_ && model_.learn_ok_ && now >= model_.learn_ok_ &&
      PeriodicMotion::seconds(now, model_.learn_ok_) <= 2 * model_.period_;
}

void PeriodicMotionWorker::service(uint64_t now) {
  checkShared();
  if (!impl_) return;
  std::unique_ptr<Impl::Result> completed;
  bool busy;
  {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return;
    completed = std::move(impl_->result);
    if (completed) impl_->busy = false;
    busy = impl_->busy;
  }
  stats_.worker_nice = impl_->nice.load();
  if (completed) {
    ++stats_.completed;
    stats_.last_ms = completed->milliseconds;
    stats_.max_ms = std::max(stats_.max_ms, stats_.last_ms);
    if (completed->failed) {
      ++stats_.failed; model_.reject_ = "周期搜索失败";
      model_.next_learn_ = now + 1000000;
    } else if (completed->job->generation == generation_) apply(completed->job->model, now);
    else ++stats_.discarded;
  }
  if (busy || model_.samples_.empty() || model_.last_stamp_ < model_.next_learn_) return;
  auto job = std::make_unique<Impl::Job>(Impl::Job{model_, generation_});
  {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return;
    impl_->pending = std::move(job); impl_->busy = true; ++stats_.submitted;
  }
  impl_->wake.notify_one();
}
}  // namespace companion
