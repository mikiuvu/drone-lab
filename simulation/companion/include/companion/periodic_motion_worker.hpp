/** @file periodic_motion_worker.hpp
 * @brief 主线程保留观测和已发布模板，后台线程独立执行周期搜索。
 */
#pragma once
#include "companion/periodic_motion.hpp"
#include <memory>
#include <map>

namespace companion {
struct PeriodicLearningStats {
  uint64_t submitted{0}, completed{0}, discarded{0}, failed{0};
  uint64_t publication_rejected{0};
  uint64_t motion_suspensions{0}, motion_resets{0}, last_motion_reset_us{0};
  double last_motion_reset_error_m{0};
  bool last_motion_reset_shared{false};
  double last_ms{0}, max_ms{0};
  int worker_nice{-1};
};

/** @brief observe、merge、service 和 predict 由同一个调用线程使用。
 * @note 后台只操作快照；待处理快照最多一份，搜索时不持有消息锁。
 *       service 不等待后台完成，调用方应按控制周期调用以接收结果。
 */
class PeriodicMotionWorker {
 public:
  explicit PeriodicMotionWorker(bool asynchronous = true);
  ~PeriodicMotionWorker();
  PeriodicMotionWorker(const PeriodicMotionWorker&) = delete;
  PeriodicMotionWorker& operator=(const PeriodicMotionWorker&) = delete;

  void setHorizonCap(double value);
  void setHorizonFraction(double value);
  void setMinSpan(double value);
  /** @brief 收集独立位置观测，同时检查该观测与已发布模型的一致性。 */
  bool observe(uint64_t stamp, const Eigen::Vector3d& point);
  /** @brief 按曝光时间学习原始位置并检查变化；支持延迟到达，同一曝光只学习一次。 */
  bool observeMeasurement(uint64_t stamp,const Eigen::Vector3d& point);
  /** @brief 收集共享样本；service 在一轮接收结束后检查同刻各来源的残差中位数。 */
  std::size_t merge(const std::vector<PeriodicSample>& incoming);
  void service(uint64_t now);
  PeriodicPrediction predict(uint64_t now) const {
    return local_consistency_.blocked||shared_consistency_.blocked?PeriodicPrediction{}:model_.predict(now);
  }
  bool ready() const { return model_.ready()&&!local_consistency_.blocked&&!shared_consistency_.blocked; }
  const std::string& learn_reject() const { return model_.learn_reject(); }
  double period_estimate() const { return model_.period_estimate(); }
  double explained() const { return model_.explained(); }
  double template_error() const { return model_.template_error(); }
  std::size_t sample_count() const { return model_.sample_count(); }
  std::size_t merged_count() const { return model_.merged_count(); }
  double window_span() const { return model_.window_span(); }
  std::vector<PeriodicSample> exportSamples(std::size_t limit) const { return model_.exportSamples(limit); }
  const PeriodicLearningStats& learningStats() const { return stats_; }

 private:
  struct Impl;
  void apply(const PeriodicMotion& fitted, uint64_t now);
  struct Consistency {
    uint64_t checked{0}, bad_since{0};
    unsigned bad{0}, good{0};
    double last_error{0};
    bool blocked{false};
  };
  void checkMotion(uint64_t stamp,double y,Consistency& stream,double minimum_gate,uint64_t measurement_stamp);
  void checkShared();
  Consistency local_consistency_,shared_consistency_;
  std::map<int,PeriodicSample> shared_latest_;
  bool shared_dirty_{false};
  PeriodicMotion model_;
  std::unique_ptr<Impl> impl_;
  PeriodicLearningStats stats_;
  uint64_t generation_{0};
};
}  // namespace companion
