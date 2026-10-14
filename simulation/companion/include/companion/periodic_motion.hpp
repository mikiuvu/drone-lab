/** @file periodic_motion.hpp
 * @brief 从视觉观测中估计滑轨往复周期，并用验证过的模板外推落点。
 */
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace companion {

/** @brief 一条可跨机共享的滑轨观测。
 * @note 滑轨只沿世界 Y 往复，X/Z 固定，因此只广播 Y 就够了。
 *       Y 是未做任何偏移处理的世界坐标。各机对同一板面的系统偏差实测在 2.6 cm 以内，
 *       折叠时同一相位格会混入多机样本，这点偏差会被摊薄；反过来若各机先按自己的
 *       窗口均值居中再合并，窗口不足整数个周期时均值本身就有几厘米偏差，反而更差。
 */
struct PeriodicSample {
  uint64_t stamp{0};  ///< 仿真时刻，μs；机间共用同一时钟。
  double y{0};        ///< 板面中心的世界 Y，m。
  int source{-1};     ///< 共享数据的原始发送者；-1 表示来源未标识。
};

struct PeriodicPrediction {
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double confidence{0};
  double horizon{0};  ///< 从最新观测起允许外推的总时长，单位 s。
};

/** 时间戳单位为微秒，位置单位为 m。调用方只传入关联已接受、位姿已补偿的视觉位置。
 *
 * 周期用「折叠搜索」估计，不用转向点。滑轨真实周期是 5.6 s（±0.28 m 行程、0.2 m/s，
 * 单程 2.8 s，全周期 5.6 s），而跟踪输出的板面 Y 单帧误差中位就有 0.08–0.1 m、
 * 重尾到 0.4 m，比判定一次转向所需的越程大一个量级；靠跟踪极值点找转向必然把噪声
 * 当成转向（实测 50 Hz 输入下学出的「周期」是 0.07–0.84 s）。折叠只看整条波形的
 * 形状，对噪声幅度不敏感，并且每格的统计量改用中位数以抗离群点。
 *
 * 学成一个模型需要覆盖至少两个完整周期（默认 12 s）的连续观测。单机一次回合里视觉
 * 连续有效的窗口最长只有 9–10 s，谁都凑不满，所以本机窗口不足时用 merge() 合入队友
 * 广播的样本——这正是「跨机共享」要解决的问题，共享的不是学成的模型（没人能学成），
 * 而是原始观测。
 */
class PeriodicMotion {
 public:
  /** @brief 设置允许的外推时长上限，单位 s。
   *  @param horizon 必须为正；默认 0.2 s，与旧的短预测窗行为一致。
   *  @note 学习与验证过程不变，只放宽外推窗。俯冲撞击需要提前约 0.5 s 得到落点，
   *        因此由任务配置放宽；滑轨是恒定速度往复，验证过的模板外推仍然可信。
   */
  void setHorizonCap(double horizon) {
    if (std::isfinite(horizon) && horizon > 0) horizon_cap_ = horizon;
  }

  /** @brief 设置外推窗相对周期长度的上限比例。
   *  @param fraction 必须为正；默认 0.12，即最多外推 12% 个周期。
   *  @note 俯冲的落点提前量约占滑轨周期 18%，短比例会先把窗口截断，
   *        所以除绝对上限外还要能放宽这个比例。
   */
  void setHorizonFraction(double fraction) {
    if (std::isfinite(fraction) && fraction > 0) horizon_fraction_ = fraction;
  }

  /** @brief 设置学成一个模型所需的最短连续观测窗，单位 s。
   *  @param seconds 必须不小于 2；默认 12 s，即覆盖滑轨两个完整周期（5.6 s）还留一点余量。
   *  @note 不足两个周期就折不出重复结构。单机窗口只有 9–10 s，必须靠 merge()
   *        合入队友样本才够，所以这个门限同时也是「共享是否真的起作用」的判据。
   */
  void setMinSpan(double seconds) {
    if (std::isfinite(seconds) && seconds >= 2) min_span_ = seconds;
  }

  /** @brief 本机是否已有可用模型。
   *  @return true 表示 predict() 能返回有效结果。
   */
  bool ready() const { return ready_; }

  /** @brief 最近一次学习尝试的失败原因，学成后为空串。
   *  @return 简短中文原因；从未尝试过时返回 "尚未触发学习"。
   *  @note 只用于诊断日志：现场只能看到「预测一直没生效」，没有这个就没法区分是
   *        观测窗不够、周期不显著，还是模板残差过大。
   */
  const std::string& learn_reject() const { return reject_; }
  /** @brief 最近一次算出的周期估计，单位 s；未算过为 0。 */
  double period_estimate() const { return period_; }
  /** @brief 最近一次折叠搜索中，模板解释掉的方差占比，0–1。 */
  double explained() const { return explained_; }
  /** @brief 最近一次模板的估计误差（每格中位数均值的标准误），单位 m。 */
  double template_error() const { return template_error_; }
  /** @brief 当前缓冲的样本总数，含合入的队友样本。 */
  std::size_t sample_count() const { return samples_.size(); }
  /** @brief 当前缓冲区里来自队友的样本数。 */
  std::size_t merged_count() const { return merged_; }
  /** @brief 当前缓冲窗口的时间跨度，单位 s。 */
  double window_span() const {
    return samples_.size() < 2 ? 0 : seconds(samples_.back().stamp, samples_.front().stamp);
  }

  /** @brief 写入一条本机观测。
   *  @param stamp 仿真时刻，μs；必须严格递增，否则丢弃。
   *  @param point 板面中心的世界位置，m。
   *  @return 是否接受。
   */
  bool observe(uint64_t stamp, const Eigen::Vector3d& point) {
    if (!stamp || stamp <= last_stamp_ || !point.allFinite()) return false;
    last_stamp_ = stamp;
    samples_.push_back(Sample{stamp, point, true});
    prune();
    // 连续两个周期都没能重学成功，说明模板已经跟不上实际运动，先停用；
    // 运动恢复后会在后续窗口上重新学成。
    if (ready_ && learn_ok_ && stamp > learn_ok_ + static_cast<uint64_t>(2 * period_ * 1e6))
      ready_ = false;
    // 未学成时按固定间隔重试；学成后每半个周期用滑窗重学一次，跟上周期漂移。
    if (automatic_learning_ && stamp >= next_learn_) learn(stamp);
    return true;
  }

  /** @brief 按曝光时间插入本机测量，允许其晚于更近的共享数据到达。
   * @note 重复本机曝光与窗口外的数据丢弃；同刻共享点由本机点替换，不回退最新时钟。
   */
  bool observeDelayed(uint64_t stamp,const Eigen::Vector3d& point){
    if(!stamp||!point.allFinite())return false;
    if(stamp>last_stamp_)return observe(stamp,point);
    if(seconds(last_stamp_,stamp)>kMaxHistoryS)return false;
    const auto at=std::lower_bound(samples_.begin(),samples_.end(),stamp,
        [](const Sample& sample,uint64_t time){return sample.stamp<time;});
    if(at!=samples_.end()&&at->stamp==stamp){
      if(at->local)return false;
      *at=Sample{stamp,point,true};--merged_;
    }else samples_.insert(at,Sample{stamp,point,true});
    prune();
    if(automatic_learning_&&last_stamp_>=next_learn_)learn(last_stamp_);
    return true;
  }

  /** @brief 合入队友广播的观测样本。
   *  @param incoming 队友导出的样本，必须按时刻升序（exportSamples 保证）。
   *  @return 实际新增的样本数；同刻已有样本时保留本机的。
   *  @note 合入的样本只参与周期估计与模板，不参与 fixed_（那是本机自己的观测均值）。
   */
  std::size_t merge(const std::vector<PeriodicSample>& incoming) {
    if (incoming.empty()) return 0;
    std::deque<Sample> merged;
    merged.swap(samples_);
    samples_.clear();
    auto left = merged.begin();
    auto right = incoming.begin();
    std::size_t added = 0;
    while (left != merged.end() || right != incoming.end()) {
      const bool take_left = right == incoming.end() ||
          (left != merged.end() && left->stamp < right->stamp);
      if (take_left) {
        samples_.push_back(*left++);
      } else if (left != merged.end() && left->stamp == right->stamp) {
        samples_.push_back(*left++);  // 同一时刻以本机样本为准。
        ++right;
      } else {
        if (right->stamp && std::isfinite(right->y)) {
          samples_.push_back(Sample{right->stamp, Eigen::Vector3d(0, right->y, 0), false});
          ++added;
        }
        ++right;
      }
    }
    merged_ += added;
    prune();
    // 队友的样本同样是「对这块板面的观测」，外推窗要从最新的那条观测起算，
    // 否则本机跟踪一旦中断，明明还在收到队友数据却会被判成模型过期。
    // 归并只增不删，这里取到的一定是缓冲里最新的时刻。
    if (!samples_.empty() && samples_.back().stamp > last_stamp_)
      last_stamp_ = samples_.back().stamp;
    // 合入本身就可能把窗口撑到门限以上，因此这里也要触发一次重学：
    // 实测中「学成」的时刻往往正是某台队友的样本刚到齐的那一刻。
    if (automatic_learning_ && !samples_.empty() && samples_.back().stamp >= next_learn_)
      learn(samples_.back().stamp);
    return added;
  }

  /** @brief 导出本机观测供队友合并。
   *  @param limit 最多导出的样本数，超出时按等间隔抽取。
   *  @return 按时刻升序的样本；本机样本不足时返回空表。
   *  @note 只导出本机样本，不含已合入的队友样本：转发会形成放大回路，而每个队友
   *        各自广播自己的窗口已经够用——四机各 9 s 窗口，任一台都能收到约 27 s。
   */
  std::vector<PeriodicSample> exportSamples(std::size_t limit) const {
    std::vector<PeriodicSample> result;
    if (limit < 2) return result;
    std::vector<const Sample*> local;
    for (const auto& sample : samples_)
      if (sample.local) local.push_back(&sample);
    if (local.size() < 32) return result;
    const auto count=std::min(limit,local.size());result.reserve(count);
    for(std::size_t i=0;i<count;++i){
      const auto index=i*(local.size()-1)/(count-1);
      result.push_back(PeriodicSample{local[index]->stamp,local[index]->point.y()});
    }
    return result;
  }

  /** 有效时间窗内重放周期，X/Z 保持本机观测均值，速度仅含 Y 分量。
   * 不允许查询早于最新曝光的时刻；缺测超过窗口时立即返回默认无效结果。
   */
  PeriodicPrediction predict(uint64_t now) const {
    if (!ready_ || now < last_stamp_) return {};
    if (learn_ok_ && now > learn_ok_ && seconds(now, learn_ok_) > 2 * period_) return {};
    const double age = seconds(now, last_stamp_);
    if (age > horizon_) return {};
    const auto bin=std::min(kBins-1,static_cast<std::size_t>(phaseOf(now)*kBins));
    if(!phase_valid_[bin]||!phase_valid_[bin+1])return {};
    auto result = replay(now);
    result.valid = true;
    result.confidence = confidence_ * (1 - .15 * age / horizon_);
    result.horizon = horizon_;
    return result;
  }

 private:
  friend class PeriodicMotionWorker;
  struct Sample {
    uint64_t stamp;
    Eigen::Vector3d point;
    bool local;  ///< true 表示本机观测；合入的队友样本不参与 fixed_。
  };
  static constexpr double kMaxGap = .25;        ///< 超过此间隔即认为观测断档，单位 s。
  static constexpr double kMaxHistoryS = 60;    ///< 缓冲窗口上限，单位 s。
  static constexpr std::size_t kMaxSamples = 8192;
  static constexpr std::size_t kMinSamples = 200;
  static constexpr double kLearnRetryS = 1.0;   ///< 未学成时的重试间隔，单位 s。
  static constexpr double kMinPeriod = .8;      ///< 周期搜索范围，单位 s。
  static constexpr double kMaxPeriod = 10.0;
  static constexpr double kCoarseStep = .02;
  static constexpr double kFineStep = .0025;
  static constexpr std::size_t kSearchBins = 48;
  static constexpr std::size_t kMinPerBin = 2;
  static constexpr double kMinExplained = .6;   ///< 折叠至少要解释掉这么多方差。
  static constexpr double kHarmonicRatio = .9;  ///< 整数分频后仍能解释这么多才算谐波歧义。
  static constexpr double kMinCycles = 2.0;     ///< 窗口至少要覆盖这么多个候选周期。
  /** 模板误差到置信度的换算尺度，单位 m。
   * @note 取 0.12 是按实测定的：跟踪输出的板面 Y 单帧噪声约 130 mm，按每格 15 个
   *       跨周期样本平均后，模板误差约 34 mm，对应置信度约 0.72，刚好越过俯冲的
   *       predict_confidence 门限（0.7）。压力面半宽只有 67 mm，模板误差再翻一倍就
   *       没得瞄了，置信度会掉到 0.5 以下并被门限挡掉。
   */
  static constexpr double kConfidenceScaleM = .12;
  /** 模板误差的硬上限，单位 m；再差连外推窗都不该开。 */
  static constexpr double kMaxTemplateError = .06;
  static constexpr std::size_t kBins = 128;     ///< 周期内的模板采样段数。

  static double seconds(uint64_t later, uint64_t earlier) {
    return static_cast<double>(later - earlier) * 1e-6;
  }

  /** @brief 相位，0–1，以 anchor_ 为零点。 */
  double phaseOf(uint64_t stamp) const {
    const double elapsed = stamp >= anchor_ ? seconds(stamp, anchor_) : -seconds(anchor_, stamp);
    double phase = std::fmod(elapsed / period_, 1.0);
    if (phase < 0) phase += 1;
    return phase;
  }

  static double medianOf(std::vector<double>& values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double high = values[middle];
    if (values.size() % 2) return high;
    std::nth_element(values.begin(), values.begin() + middle - 1, values.begin() + middle);
    return .5 * (high + values[middle - 1]);
  }

  /** @brief 把窗口样本按相位折成 bins 格，返回格内残差均方根。
   *  @return 残差；任何一格样本数不足 min_per_bin 时返回负值表示覆盖不够。
   *  @note 每格用中位数而不是均值：跟踪输出的离群点重尾，均值会被单点拽偏。
   */
  double foldResidual(std::size_t start, double period, const std::vector<double>& elapsed,
                      std::array<std::vector<double>, kSearchBins>& buckets) const {
    for (auto& bucket : buckets) bucket.clear();
    for (std::size_t i = start; i < samples_.size(); ++i) {
      const double cycles = elapsed[i - start] / period;
      // 已排序窗口的相对时间非负，减去整周期与 fmod(cycles, 1) 等价。
      const double phase = cycles - std::floor(cycles);
      const auto bin = std::min(kSearchBins - 1, static_cast<std::size_t>(phase * kSearchBins));
      buckets[bin].push_back(samples_[i].point.y());
    }
    double total = 0;
    std::size_t used = 0;
    for (auto& bucket : buckets) {
      if (bucket.size() < kMinPerBin) return -1;
      const double center = medianOf(bucket);
      for (double value : bucket) {
        const double delta = value - center;
        total += delta * delta;
      }
      used += bucket.size();
    }
    return used ? std::sqrt(total / used) : -1;
  }

  void prune() {
    while (samples_.size() > kMaxSamples ||
           (samples_.size() > 2 &&
            seconds(samples_.back().stamp, samples_.front().stamp) > kMaxHistoryS)){
      if(!samples_.front().local)--merged_;
      samples_.pop_front();
    }
  }

  Eigen::Vector3d interpolate(uint64_t stamp) const {
    auto right = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const Sample& sample, uint64_t time) { return sample.stamp < time; });
    if (right == samples_.begin()) return right->point;
    if (right == samples_.end()) return samples_.back().point;
    const auto left = std::prev(right);
    const double ratio = seconds(stamp, left->stamp) / seconds(right->stamp, left->stamp);
    return left->point + ratio * (right->point - left->point);
  }

  PeriodicPrediction replay(uint64_t stamp) const {
    const double phase = phaseOf(stamp);
    const double bin = phase * kBins;
    const auto index = std::min(kBins - 1, static_cast<std::size_t>(bin));
    const double ratio = bin - index;
    PeriodicPrediction result;
    result.position = fixed_;
    result.position.y() = shape_[index] + ratio * (shape_[index + 1] - shape_[index]);
    result.velocity.y() = (shape_[index + 1] - shape_[index]) * kBins / period_;
    return result;
  }

  /** 折叠搜索定周期，再按相位分格取中位数做模板，最后用时间上留出的后段验证。
   * @param stamp 触发本次学习的最新样本时刻，用来排下一次重试。 */
  void learn(uint64_t stamp) {
    next_learn_ = stamp + static_cast<uint64_t>(
        (ready_ ? std::max(.5, period_ * .5) : kLearnRetryS) * 1e6);
    // 断档会把两段互不相干的观测折到一起，只取最近一次大断档之后的样本。
    std::size_t start = 0;
    for (std::size_t i = 1; i < samples_.size(); ++i)
      if (seconds(samples_[i].stamp, samples_[i - 1].stamp) > kMaxGap) start = i;
    if (samples_.size() - start < kMinSamples) { reject_ = "样本不足"; return; }
    const double span = seconds(samples_.back().stamp, samples_[start].stamp);
    if (span < min_span_) { reject_ = "观测窗太短"; return; }

    // 全窗的稳健散布，作为「折叠解释掉多少」的分母。
    std::vector<double> values;
    values.reserve(samples_.size() - start);
    for (std::size_t i = start; i < samples_.size(); ++i) values.push_back(samples_[i].point.y());
    const double center = medianOf(values);
    double spread = 0;
    for (double value : values) spread += (value - center) * (value - center);
    spread = std::sqrt(spread / values.size());
    if (spread < .02) { reject_ = "板面几乎不动"; ready_ = false; return; }

    // 粗搜：能解释掉的方差最多者最像真周期。搜索上界受窗口长度限制——候选周期接近
    // 窗口长度时，每个相位格只装得下时间上紧挨着的样本，格内残差被「信号本来就平滑」
    // 而不是「跨周期可重复」压小，于是长周期一律假性吻合（实测板面停住后能学出一个
    // 6.14 s 的周期）。要求窗口至少覆盖两个候选周期，才能保证每格被跨周期地访问两次。
    const double upper = std::min(kMaxPeriod, span / kMinCycles);
    std::vector<double> elapsed;
    elapsed.reserve(samples_.size() - start);
    for (std::size_t i = start; i < samples_.size(); ++i)
      elapsed.push_back(seconds(samples_[i].stamp, samples_[start].stamp));
    // 候选周期共用分桶容量和相对时间，减少每次搜索的分配与转换。
    std::array<std::vector<double>, kSearchBins> search_buckets;
    for (auto& bucket : search_buckets) bucket.reserve(elapsed.size() / kSearchBins * 2 + 8);
    double best = -1, best_period = 0;
    for (double candidate = kMinPeriod; candidate <= upper; candidate += kCoarseStep) {
      const double residual = foldResidual(start, candidate, elapsed, search_buckets);
      if (residual < 0) continue;
      const double explained = 1 - residual * residual / (spread * spread);
      if (explained > best) { best = explained; best_period = candidate; }
    }
    if (best < kMinExplained || best_period <= 0) {
      reject_ = "周期不显著";
      ready_ = false;
      return;
    }
    // 真周期的整数倍同样能把波形折到重合，所以要往下找基本周期：从最优候选出发依次
    // 试 1/2、1/3、1/4，取仍然折叠得同样好的最小者。不能简单地「取解释度接近最优的
    // 最小周期」——真周期附近是一片平台，那样会把周期系统性地取小（实测 5.6 s 会被
    // 取成 5.3 s），只有整数分频才是真正的谐波歧义。
    double period = best_period;
    for (std::size_t divisor = 2; divisor <= 4; ++divisor) {
      const double candidate = best_period / static_cast<double>(divisor);
      if (candidate < kMinPeriod) break;
      const double residual = foldResidual(start, candidate, elapsed, search_buckets);
      if (residual < 0) continue;
      if (1 - residual * residual / (spread * spread) >= kHarmonicRatio * best) {
        period = candidate;
        break;
      }
    }

    // 粗搜的步长带来最多半个步长的相位误差，在基本周期附近细搜一次。
    double refined = period, refined_score = -1;
    for (double candidate = std::max(kMinPeriod, period - kCoarseStep);
         candidate <= std::min(kMaxPeriod, period + kCoarseStep);
         candidate += kFineStep) {
      const double residual = foldResidual(start, candidate, elapsed, search_buckets);
      if (residual < 0) continue;
      const double explained = 1 - residual * residual / (spread * spread);
      if (explained > refined_score) { refined_score = explained; refined = candidate; }
    }
    period_ = refined;
    anchor_ = samples_[start].stamp;

    // 模板：每格取中位数，抗离群点。合入的队友样本按 2 Hz、等间隔抽取后到达，
    // 相位格会稀密不均，空格由相邻格线性插值补上；空格太多说明覆盖不够，放弃。
    std::vector<std::vector<double>> buckets(kBins);
    Eigen::Vector3d fixed = Eigen::Vector3d::Zero();
    std::size_t local = 0;
    for (std::size_t i = start; i < samples_.size(); ++i) {
      const auto bin = std::min(kBins - 1, static_cast<std::size_t>(phaseOf(samples_[i].stamp) * kBins));
      buckets[bin].push_back(samples_[i].point.y());
      if (samples_[i].local) { fixed += samples_[i].point; ++local; }
    }
    std::vector<double> centers(kBins, 0);
    std::vector<char> filled(kBins, 0);
    std::size_t missing = 0;
    for (std::size_t bin = 0; bin < kBins; ++bin) {
      if (buckets[bin].empty()) { ++missing; continue; }
      centers[bin] = medianOf(buckets[bin]);
      filled[bin] = 1;
    }
    // 空格上限取三分之一：模板分辨率是 128 格，若单周期内的采样相位不足 128 个
    // （例如 100 Hz 喂 1 s 周期只有 100 个相位），必然有一批格子收不到样本。
    // 滑轨是分段线性的三角波，相邻格线性插值本身就能补准，所以放宽而不是放弃。
    if (missing == kBins || missing > kBins / 3) { reject_ = "相位覆盖不足"; return; }
    for (std::size_t bin = 0; bin < kBins; ++bin) {
      if (filled[bin]) { shape_[bin] = centers[bin]; continue; }
      std::size_t back = 1;
      while (back < kBins && !filled[(bin + kBins - back) % kBins]) ++back;
      std::size_t forward = 1;
      while (forward < kBins && !filled[(bin + forward) % kBins]) ++forward;
      const double low = centers[(bin + kBins - back) % kBins];
      const double high = centers[(bin + forward) % kBins];
      shape_[bin] = low + (high - low) * static_cast<double>(back) /
                              static_cast<double>(back + forward);
    }
    shape_[kBins] = shape_[0];
    // 全局残差会稀释少数相位格的尖峰；用时间跨度折算局部速度上限。
    // 上限至少 1 m/s，并留出全模板平均往复速度的四倍余量。
    const auto shape_bounds=std::minmax_element(shape_.begin(),shape_.end());
    const double speed_limit=std::max(1.0,8*(*shape_bounds.second-*shape_bounds.first)/period_);
    phase_valid_.fill(true);
    for(std::size_t bin=0;bin<kBins;++bin){
      const auto left=(bin+kBins-1)%kBins,right=(bin+1)%kBins;
      const double neighbor=.5*(shape_[left]+shape_[right]);
      if(std::abs(shape_[bin]-neighbor)>speed_limit*period_/kBins){
        phase_valid_[left]=phase_valid_[bin]=phase_valid_[right]=false;
      }
    }
    phase_valid_[kBins]=phase_valid_[0];
    if (local) fixed_ = fixed / static_cast<double>(local);
    else fixed_.setZero();
    const auto bounds = std::minmax_element(shape_.begin(), shape_.end());
    const double shape_span = *bounds.second - *bounds.first;
    if (shape_span < .05) { reject_ = "板面几乎不动"; ready_ = false; return; }
    tolerance_ = .008 + .025 * shape_span;

    // 模板是每格中位数，误差随格内样本数按平方根下降；单条样本的残差不能代表模板质量。
    // 另外单看「最近一个周期」的残差：滑轨停住或换向时窗口前段仍然吻合，只有最近这段
    // 会露馅，这是判断模板是否还成立最灵敏的地方，也是停用陈旧模型的唯一触发点。
    // 基准要用「最近一个周期以外」的样本估，不能拿全窗残差：异常段自己会把基准抬高，
    // 阈值就永远够不着了。
    const uint64_t recent = last_stamp_ > static_cast<uint64_t>(period_ * 1e6)
        ? last_stamp_ - static_cast<uint64_t>(period_ * 1e6) : 0;
    double squared = 0, recent_squared = 0, older_squared = 0;
    std::size_t used = 0, recent_count = 0, older_count = 0;
    for (std::size_t i = start; i < samples_.size(); ++i) {
      const double error = samples_[i].point.y() - replay(samples_[i].stamp).position.y();
      squared += error * error;
      ++used;
      if (samples_[i].stamp >= recent) { recent_squared += error * error; ++recent_count; }
      else { older_squared += error * error; ++older_count; }
    }
    if (!used || recent_count < 16 || older_count < 16) { reject_ = "验证样本不足"; return; }
    const double per_bin = static_cast<double>(used) / kBins;
    template_error_ = std::sqrt(squared / used) / std::sqrt(std::max(1.0, per_bin));
    explained_ = refined_score;
    const double recent_residual = std::sqrt(recent_squared / recent_count);
    const double older_residual = std::sqrt(older_squared / older_count);
    if (template_error_ > kMaxTemplateError ||
        recent_residual > std::max(tolerance_ * 3, older_residual * 1.8)) {
      reject_ = "模板残差过大";
      return;
    }
    horizon_ = std::min(horizon_cap_, period_ * horizon_fraction_);
    confidence_ = std::clamp(1 - template_error_ / kConfidenceScaleM, .3, .99);
    ready_ = true;
    learn_ok_ = last_stamp_;
    reject_.clear();
  }

  std::deque<Sample> samples_;
  uint64_t last_stamp_{0}, anchor_{0}, next_learn_{0}, learn_ok_{0};
  std::size_t merged_{0};
  bool ready_{false};
  bool automatic_learning_{true};
  std::array<double, kBins + 1> shape_{};
  std::array<bool,kBins+1> phase_valid_{};
  Eigen::Vector3d fixed_{Eigen::Vector3d::Zero()};
  double period_{0}, tolerance_{0}, confidence_{0}, horizon_{0};
  double horizon_cap_{.2}, horizon_fraction_{.12}, min_span_{12.0};
  double explained_{0}, template_error_{0};
  std::string reject_{"尚未触发学习"};  ///< 最近一次学习尝试的失败原因，只用于诊断。
};
}  // namespace companion
