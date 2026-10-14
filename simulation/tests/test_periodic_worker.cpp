#include "companion/periodic_motion_worker.hpp"
#include "companion/armor_tracker.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <thread>

using companion::PeriodicMotion;
using companion::PeriodicMotionWorker;
namespace {
Eigen::Vector3d point(uint64_t stamp) {
  return {11, .2 * std::sin(stamp * 1e-6 * 6.283185307179586), 1.4};
}
void complete(PeriodicMotionWorker& worker, uint64_t now, uint64_t expected = 1) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (worker.learningStats().completed < expected) {
    worker.service(now);
    assert(std::chrono::steady_clock::now() < end);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void submit(PeriodicMotionWorker& worker, uint64_t now, uint64_t expected) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (worker.learningStats().submitted < expected) {
    worker.service(now);
    assert(std::chrono::steady_clock::now() < end);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
}

int main() {
  PeriodicMotionWorker worker;
  worker.setMinSpan(4);
  PeriodicMotion reference;
  reference.setMinSpan(4);
  for (uint64_t stamp = 5000; stamp <= 5200000; stamp += 5000) {
    assert(worker.observe(stamp, point(stamp)));
    assert(reference.observe(stamp, point(stamp)));
  }
  // 收集观测不会在调用线程中隐式启动搜索。
  assert(!worker.ready());
  assert(worker.learningStats().submitted == 0);
  complete(worker, 5200000);
  assert(worker.ready());
  assert(std::abs(worker.period_estimate() - reference.period_estimate()) < .005);
  auto prediction = worker.predict(5300000);
  assert(prediction.valid && prediction.confidence > .8);
  assert(std::abs(prediction.position.y() - point(5300000).y()) < .03);
  assert(!worker.predict(5190000).valid);
  assert(!worker.predict(5500000).valid);

  // 后台结果不得把仍在接收的观测或时钟回退到旧快照。
  for (uint64_t stamp = 5205000; stamp <= 6400000; stamp += 5000)
    assert(worker.observe(stamp, point(stamp)));
  submit(worker, 6400000, 2);
  for (uint64_t stamp = 6405000; stamp <= 6800000; stamp += 5000)
    assert(worker.observe(stamp, point(stamp)));
  complete(worker, 6800000, 2);
  assert(worker.exportSamples(192).back().stamp > 6700000);
  assert(!worker.observe(6700000, point(6700000)));
  assert(worker.predict(6800000).valid);

  // 样本持续到达也不能无限续用两周期以前的模型。
  for (uint64_t stamp = 6805000; stamp <= 9000000; stamp += 5000)
    assert(worker.observe(stamp, {11, .2, 1.4}));
  assert(!worker.predict(9000000).valid);

  // 配置改变后，尚未完成的旧配置快照不能发布。
  PeriodicMotionWorker changed;
  changed.setMinSpan(4);
  for (uint64_t stamp = 5000; stamp <= 5200000; stamp += 5000)
    assert(changed.observe(stamp, point(stamp)));
  changed.service(5200000);
  changed.setMinSpan(20);
  complete(changed, 5200000);
  assert(changed.learningStats().discarded == 1);
  assert(!changed.ready());

  // 搜索尚未发布时目标改变，完成的旧模板应由新观测留出集拒绝。
  PeriodicMotionWorker drift;
  drift.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)drift.observe(stamp,point(stamp));
  submit(drift,5200000,1);
  for(uint64_t stamp=5205000;stamp<=5600000;stamp+=5000){
    auto shifted=point(stamp);shifted.y()+=.8;drift.observe(stamp,shifted);
  }
  complete(drift,5600000);
  assert(drift.learningStats().publication_rejected==1);
  assert(!drift.predict(5600000).valid);

  // 共享观测能补足窗口，但导出仍仅包含本机数据。
  PeriodicMotionWorker shared;
  shared.setMinSpan(4);
  for (uint64_t stamp = 3005000; stamp <= 5200000; stamp += 5000)
    assert(shared.observe(stamp, point(stamp)));
  std::vector<companion::PeriodicSample> incoming;
  for (uint64_t stamp = 5000; stamp <= 3000000; stamp += 5000)
    incoming.push_back({stamp, point(stamp).y()});
  assert(shared.merge(incoming) == incoming.size());
  assert(shared.merge(incoming) == 0);
  complete(shared, 5200000);
  assert(shared.ready());
  assert(shared.exportSamples(192).front().stamp >= 3005000);
  assert(shared.predict(5300000).valid);

  // 带噪三角波模拟滑轨；相同快照的后台与同步搜索必须给出相同模板。
  PeriodicMotionWorker triangle, synchronous(false);
  triangle.setHorizonCap(.6); synchronous.setHorizonCap(.6);
  triangle.setHorizonFraction(.25); synchronous.setHorizonFraction(.25);
  std::vector<companion::PeriodicSample> samples;
  for (uint64_t stamp = 20000; stamp <= 20000000; stamp += 20000) {
    const double phase = std::fmod(stamp * 1e-6 / 5.6, 1.0);
    const double y = .28 - 1.12 * std::min(phase, 1 - phase) + .02 * std::sin(stamp * 1e-6 * 97);
    samples.push_back({stamp, y});
  }
  triangle.merge(samples); synchronous.merge(samples);
  complete(triangle, 20000000);
  assert(triangle.ready() && synchronous.ready());
  assert(std::abs(triangle.period_estimate() - 5.6) < .025);
  assert(std::abs(triangle.period_estimate() - synchronous.period_estimate()) < 1e-12);
  for (uint64_t now = 20000000; now <= 20600000; now += 20000) {
    const auto background = triangle.predict(now), foreground = synchronous.predict(now);
    assert(background.valid == foreground.valid);
    if (background.valid) {
      assert((background.position - foreground.position).norm() < 1e-12);
      assert((background.velocity - foreground.velocity).norm() < 1e-12);
      assert(std::abs(background.confidence - foreground.confidence) < 1e-12);
    }
  }
  // 只有队友继续发包时，也必须停用超过两周期未验证的模型。
  triangle.merge({{32000000, .1}});
  assert(!triangle.predict(32000000).valid);

  // 少数相位格的重复离群点不能凭全局置信度放行错误预测。
  PeriodicMotion spikes;
  spikes.setMinSpan(3); spikes.setHorizonCap(1); spikes.setHorizonFraction(.9);
  for(uint64_t stamp=1000;stamp<=5200000;stamp+=1000){
    const double phase=std::fmod(stamp*1e-6,1.0);
    auto observed=point(stamp);
    if(phase>=.450&&phase<.459)observed.y()+=.35;
    spikes.observe(stamp,observed);
  }
  assert(spikes.ready());
  int valid=0;
  for(uint64_t stamp=5200000;stamp<=5900000;stamp+=1000){
    const auto p=spikes.predict(stamp);
    if(p.valid){++valid;assert(std::abs(p.position.y()-point(stamp).y())<.08);}
  }
  assert(valid>600);

  // 已发布的模型也要逐次检查；搜索没有完成不能继续放行相位已改变的预测。
  PeriodicMotionWorker motion;
  motion.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)motion.observe(stamp,point(stamp));
  complete(motion,5200000);
  auto outlier=point(5205000);outlier.y()+=.4;
  motion.observe(5205000,outlier);
  assert(!motion.predict(5205000).valid);
  motion.observe(5210000,point(5210000));
  motion.observe(5215000,point(5215000));
  assert(motion.predict(5215000).valid);
  for(uint64_t stamp=5220000;stamp<=5260000;stamp+=20000)
    motion.observe(stamp,point(stamp+500000));
  assert(!motion.predict(5260000).valid);
  assert(motion.learningStats().motion_resets==1);
  for(uint64_t stamp=5265000;stamp<=10000000;stamp+=5000)
    motion.observe(stamp,point(stamp+500000));
  complete(motion,10000000,2);
  const auto recovered=motion.predict(10100000);
  assert(recovered.valid);
  assert(std::abs(recovered.position.y()-point(10600000).y())<.03);

  // 运动变化发生在搜索期间时，旧窗口结果不能重新启用模型。
  PeriodicMotionWorker changing;
  changing.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)changing.observe(stamp,point(stamp));
  complete(changing,5200000);
  for(uint64_t stamp=5205000;stamp<=6400000;stamp+=5000)changing.observe(stamp,point(stamp));
  submit(changing,6400000,2);
  for(uint64_t stamp=6420000;stamp<=6460000;stamp+=20000){
    auto shifted=point(stamp);shifted.y()+=.4;
    changing.observe(stamp,shifted);
  }
  assert(changing.learningStats().motion_resets==1);
  complete(changing,6460000,2);
  assert(changing.learningStats().discarded==1);
  assert(!changing.ready()&&!changing.predict(6460000).valid);

  // 候选拒绝后仍需继续确认旧模型的异常；重复曝光只暂停，不提前重置。
  PeriodicMotionWorker rejected;
  rejected.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)rejected.observe(stamp,point(stamp));
  complete(rejected,5200000);
  for(uint64_t stamp=5205000;stamp<=6400000;stamp+=5000)rejected.observe(stamp,point(stamp));
  submit(rejected,6400000,2);
  std::vector<companion::PeriodicSample> late_changes;
  for(uint64_t stamp=6405000;stamp<=6600000;stamp+=5000)
    late_changes.push_back({stamp,point(stamp).y()+.8,1});
  rejected.merge(late_changes);
  complete(rejected,6600000,2);
  assert(rejected.learningStats().publication_rejected==1);
  assert(rejected.learningStats().motion_resets==0);
  for(uint64_t stamp=6620000;stamp<=6660000;stamp+=20000){
    rejected.merge({{stamp,point(stamp).y()+.8,1}});rejected.service(stamp);
  }
  assert(rejected.learningStats().motion_resets==1&&!rejected.ready());

  // 新模板发布不能清除发布前刚收到的异常；两次新正常曝光才恢复预测。
  PeriodicMotionWorker pending_guard;
  pending_guard.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)pending_guard.observe(stamp,point(stamp));
  complete(pending_guard,5200000);
  for(uint64_t stamp=5205000;stamp<=6400000;stamp+=5000)pending_guard.observe(stamp,point(stamp));
  submit(pending_guard,6400000,2);
  auto newest=point(6405000);newest.y()+=.4;
  pending_guard.observe(6405000,newest);
  complete(pending_guard,6405000,2);
  assert(!pending_guard.predict(6405000).valid);
  pending_guard.observe(6420000,point(6420000));
  pending_guard.observe(6440000,point(6440000));
  assert(pending_guard.predict(6440000).valid);

  // 原始共享点允许更大的单帧噪声；重复广播和正负交替误差不能触发窗口重置。
  PeriodicMotionWorker raw;
  raw.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)raw.observe(stamp,point(stamp));
  complete(raw,5200000);
  raw.merge({{5210000,point(5210000).y()+.18}});
  raw.service(5210000);
  assert(raw.predict(5210000).valid);
  const std::vector<companion::PeriodicSample> noisy{{5230000,point(5230000).y()+.25}};
  raw.merge(noisy);
  for(int i=0;i<4;++i)raw.merge(noisy);
  raw.service(5230000);
  raw.observe(5235000,point(5235000));
  assert(!raw.predict(5235000).valid);
  raw.merge({{5250000,point(5250000).y()-.25}});raw.service(5250000);
  raw.merge({{5270000,point(5270000).y()+.25}});raw.service(5270000);
  assert(raw.learningStats().motion_resets==0);
  raw.merge({{5290000,point(5290000).y()}});raw.service(5290000);
  raw.merge({{5310000,point(5310000).y()}});raw.service(5310000);
  assert(raw.predict(5310000).valid);
  for(uint64_t stamp=5330000;stamp<=5370000;stamp+=20000){
    raw.merge({{stamp,point(stamp).y()+.25}});raw.service(stamp);
  }
  assert(raw.learningStats().motion_resets==1&&!raw.predict(5370000).valid);

  // 同一曝光可用于连续预测，但不能被当成多次独立的运动变化证据。
  PeriodicMotionWorker repeated;
  repeated.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)repeated.observe(stamp,point(stamp));
  complete(repeated,5200000);
  for(uint64_t stamp=5220000;stamp<=5320000;stamp+=20000){
    auto wrong=point(5220000);wrong.y()+=.3;
    repeated.observeMeasurement(5220000,wrong);
  }
  assert(!repeated.predict(5320000).valid&&repeated.learningStats().motion_resets==0);
  repeated.observeMeasurement(5340000,point(5340000));
  repeated.observeMeasurement(5360000,point(5360000));
  assert(repeated.predict(5360000).valid);
  companion::ArmorTracker clock;
  clock.observe(1000000,{11,0,1.4});clock.observe(1020000,{11,.004,1.4});clock.observe(1040000,{11,.008,1.4});
  assert(clock.at(1100000).measurement_time_us==1040000);
  assert(clock.at(1200000).measurement_time_us==1040000);
  assert(std::abs(clock.at(1200000).measurement_position.y()-.008)<1e-12);

  // 延迟本机曝光可插入已有共享时间线；同刻优先本机，重复曝光不增加权重。
  PeriodicMotionWorker exposure;
  exposure.setMinSpan(4);exposure.setHorizonCap(.6);exposure.setHorizonFraction(.8);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)exposure.observe(stamp,point(stamp));
  complete(exposure,5200000);
  exposure.merge({{5600000,point(5600000).y(),1}});exposure.service(5600000);
  const auto previous_samples=exposure.sample_count();
  assert(exposure.observeMeasurement(5350000,point(5350000)));
  assert(exposure.sample_count()==previous_samples+1);
  assert(!exposure.observeMeasurement(5350000,point(5350000)));
  assert(exposure.observeMeasurement(5600000,point(5600000)));
  assert(exposure.merged_count()==0);
  const auto exported=exposure.exportSamples(192);
  assert(exported.size()==192&&exported.front().stamp==5000&&exported.back().stamp==5600000);
  assert(!exposure.predict(5590000).valid);
  assert(exposure.ready()&&exposure.predict(5600000).valid);
  assert(exposure.learningStats().motion_resets==0);
  for(uint64_t stamp=5620000;stamp<=5660000;stamp+=20000){
    auto shifted=point(stamp);shifted.y()+=.4;
    exposure.observeMeasurement(stamp,shifted);
  }
  assert(exposure.learningStats().motion_resets==1&&!exposure.predict(5660000).valid);
  const auto remaining=exposure.sample_count();
  assert(!exposure.observeMeasurement(5350000,point(5350000)));
  assert(exposure.merge({{5350000,point(5350000).y(),2}})==0);
  assert(exposure.sample_count()==remaining);

  // 同刻的多机观测保留来源；单机持续偏差不能覆盖另外两台的正常观测。
  PeriodicMotionWorker consensus;
  consensus.setMinSpan(4);
  for(uint64_t stamp=5000;stamp<=5200000;stamp+=5000)consensus.observe(stamp,point(stamp));
  complete(consensus,5200000);
  for(uint64_t stamp=5220000;stamp<=5320000;stamp+=20000){
    consensus.merge({{stamp,point(stamp).y()+.8,1}});
    consensus.merge({{stamp,point(stamp).y(),2}});
    if(stamp>=5280000)consensus.merge({{stamp,point(stamp).y(),3}});
    consensus.service(stamp);
    assert(consensus.ready()&&consensus.predict(stamp).valid);
  }
  assert(consensus.learningStats().motion_resets==0);
  for(uint64_t stamp=5340000;stamp<=5380000;stamp+=20000){
    // 刚到达的新曝光不能被另外两个来源上一帧的正常值掩盖。
    consensus.merge({{stamp,point(stamp).y()+.4,1}});
    consensus.service(stamp);
    assert(!consensus.predict(stamp).valid);
    consensus.merge({{stamp,point(stamp).y()+.4,1},{stamp,point(stamp).y()+.4,2},{stamp,point(stamp).y()+.4,3}});
    consensus.service(stamp);
  }
  assert(consensus.learningStats().motion_resets==1&&!consensus.ready());
}
