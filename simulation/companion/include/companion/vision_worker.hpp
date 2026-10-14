/** @file vision_worker.hpp
 * @brief 独立处理图像，保留一份执行任务和一份最新待处理帧。
 */
#pragma once
#include "companion/vision.hpp"
#include "companion/rm_auto_aim.hpp"
#include <memory>

namespace companion {
enum class GuideMode { Disabled, Fallback, Always };
struct VisionRequest {
  uint64_t stamp{0}, sequence{0};
  std::shared_ptr<const PpmImage> image, guide_image;
  std::string image_path;
  bool verify_metadata{true}, pixel{true}, pnp{true}, pose_valid{false};
  CameraConfig camera;
  ArmorDetectionConfig detection;
  double strip_height_m{.055}, bottom_separation_m{0}, camera_forward_m{.052};
  Pose3 exposure_pose;
  GuideMode guide_mode{GuideMode::Disabled};
  double guide_x_px{0}, guide_y_px{0}, guide_radius_px{0};
  Eigen::Vector3d guide_center{Eigen::Vector3d::Zero()}, guide_offset{Eigen::Vector3d::Zero()};
  double guide_y_gate_m{std::numeric_limits<double>::infinity()};
  double guide_z_gate_m{std::numeric_limits<double>::infinity()};
};
struct VisionResult {
  uint64_t stamp{0}, sequence{0};
  bool frame_valid{false}, world_valid{false}, green{false}, green_ran{false};
  ArmorObservation pixel;
  RmArmorDetection pnp;
  GuideObservation guide;
  Eigen::Vector3d world{Eigen::Vector3d::Zero()};
  std::string error;
};
struct VisionStageStats {
  std::vector<double> milliseconds;
  uint64_t accepted{0}, rejected{0};
};
struct VisionWorkerStats {
  uint64_t submitted{0}, completed{0}, pending_replaced{0}, result_replaced{0};
  uint64_t stale{0}, future{0}, failed{0};
  int worker_nice{-1};
  double last_compute_ms{0}, max_compute_ms{0};
  std::array<VisionStageStats, 3> stages;
};
/** @brief submit/poll 由控制线程调用；图像和检测配置在任务内保持不变。
 * @note submit/poll 只尝试消息锁。stats 用于停止测量后的统计快照。
 *       poll 以曝光时间检查时效；完成时间不替换曝光时间。
 */
class VisionWorker {
 public:
  explicit VisionWorker(bool asynchronous = true, bool record_timings = false);
  ~VisionWorker();
  VisionWorker(const VisionWorker&) = delete;
  VisionWorker& operator=(const VisionWorker&) = delete;
  bool submit(VisionRequest request);
  std::optional<VisionResult> poll(uint64_t now, uint64_t timeout);
  VisionWorkerStats stats() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  uint64_t last_submitted_{0}, last_published_{0};
};
}  // namespace companion
