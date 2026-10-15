#pragma once
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace companion {
struct CameraMetadata {
  uint64_t sequence{0}, stamp{0};
  std::string image_path;
};
struct CameraMetadataStats {
  uint64_t reads{0}, failures{0}, result_replaced{0}, max_read_us{0};
  uint64_t request_busy{0}, poll_busy{0};
};
/** @brief 工作线程探测目录并读取相机元数据，保留最新结果和一个待处理请求。
 * @note request/poll 由同一控制线程调用，只尝试消息锁，不执行文件访问。
 *       stamp 和 sequence 来自曝光元数据，图像内容由视觉线程另行校验。
 */
class CameraMetadataWorker {
 public:
  CameraMetadataWorker(std::filesystem::path runtime, int vehicle);
  ~CameraMetadataWorker();
  CameraMetadataWorker(const CameraMetadataWorker&) = delete;
  CameraMetadataWorker& operator=(const CameraMetadataWorker&) = delete;
  void request();
  std::optional<CameraMetadata> poll();
  CameraMetadataStats stats() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  uint64_t request_busy_{0}, poll_busy_{0};
};
}  // namespace companion
