#include "companion/camera_metadata_worker.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace companion;
namespace {
CameraMetadata await(CameraMetadataWorker& worker, uint64_t sequence) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto result = worker.poll()) if (result->sequence == sequence) return *result;
    worker.request(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(false && "camera metadata did not arrive"); return {};
}
void write(const std::filesystem::path& path, const std::string& value) {
  const auto pending = std::filesystem::path(path.string() + ".pending");
  { std::ofstream stream(pending); stream << value; assert(stream.good()); }
  std::filesystem::rename(pending, path);
}
}
int main() {
  const auto root = std::filesystem::temp_directory_path() / ("camera-metadata-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(root);
  {
    CameraMetadataWorker worker(root, 3);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (worker.stats().failures == 0 && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(worker.stats().failures > 0);
    write(root / "camera_3.json", R"({"frame_seq":1,"sim_time_us":1234567})");
    const auto first = await(worker, 1);
    assert(first.stamp == 1234567 && first.image_path == (root / "camera_3.ppm").string());
    std::filesystem::create_directories(root / "camera");
    write(root / "camera/camera_3.json", "{");
    const auto failed = worker.stats().failures;
    worker.request();
    const auto invalid_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (worker.stats().failures <= failed && std::chrono::steady_clock::now() < invalid_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(worker.stats().failures > failed);
    write(root / "camera/camera_3.json", R"({"frame_seq":2,"sim_time_us":7654321})");
    const auto second = await(worker, 2);
    assert(second.stamp == 7654321 && second.image_path == (root / "camera/camera_3.ppm").string());
    std::filesystem::remove_all(root / "camera");
    write(root / "camera_3.json", R"({"frame_seq":3,"sim_time_us":9999999})");
    const auto third = await(worker, 3);
    assert(third.stamp == 9999999 && third.image_path == (root / "camera_3.ppm").string());
  }
  std::filesystem::remove_all(root);
}
