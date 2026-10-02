#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

namespace
{
constexpr double kDefaultTofMinM = 0.02;
constexpr double kDefaultTofMaxM = 4.0;

std::string ReplaceAll(std::string _value, const std::string &_needle,
                       const std::string &_replacement)
{
  size_t position = 0;
  while ((position = _value.find(_needle, position)) != std::string::npos)
  {
    _value.replace(position, _needle.size(), _replacement);
    position += _replacement.size();
  }
  return _value;
}

int64_t SimTimeUs(const gz::msgs::Image &_msg)
{
  if (!_msg.has_header() || !_msg.header().has_stamp())
    return 0;
  const auto &stamp = _msg.header().stamp();
  return static_cast<int64_t>(stamp.sec()) * 1000000LL + stamp.nsec() / 1000LL;
}

bool FiniteInRange(double _value, double _min, double _max)
{
  return std::isfinite(_value) && _value >= _min && _value <= _max;
}

class SensorGateway final
{
  public: SensorGateway() = default;
  public: ~SensorGateway()
  {
    if (socketFd_ >= 0)
      ::close(socketFd_);
  }

  public: bool Configure(int _argc, char **_argv)
  {
    for (int i = 1; i < _argc; ++i)
    {
      const std::string argument = _argv[i];
      auto requireValue = [&](const char *, std::string &_target) -> bool
      {
        if (i + 1 >= _argc)
        {
          std::cerr << argument << " requires a value\n";
          return false;
        }
        _target = _argv[++i];
        return true;
      };
      if (argument == "--help")
      {
        PrintUsage();
        return false;
      }
      if (argument == "--runtime-dir")
      {
        if (!requireValue("--runtime-dir", runtimeDir_)) return false;
      }
      else if (argument == "--tof-topic")
      {
        if (!requireValue("--tof-topic", tofTopicTemplate_)) return false;
      }
      else if (argument == "--camera-topic")
      {
        if (!requireValue("--camera-topic", cameraTopicTemplate_)) return false;
      }
      else if (argument == "--udp-host")
      {
        if (!requireValue("--udp-host", udpHost_)) return false;
      }
      else if (argument == "--vehicle-count")
      {
        std::string value;
        if (!requireValue("--vehicle-count", value)) return false;
        vehicleCount_ = std::stoi(value);
      }
      else if (argument == "--udp-base-port")
      {
        std::string value;
        if (!requireValue("--udp-base-port", value)) return false;
        udpBasePort_ = std::stoi(value);
      }
      else if (argument == "--tof-noise-std-m")
      {
        std::string value;
        if (!requireValue("--tof-noise-std-m", value)) return false;
        tofNoiseStdM_ = std::stod(value);
      }
      else if (argument == "--tof-invalid-probability")
      {
        std::string value;
        if (!requireValue("--tof-invalid-probability", value)) return false;
        tofInvalidProbability_ = std::stod(value);
      }
      else if (argument == "--tof-mixed-pixel-probability")
      {
        std::string value;
        if (!requireValue("--tof-mixed-pixel-probability", value)) return false;
        tofMixedPixelProbability_ = std::stod(value);
      }
      else if (argument == "--tof-mixed-pixel-threshold-m")
      {
        std::string value;
        if (!requireValue("--tof-mixed-pixel-threshold-m", value)) return false;
        tofMixedPixelThresholdM_ = std::stod(value);
      }
      else if (argument == "--seed")
      {
        std::string value;
        if (!requireValue("--seed", value)) return false;
        seed_ = std::stoull(value, nullptr, 0);
      }
      else
      {
        std::cerr << "unknown argument: " << argument << '\n';
        PrintUsage();
        return false;
      }
    }

    if (vehicleCount_ < 1 || vehicleCount_ > 4 || udpBasePort_ < 1 ||
        udpBasePort_ > 65532)
    {
      std::cerr << "vehicle count or UDP port is outside the supported range\n";
      return false;
    }
    if (tofNoiseStdM_ < 0.0 || tofInvalidProbability_ < 0.0 ||
        tofInvalidProbability_ > 1.0 || tofMixedPixelProbability_ < 0.0 ||
        tofMixedPixelProbability_ > 1.0 || tofMixedPixelThresholdM_ < 0.0)
    {
      std::cerr << "ToF noise and probability values are invalid\n";
      return false;
    }

    socketFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd_ < 0)
    {
      std::perror("sensor_gateway: socket");
      return false;
    }

    for (int vehicle = 0; vehicle < vehicleCount_; ++vehicle)
    {
      for (int sensor = 0; sensor < 3; ++sensor)
      {
        const auto topic = ReplaceAll(
            ReplaceAll(tofTopicTemplate_, "{i}", std::to_string(vehicle)),
            "{s}", std::to_string(sensor));
        const bool subscribed = node_.Subscribe<gz::msgs::Image>(
            topic,
            [this, vehicle, sensor](const gz::msgs::Image &_msg)
            {
              OnDepth(vehicle, sensor, _msg);
            });
        if (!subscribed)
        {
          std::cerr << "sensor_gateway: failed to subscribe to " << topic << '\n';
          return false;
        }
      }

      const auto cameraTopic = ReplaceAll(
          cameraTopicTemplate_, "{i}", std::to_string(vehicle));
      const bool subscribed = node_.Subscribe<gz::msgs::Image>(
          cameraTopic,
          [this, vehicle](const gz::msgs::Image &_msg)
          {
            OnCamera(vehicle, _msg);
          });
      if (!subscribed)
      {
        std::cerr << "sensor_gateway: failed to subscribe to " << cameraTopic << '\n';
        return false;
      }
    }

    std::error_code error;
    std::filesystem::create_directories(runtimeDir_, error);
    if (error)
    {
      std::cerr << "sensor_gateway: cannot create runtime directory "
                << runtimeDir_ << ": " << error.message() << '\n';
      return false;
    }

    std::cerr << "sensor_gateway: subscribed to " << vehicleCount_
              << " vehicles; runtime output is " << runtimeDir_ << '\n';
    return true;
  }

  private: static void PrintUsage()
  {
    std::cout
        << "sensor_gateway [options]\n"
        << "  --runtime-dir PATH\n"
        << "  --tof-topic TEMPLATE   default /simulation/vehicle_{i}/tof_{s}/depth_image\n"
        << "  --camera-topic TEMPLATE default /simulation/vehicle_{i}/camera/image\n"
        << "  --udp-host IPv4        default 127.0.0.1\n"
        << "  --udp-base-port N      default 16000\n"
        << "  --tof-noise-std-m M    default 0.003\n"
        << "  --tof-invalid-probability P default 0.03\n"
        << "  --tof-mixed-pixel-probability P default 0.05\n"
        << "  --tof-mixed-pixel-threshold-m M default 0.10\n"
        << "  --seed N                default 0x544f465f53494d\n";
  }

  private: bool Send(int _vehicle, const std::string &_json)
  {
    if (socketFd_ < 0)
      return false;
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<uint16_t>(udpBasePort_ + _vehicle));
    if (::inet_pton(AF_INET, udpHost_.c_str(), &destination.sin_addr) != 1)
      return false;
    const auto result = ::sendto(socketFd_, _json.data(), _json.size(), 0,
        reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
    return result == static_cast<ssize_t>(_json.size());
  }

  private: bool ReadDepth(const gz::msgs::Image &_msg, uint32_t _x, uint32_t _y,
                          double &_depthM) const
  {
    if (_x >= _msg.width() || _y >= _msg.height())
      return false;
    const auto format = _msg.pixel_format_type();
    size_t bytesPerPixel = 0;
    switch (format)
    {
      case gz::msgs::R_FLOAT32: bytesPerPixel = sizeof(float); break;
      case gz::msgs::L_INT16: bytesPerPixel = sizeof(uint16_t); break;
      default: return false;
    }
    const auto stride = _msg.step() != 0 ? _msg.step() : _msg.width() * bytesPerPixel;
    const auto offset = static_cast<size_t>(_y) * stride +
                        static_cast<size_t>(_x) * bytesPerPixel;
    if (offset + bytesPerPixel > _msg.data().size())
      return false;
    if (format == gz::msgs::R_FLOAT32)
    {
      float value = 0.0F;
      std::memcpy(&value, _msg.data().data() + offset, sizeof(value));
      _depthM = static_cast<double>(value);
    }
    else
    {
      uint16_t value = 0;
      std::memcpy(&value, _msg.data().data() + offset, sizeof(value));
      _depthM = static_cast<double>(value) * 0.001;
    }
    return FiniteInRange(_depthM, tofMinM_, tofMaxM_);
  }

  private: void OnDepth(int _vehicle, int _sensor, const gz::msgs::Image &_msg)
  {
    if (_msg.width() != 8 || _msg.height() != 8)
      return;

    const auto simUs = SimTimeUs(_msg);
    std::array<double, 64> depth{};
    std::array<bool, 64> sourceValid{};
    for (int y = 0; y < 8; ++y)
    {
      for (int x = 0; x < 8; ++x)
      {
        const auto index = y * 8 + x;
        sourceValid[index] = ReadDepth(_msg, x, y, depth[index]);
      }
    }

    std::mt19937_64 random(seed_ + static_cast<uint64_t>(_vehicle) * 1009ULL +
        static_cast<uint64_t>(_sensor) * 9176ULL + static_cast<uint64_t>(simUs) * 104729ULL);
    std::bernoulli_distribution invalid(tofInvalidProbability_);
    std::bernoulli_distribution mixed(tofMixedPixelProbability_);
    std::normal_distribution<double> noise(0.0, tofNoiseStdM_);

    std::ostringstream json;
    json << std::setprecision(9)
         << "{\"schema\":1,\"kind\":\"tof\",\"vehicle\":" << _vehicle
         << ",\"sensor\":" << _sensor
         << ",\"sim_time_us\":" << SimTimeUs(_msg) << ",\"distance_m\":[";
    std::array<bool, 64> valid{};
    std::array<double, 64> output{};
    for (int index = 0; index < 64; ++index)
    {
      bool edgeMixed = false;
      const int x = index % 8;
      const int y = index / 8;
      constexpr std::array<std::array<int, 2>, 4> kNeighbors{{
          {{-1, 0}}, {{1, 0}}, {{0, -1}}, {{0, 1}}}};
      for (const auto &neighbor : kNeighbors)
      {
        const int nx = x + neighbor[0];
        const int ny = y + neighbor[1];
        if (nx < 0 || nx >= 8 || ny < 0 || ny >= 8)
          continue;
        const int neighborIndex = ny * 8 + nx;
        if (sourceValid[index] && sourceValid[neighborIndex] &&
            std::abs(depth[index] - depth[neighborIndex]) > tofMixedPixelThresholdM_)
          edgeMixed = true;
      }
      valid[index] = sourceValid[index] && !invalid(random) &&
                     !(edgeMixed && mixed(random));
      output[index] = valid[index]
          ? std::clamp(depth[index] + noise(random), tofMinM_, tofMaxM_) : 0.0;
      if (index != 0)
        json << ',';
      json << output[index];
    }
    json << "],\"valid\":[";
    for (int index = 0; index < 64; ++index)
    {
      if (index != 0)
        json << ',';
      json << (valid[index] ? "true" : "false");
    }
    json << "]}\n";
    Send(_vehicle, json.str());
  }

  private: bool ConvertToPpm(const gz::msgs::Image &_msg,
                             std::vector<uint8_t> &_rgb) const
  {
    const auto width = _msg.width();
    const auto height = _msg.height();
    if (width == 0 || height == 0 || width > 12000 || height > 12000)
      return false;
    const auto format = _msg.pixel_format_type();
    size_t bytesPerPixel = 0;
    if (format == gz::msgs::RGB_INT8 || format == gz::msgs::BGR_INT8)
      bytesPerPixel = 3;
    else if (format == gz::msgs::RGBA_INT8 || format == gz::msgs::BGRA_INT8)
      bytesPerPixel = 4;
    else
      return false;
    const auto stride = _msg.step() != 0 ? _msg.step() : width * bytesPerPixel;
    if (stride < width * bytesPerPixel ||
        static_cast<size_t>(height - 1) * stride + width * bytesPerPixel >
            _msg.data().size())
      return false;
    _rgb.resize(static_cast<size_t>(width) * height * 3U);
    for (uint32_t y = 0; y < height; ++y)
    {
      const auto *source = reinterpret_cast<const uint8_t *>(
          _msg.data().data() + static_cast<size_t>(y) * stride);
      for (uint32_t x = 0; x < width; ++x)
      {
        const auto sourceIndex = static_cast<size_t>(x) * bytesPerPixel;
        const auto destinationIndex = (static_cast<size_t>(y) * width + x) * 3U;
        const bool bgr = format == gz::msgs::BGR_INT8 || format == gz::msgs::BGRA_INT8;
        _rgb[destinationIndex + 0] = source[sourceIndex + (bgr ? 2 : 0)];
        _rgb[destinationIndex + 1] = source[sourceIndex + 1];
        _rgb[destinationIndex + 2] = source[sourceIndex + (bgr ? 0 : 2)];
      }
    }
    return true;
  }

  private: bool AtomicWrite(const std::filesystem::path &_target,
                            const std::string &_data, std::ios::openmode _mode)
  {
    const auto temporary = _target.string() + ".tmp";
    std::ofstream file(temporary, _mode | std::ios::trunc);
    if (!file)
      return false;
    file.write(_data.data(), static_cast<std::streamsize>(_data.size()));
    file.flush();
    if (!file)
      return false;
    file.close();
    std::error_code error;
    std::filesystem::rename(temporary, _target, error);
    if (error)
    {
      std::filesystem::remove(temporary);
      return false;
    }
    return true;
  }

  private: bool AtomicWrite(const std::filesystem::path &_target,
                            const std::vector<uint8_t> &_data)
  {
    const auto temporary = _target.string() + ".tmp";
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    if (!file)
      return false;
    file.write(reinterpret_cast<const char *>(_data.data()),
               static_cast<std::streamsize>(_data.size()));
    file.flush();
    if (!file)
      return false;
    file.close();
    std::error_code error;
    std::filesystem::rename(temporary, _target, error);
    if (error)
    {
      std::filesystem::remove(temporary);
      return false;
    }
    return true;
  }

  private: void OnCamera(int _vehicle, const gz::msgs::Image &_msg)
  {
    std::lock_guard<std::mutex> lock(cameraMutex_[_vehicle]);
    std::vector<uint8_t> rgb;
    if (!ConvertToPpm(_msg, rgb))
      return;
    const auto sequence = ++cameraSequence_[_vehicle];
    const auto width = _msg.width();
    const auto height = _msg.height();
    std::vector<uint8_t> ppm;
    const auto prefix = "P6\n# frame_seq=" + std::to_string(sequence) +
                        " sim_time_us=" + std::to_string(SimTimeUs(_msg)) +
                        "\n" + std::to_string(width) + " " +
                        std::to_string(height) + "\n255\n";
    ppm.reserve(prefix.size() + rgb.size());
    ppm.insert(ppm.end(), prefix.begin(), prefix.end());
    ppm.insert(ppm.end(), rgb.begin(), rgb.end());

    std::error_code error;
    std::filesystem::create_directories(runtimeDir_, error);
    if (error)
      return;
    const auto base = std::filesystem::path(runtimeDir_) /
                      ("camera_" + std::to_string(_vehicle));
    if (!AtomicWrite(base.string() + ".ppm", ppm))
      return;

    std::ostringstream metadata;
    metadata << "{\"schema\":1,\"kind\":\"camera_frame\",\"vehicle\":"
             << _vehicle << ",\"sim_time_us\":" << SimTimeUs(_msg)
             << ",\"frame_seq\":" << sequence
             << ",\"width\":" << width << ",\"height\":" << height
             << ",\"source_pixel_format\":\""
             << gz::msgs::PixelFormatType_Name(_msg.pixel_format_type())
             << "\",\"source_bytes\":" << _msg.data().size()
             << ",\"frame_bytes\":" << ppm.size()
             << ",\"path\":\"camera_" << _vehicle << ".ppm\"}\n";
    AtomicWrite(base.string() + ".json", metadata.str(), std::ios::out);
  }

  private: gz::transport::Node node_;
  private: int socketFd_{-1};
  private: std::string runtimeDir_{"simulation/runtime"};
  private: std::string tofTopicTemplate_{"/simulation/vehicle_{i}/tof_{s}/depth_image"};
  private: std::string cameraTopicTemplate_{"/simulation/vehicle_{i}/camera/image"};
  private: std::string udpHost_{"127.0.0.1"};
  private: int vehicleCount_{4};
  private: int udpBasePort_{16000};
  private: double tofMinM_{kDefaultTofMinM};
  private: double tofMaxM_{kDefaultTofMaxM};
  private: double tofNoiseStdM_{0.003};
  private: double tofInvalidProbability_{0.03};
  private: double tofMixedPixelProbability_{0.05};
  private: double tofMixedPixelThresholdM_{0.10};
  private: uint64_t seed_{0x544f465f53494dULL};
  private: std::array<std::mutex, 4> cameraMutex_{};
  private: std::array<std::atomic<uint64_t>, 4> cameraSequence_{};
};

}  // namespace

int main(int argc, char **argv)
{
  SensorGateway gateway;
  if (!gateway.Configure(argc, argv))
    return 2;
  while (true)
    std::this_thread::sleep_for(std::chrono::hours(24));
}
