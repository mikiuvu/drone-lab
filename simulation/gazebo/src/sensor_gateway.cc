/**
 * @file sensor_gateway.cc
 * @brief 将 Gazebo 深度图转为 ToF UDP 观测，将彩色图发布为带时间戳的最新 PPM 文件。
 */
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
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// 与离线消融实验共用的 ToF 误差模型；相对路径使本文件不需要额外的包含目录。
#include "../../research/tof_error_model.hpp"

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

/**
 * @brief 展开话题模板中的车辆和传感器占位符。
 * @param _value 待替换的字符串副本。
 * @param _needle 非空的匹配文本。
 * @param _replacement 替换文本；新插入部分不会再次参与匹配。
 * @return 替换全部匹配后的字符串。
 */
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

/**
 * @brief 提取图像采集时的仿真时间，纳秒部分截断到微秒。
 * @param _msg Gazebo 图像消息。
 * @return 仿真时间，单位 us；消息缺少时间戳时返回 0。
 */
int64_t SimTimeUs(const gz::msgs::Image &_msg)
{
  if (!_msg.has_header() || !_msg.header().has_stamp())
    return 0;
  const auto &stamp = _msg.header().stamp();
  return static_cast<int64_t>(stamp.sec()) * 1000000LL + stamp.nsec() / 1000LL;
}

// 同时排除 NaN/无穷和量程外深度，端点属于有效量程。
bool FiniteInRange(double _value, double _min, double _max)
{
  return std::isfinite(_value) && _value >= _min && _value <= _max;
}

/**
 * @brief 持有图像订阅、UDP 套接字和各机相机发布状态。
 * @note 图像处理在 transport 回调中进行；同一车辆的相机写入由互斥锁串行化。
 */
class SensorGateway final
{
  public: SensorGateway() = default;
  public: ~SensorGateway()
  {
    if (socketFd_ >= 0)
      ::close(socketFd_);
  }

  /**
   * @brief 解析参数、建立 UDP 套接字并订阅每机三个深度话题和一个彩色话题。
   * @param _argc 参数数量，包含程序名。
   * @param _argv 参数数组；话题模板中的 {i}/{s} 分别为车辆/传感器编号。
   * @return 初始化完成返回 true；帮助请求、参数检查或资源创建失败返回 false。
   * @note 数值转换异常向调用方传播；每机 UDP 目标端口为 base + vehicle。
   */
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
      else if (argument == "--tof-range-sigma-per-m")
      {
        std::string value;
        if (!requireValue("--tof-range-sigma-per-m", value)) return false;
        tofRangeSigmaPerM_ = std::stod(value);
      }
      else if (argument == "--tof-bias-bound-m")
      {
        std::string value;
        if (!requireValue("--tof-bias-bound-m", value)) return false;
        tofBiasBoundM_ = std::stod(value);
      }
      else if (argument == "--tof-scale-bound")
      {
        std::string value;
        if (!requireValue("--tof-scale-bound", value)) return false;
        tofScaleBound_ = std::stod(value);
      }
      else if (argument == "--tof-frame-bias-std-m")
      {
        std::string value;
        if (!requireValue("--tof-frame-bias-std-m", value)) return false;
        tofFrameBiasStdM_ = std::stod(value);
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

    if (vehicleCount_ < 1 || udpBasePort_ < 1 ||
        udpBasePort_ + vehicleCount_ > 65536)
    {
      std::cerr << "vehicle count or UDP port is outside the supported range\n";
      return false;
    }
    // 命令行参数组装成共享剖面；缺省值等于本程序原有参数，不改变默认行为。
    profile_ = drone_sim::research::TofNoiseProfile{
        tofNoiseStdM_, tofRangeSigmaPerM_, tofBiasBoundM_, tofScaleBound_,
        tofFrameBiasStdM_, tofInvalidProbability_};
    edgePolicy_ = drone_sim::research::TofEdgePolicy{
        true, tofMixedPixelProbability_, tofMixedPixelThresholdM_};
    if (!drone_sim::research::TofProfileIsValid(profile_) ||
        !std::isfinite(tofMixedPixelProbability_) || tofMixedPixelProbability_ < 0.0 ||
        tofMixedPixelProbability_ > 1.0 || !std::isfinite(tofMixedPixelThresholdM_) ||
        tofMixedPixelThresholdM_ < 0.0)
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

    cameraMutex_=std::make_unique<std::mutex[]>(vehicleCount_);
    cameraSequence_=std::make_unique<std::atomic<uint64_t>[]>(vehicleCount_);
    for(int i=0;i<vehicleCount_;++i)cameraSequence_[i]=0;
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
        << "  --tof-range-sigma-per-m M default 0.0\n"
        << "  --tof-bias-bound-m M   default 0.0\n"
        << "  --tof-scale-bound S    default 0.0\n"
        << "  --tof-frame-bias-std-m M default 0.0\n"
        << "  --seed N                default 0x544f465f53494d\n";
  }

  /**
   * @brief 将一条 JSON 观测作为一个 UDP 数据报发送到指定车辆。
   * @param _vehicle 已配置的车辆编号。
   * @param _json 完整报文文本。
   * @return sendto 接受全部字节时为 true；不表示接收方已处理。
   */
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

  /**
   * @brief 按图像行跨度读取并校验一个轴向深度像素。
   * @param _msg R_FLOAT32 米制或 L_INT16 毫米制图像。
   * @param _x 列号，从 0 开始。
   * @param _y 行号，从 0 开始。
   * @param _depthM 输出深度，单位 m；仅返回 true 时可作为有效测量使用。
   * @return 格式、字节范围和有限量程检查全部通过时为 true。
   */
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

  /**
   * @brief 将 8×8 深度图加入测量噪声和失效模型，再发送行优先 ToF 报文。
   * @param _vehicle 车辆编号。
   * @param _sensor 三向 ToF 编号 0..2。
   * @param _msg transport 提供的原始深度图；非 8×8 图像直接忽略。
   * @note distance_m 为轴向深度；无效像素输出 0，消费者须同时检查 valid 掩码。
   */
  private: void OnDepth(int _vehicle, int _sensor, const gz::msgs::Image &_msg)
  {
    if (_msg.width() != 8 || _msg.height() != 8)
      return;

    // 1. 先保存原始深度和有效性，边缘判定始终使用未加噪的邻域。
    const auto simUs = SimTimeUs(_msg);
    std::array<double, drone_sim::research::kTofPixelCount> depth{};
    std::array<bool, drone_sim::research::kTofPixelCount> sourceValid{};
    for (int y = 0; y < 8; ++y)
    {
      for (int x = 0; x < 8; ++x)
      {
        const auto index = y * 8 + x;
        sourceValid[index] = ReadDepth(_msg, x, y, depth[index]);
      }
    }

    // 2. 共享误差模型按 (种子, 车辆, 传感器, 帧时间, 像素) 派生随机量，
    //    回调先后顺序不影响同帧结果；量程外像素沿用本程序的限幅行为。
    const auto frame = drone_sim::research::ApplyTofFrame(
        profile_, edgePolicy_, seed_, _vehicle, _sensor, simUs, depth, sourceValid,
        tofMinM_, tofMaxM_, drone_sim::research::TofRangePolicy::kClamp);

    std::ostringstream json;
    json << std::setprecision(9)
         << "{\"schema\":1,\"kind\":\"tof\",\"vehicle\":" << _vehicle
         << ",\"sensor\":" << _sensor
         << ",\"sim_time_us\":" << SimTimeUs(_msg) << ",\"distance_m\":[";
    for (int index = 0; index < drone_sim::research::kTofPixelCount; ++index)
    {
      if (index != 0)
        json << ',';
      json << frame.distance_m[index];
    }
    json << "],\"valid\":[";
    for (int index = 0; index < drone_sim::research::kTofPixelCount; ++index)
    {
      if (index != 0)
        json << ',';
      json << (frame.valid[index] ? "true" : "false");
    }
    json << "]}\n";
    Send(_vehicle, json.str());
  }

  /**
   * @brief 去除行填充和 alpha，并把 RGB/BGR 图像统一为连续 RGB 字节。
   * @param _msg RGB、BGR、RGBA 或 BGRA 的 8 位图像。
   * @param _rgb 输出 width×height×3 字节，不包含 PPM 文件头。
   * @return 尺寸、格式和缓冲区完整性检查通过时为 true。
   */
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

  /**
   * @brief 完整写入同目录临时文件，再重命名替换文本目标。
   * @param _target 输出文件；调用方须串行化对同一目标的写入。
   * @param _data 完整文件内容。
   * @param _mode 流打开模式，内部追加 trunc。
   * @return 写入、刷新和重命名均成功时为 true。
   */
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

  /**
   * @brief 以二进制模式写临时文件并重命名替换目标，避免读到半帧。
   * @param _target 输出文件；调用方须串行化对同一目标的写入。
   * @param _data 完整二进制内容。
   * @return 写入、刷新和重命名均成功时为 true。
   */
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

  /**
   * @brief 串行发布该车辆的最新 PPM 图像，随后更新配套 JSON 元数据。
   * @param _vehicle 车辆编号，索引对应的互斥锁和帧序号。
   * @param _msg transport 彩色图像消息。
   * @note 图像头和 JSON 均携带 frame_seq 与 sim_time_us；两个文件分别替换，
   *       消费者应核对帧标识，不能把两次替换视为一个文件事务。
   */
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
  // ToF 有效深度下限/上限与噪声标准差使用 m；失效概率无单位。
  private: double tofMinM_{kDefaultTofMinM};
  private: double tofMaxM_{kDefaultTofMaxM};
  // 缺省值直接引用共享误差模型常量，保证默认参数只有一处定义。
  private: double tofNoiseStdM_{drone_sim::research::kDefaultTofNoiseStdM};
  private: double tofInvalidProbability_{drone_sim::research::kDefaultTofDropProbability};
  private: double tofMixedPixelProbability_{
      drone_sim::research::kDefaultTofMixedPixelProbability};
  // 四邻域边缘深度差门限，单位 m。
  private: double tofMixedPixelThresholdM_{
      drone_sim::research::kDefaultTofMixedPixelThresholdM};
  // 共享剖面的新增项；未显式传参时保持原有的纯白噪声行为。
  private: double tofRangeSigmaPerM_{0.0};
  private: double tofBiasBoundM_{0.0};
  private: double tofScaleBound_{0.0};
  private: double tofFrameBiasStdM_{0.0};
  private: drone_sim::research::TofNoiseProfile profile_;
  private: drone_sim::research::TofEdgePolicy edgePolicy_;
  private: uint64_t seed_{0x544f465f53494dULL};
  private: std::unique_ptr<std::mutex[]> cameraMutex_;
  private: std::unique_ptr<std::atomic<uint64_t>[]> cameraSequence_;
};

}  // namespace

/**
 * @brief 配置图像网关并保持进程存活，消息处理由 transport 回调驱动。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 配置失败返回 2；成功后持续等待外部进程管理器结束进程。
 */
int main(int argc, char **argv)
{
  SensorGateway gateway;
  if (!gateway.Configure(argc, argv))
    return 2;
  while (true)
    std::this_thread::sleep_for(std::chrono::hours(24));
}
