/**
 * @file tof_error_model.hpp
 * @brief 8×8 ToF 测距误差模型：设备固定偏差/比例、帧公共误差、逐像素白噪声与失效判定。
 * @note 供真实 sensor_gateway 与离线消融实验共用；不依赖 Eigen，全部为纯函数，
 *       随机量由 (种子, 车辆, 传感器, 帧时间, 像素) 直接派生，回调顺序不影响同帧结果。
 */

#ifndef DRONE_SIM_RESEARCH_TOF_ERROR_MODEL_HPP
#define DRONE_SIM_RESEARCH_TOF_ERROR_MODEL_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace drone_sim::research
{

// 当前 sensor_gateway 的默认参数。模型只读取这些常量作为缺省值，
// 任何替换都必须由调用方显式传入，不在模型内部静默改变。
constexpr double kDefaultTofNoiseStdM = 0.003;
constexpr double kDefaultTofDropProbability = 0.03;
constexpr double kDefaultTofMixedPixelProbability = 0.05;
constexpr double kDefaultTofMixedPixelThresholdM = 0.10;
constexpr double kDefaultTofMinRangeM = 0.02;
constexpr double kDefaultTofMaxRangeM = 4.0;
constexpr int kTofPixelCount = 64;
constexpr double kTofPi = 3.14159265358979323846;

/**
 * @brief 一条测距剖面的误差参数，字段名与 scenarios.json 的 profiles 一致。
 * @note 单位 m；scale_bound 与 drop_probability 为无量纲比例。
 */
struct TofNoiseProfile
{
  double noise_std_m = kDefaultTofNoiseStdM;
  double range_sigma_per_m = 0.0;
  double bias_bound_m = 0.0;
  double scale_bound = 0.0;
  double frame_bias_std_m = 0.0;
  double drop_probability = kDefaultTofDropProbability;
};

/**
 * @brief 校验剖面参数是否为有限且非负、概率是否落在 0..1。
 * @param _profile 待校验剖面。
 * @return 全部通过返回 true。
 */
inline bool TofProfileIsValid(const TofNoiseProfile &_profile)
{
  const double values[] = {_profile.noise_std_m, _profile.range_sigma_per_m,
                           _profile.bias_bound_m, _profile.scale_bound,
                           _profile.frame_bias_std_m};
  for (const double value : values)
  {
    if (!std::isfinite(value) || value < 0.0)
      return false;
  }
  return std::isfinite(_profile.drop_probability) &&
         _profile.drop_probability >= 0.0 && _profile.drop_probability <= 1.0;
}

// 设备固定项：同一车辆同一传感器整段运行不变。
struct TofNoiseDevice
{
  double bias_m = 0.0;
  double scale = 1.0;
};

// 帧公共项：同一帧 64 像素共享；frame_key 供逐像素抽样继续派生。
struct TofFrameError
{
  double frame_bias_m = 0.0;
  uint64_t frame_key = 0;
};

// 单像素结果：valid 为 false 时 distance_m 为 0，调用方须同时读取 valid。
struct TofPixelResult
{
  double distance_m = 0.0;
  bool valid = false;
};

// 量程外像素的处理方式：按无效上报，或沿用现有 gateway 的限幅行为。
enum class TofRangePolicy
{
  kMarkInvalid,
  kClamp,
};

// 深度突变边缘的失效模型，默认关闭，不改变现有行为。
struct TofEdgePolicy
{
  bool enabled = false;
  double mixed_pixel_probability = kDefaultTofMixedPixelProbability;
  double mixed_pixel_threshold_m = kDefaultTofMixedPixelThresholdM;
};

// 整帧输出：行优先 64 像素的轴向深度与有效掩码。
struct TofFrameResult
{
  std::array<double, kTofPixelCount> distance_m{};
  std::array<bool, kTofPixelCount> valid{};
};

/**
 * @brief splitmix64 混合函数，把任意键扩散为统计独立的 64 位值。
 * @param _key 输入键。
 * @return 混合后的键。
 */
constexpr uint64_t TofMix(uint64_t _key)
{
  _key += 0x9E3779B97F4A7C15ULL;
  _key = (_key ^ (_key >> 30)) * 0xBF58476D1CE4E5B9ULL;
  _key = (_key ^ (_key >> 27)) * 0x94D049BB133111EBULL;
  return _key ^ (_key >> 31);
}

/**
 * @brief 由键得到 [0,1) 的均匀抽样，取 53 位尾数。
 * @param _key 输入键。
 * @return 均匀分布样本。
 */
inline double TofUniform(uint64_t _key)
{
  constexpr double kInverse = 1.0 / 9007199254740992.0;
  return static_cast<double>(TofMix(_key) >> 11) * kInverse;
}

/**
 * @brief 由键得到标准正态抽样，使用 Box-Muller 变换。
 * @param _key 输入键。
 * @return 标准正态样本。
 */
inline double TofNormal(uint64_t _key)
{
  const double first = TofUniform(_key);
  const double second = TofUniform(_key ^ 0xD1B54A32D192ED03ULL);
  const double safe = first > std::numeric_limits<double>::min()
                          ? first
                          : std::numeric_limits<double>::min();
  return std::sqrt(-2.0 * std::log(safe)) * std::cos(2.0 * kTofPi * second);
}

/**
 * @brief 由键得到区间内的均匀抽样。
 * @param _key 输入键。
 * @param _low 下界。
 * @param _high 上界。
 * @return 区间内样本。
 */
inline double TofUniformRange(uint64_t _key, double _low, double _high)
{
  return _low + (_high - _low) * TofUniform(_key);
}

/**
 * @brief 设备固定项键：同一车辆同一传感器在整段运行中恒定。
 * @param _seed 全局种子。
 * @param _vehicle 车辆编号。
 * @param _sensor 传感器编号 0..2。
 * @return 设备键。
 */
inline uint64_t TofDeviceKey(uint64_t _seed, int _vehicle, int _sensor)
{
  return TofMix(_seed ^ (static_cast<uint64_t>(_vehicle) * 1009ULL +
                         static_cast<uint64_t>(_sensor) * 9176ULL +
                         0xA5A5A5A5A5A5A5A5ULL));
}

/**
 * @brief 帧键：由设备与仿真帧时间派生，同帧固定、跨帧独立。
 * @param _seed 全局种子。
 * @param _vehicle 车辆编号。
 * @param _sensor 传感器编号。
 * @param _sim_time_us 帧时间戳，单位 us。
 * @return 帧键。
 */
inline uint64_t TofFrameKey(uint64_t _seed, int _vehicle, int _sensor,
                            int64_t _sim_time_us)
{
  return TofMix(TofDeviceKey(_seed, _vehicle, _sensor) ^
                (static_cast<uint64_t>(_sim_time_us) * 104729ULL));
}

/**
 * @brief 像素键：由帧键与像素序号派生。
 * @param _frame_key 帧键。
 * @param _pixel_index 行优先像素序号 0..63。
 * @return 像素键。
 */
inline uint64_t TofPixelKey(uint64_t _frame_key, int _pixel_index)
{
  return TofMix(_frame_key ^ (static_cast<uint64_t>(_pixel_index) * 2654435761ULL));
}

/**
 * @brief 抽取设备固定偏差与比例，整段运行复用同一个结果。
 * @param _profile 误差剖面。
 * @param _seed 全局种子。
 * @param _vehicle 车辆编号。
 * @param _sensor 传感器编号。
 * @return 设备固定项；bias 在 ±bias_bound_m 内，scale 在 1±scale_bound 内。
 */
inline TofNoiseDevice MakeTofNoiseDevice(const TofNoiseProfile &_profile,
                                         uint64_t _seed, int _vehicle,
                                         int _sensor)
{
  const uint64_t key = TofDeviceKey(_seed, _vehicle, _sensor);
  TofNoiseDevice device;
  device.bias_m = TofUniformRange(key ^ 0x11ULL, -_profile.bias_bound_m,
                                  _profile.bias_bound_m);
  device.scale = TofUniformRange(key ^ 0x12ULL, 1.0 - _profile.scale_bound,
                                 1.0 + _profile.scale_bound);
  return device;
}

/**
 * @brief 抽取帧公共误差，同帧 64 像素共用该值。
 * @param _profile 误差剖面。
 * @param _seed 全局种子。
 * @param _vehicle 车辆编号。
 * @param _sensor 传感器编号。
 * @param _sim_time_us 帧时间戳，单位 us。
 * @return 帧公共项与帧键。
 */
inline TofFrameError MakeTofFrameError(const TofNoiseProfile &_profile,
                                       uint64_t _seed, int _vehicle, int _sensor,
                                       int64_t _sim_time_us)
{
  TofFrameError frame;
  frame.frame_key = TofFrameKey(_seed, _vehicle, _sensor, _sim_time_us);
  frame.frame_bias_m = _profile.frame_bias_std_m * TofNormal(frame.frame_key ^ 0x21ULL);
  return frame;
}

/**
 * @brief 判断像素是否落在深度突变边缘，边缘判定使用未加噪的源深度。
 * @param _depth_m 64 像素源轴向深度。
 * @param _source_valid 64 像素源有效性。
 * @param _pixel_index 行优先像素序号。
 * @param _threshold_m 四邻域深度差门限，单位 m。
 * @return 存在超过门限的有效邻居时返回 true。
 */
inline bool TofIsEdgePixel(const std::array<double, kTofPixelCount> &_depth_m,
                           const std::array<bool, kTofPixelCount> &_source_valid,
                           int _pixel_index, double _threshold_m)
{
  if (_pixel_index < 0 || _pixel_index >= kTofPixelCount || !_source_valid[_pixel_index])
    return false;
  const int x = _pixel_index % 8;
  const int y = _pixel_index / 8;
  constexpr std::array<std::array<int, 2>, 4> kNeighbors{{{{-1, 0}}, {{1, 0}}, {{0, -1}}, {{0, 1}}}};
  for (const auto &neighbor : kNeighbors)
  {
    const int nx = x + neighbor[0];
    const int ny = y + neighbor[1];
    if (nx < 0 || nx >= 8 || ny < 0 || ny >= 8)
      continue;
    const int index = ny * 8 + nx;
    if (_source_valid[index] &&
        std::abs(_depth_m[_pixel_index] - _depth_m[index]) > _threshold_m)
      return true;
  }
  return false;
}

/**
 * @brief 对单个像素施加误差模型。
 * @param _profile 误差剖面。
 * @param _device 设备固定项。
 * @param _frame 帧公共项。
 * @param _true_depth_m 未加噪的轴向深度，单位 m。
 * @param _pixel_index 行优先像素序号。
 * @param _min_range_m 有效量程下限。
 * @param _max_range_m 有效量程上限。
 * @param _range_policy 量程外处理方式。
 * @param _edge_dropped 该像素是否已被边缘模型判为无效，默认 false。
 * @return 像素结果；无效时 distance_m 为 0。
 */
inline TofPixelResult ApplyTofPixel(const TofNoiseProfile &_profile,
                                    const TofNoiseDevice &_device,
                                    const TofFrameError &_frame,
                                    double _true_depth_m, int _pixel_index,
                                    double _min_range_m = kDefaultTofMinRangeM,
                                    double _max_range_m = kDefaultTofMaxRangeM,
                                    TofRangePolicy _range_policy = TofRangePolicy::kMarkInvalid,
                                    bool _edge_dropped = false)
{
  TofPixelResult result;
  const uint64_t key = TofPixelKey(_frame.frame_key, _pixel_index);
  if (_edge_dropped || TofUniform(key ^ 0x31ULL) < _profile.drop_probability)
    return result;
  const double sigma = _profile.noise_std_m +
                       _profile.range_sigma_per_m * _true_depth_m;
  const double measured = _true_depth_m * _device.scale + _device.bias_m +
                          _frame.frame_bias_m + sigma * TofNormal(key ^ 0x32ULL);
  if (!std::isfinite(measured))
    return result;
  if (measured < _min_range_m || measured > _max_range_m)
  {
    if (_range_policy == TofRangePolicy::kMarkInvalid)
      return result;
    result.distance_m = std::fmin(std::fmax(measured, _min_range_m), _max_range_m);
    result.valid = true;
    return result;
  }
  result.distance_m = measured;
  result.valid = true;
  return result;
}

/**
 * @brief 对整帧 64 像素施加误差模型。
 * @param _profile 误差剖面。
 * @param _edge 边缘失效策略；enabled 为 false 时不做边缘判定。
 * @param _seed 全局种子。
 * @param _vehicle 车辆编号。
 * @param _sensor 传感器编号。
 * @param _sim_time_us 帧时间戳，单位 us。
 * @param _true_depth_m 64 像素源轴向深度。
 * @param _source_valid 64 像素源有效性；false 的像素直接判为无效。
 * @param _min_range_m 有效量程下限。
 * @param _max_range_m 有效量程上限。
 * @param _range_policy 量程外处理方式。
 * @return 整帧输出。
 */
inline TofFrameResult ApplyTofFrame(const TofNoiseProfile &_profile,
                                    const TofEdgePolicy &_edge, uint64_t _seed,
                                    int _vehicle, int _sensor, int64_t _sim_time_us,
                                    const std::array<double, kTofPixelCount> &_true_depth_m,
                                    const std::array<bool, kTofPixelCount> &_source_valid,
                                    double _min_range_m = kDefaultTofMinRangeM,
                                    double _max_range_m = kDefaultTofMaxRangeM,
                                    TofRangePolicy _range_policy = TofRangePolicy::kMarkInvalid)
{
  const TofNoiseDevice device = MakeTofNoiseDevice(_profile, _seed, _vehicle, _sensor);
  const TofFrameError frame = MakeTofFrameError(_profile, _seed, _vehicle, _sensor,
                                                _sim_time_us);
  TofFrameResult result;
  for (int index = 0; index < kTofPixelCount; ++index)
  {
    if (!_source_valid[index])
      continue;
    bool edge_dropped = false;
    if (_edge.enabled)
    {
      const bool edge = TofIsEdgePixel(_true_depth_m, _source_valid, index,
                                       _edge.mixed_pixel_threshold_m);
      edge_dropped = edge &&
                     TofUniform(TofPixelKey(frame.frame_key, index) ^ 0x41ULL) <
                         _edge.mixed_pixel_probability;
    }
    const TofPixelResult pixel = ApplyTofPixel(_profile, device, frame,
                                               _true_depth_m[index], index,
                                               _min_range_m, _max_range_m,
                                               _range_policy, edge_dropped);
    result.distance_m[index] = pixel.distance_m;
    result.valid[index] = pixel.valid;
  }
  return result;
}

}  // namespace drone_sim::research

#endif  // DRONE_SIM_RESEARCH_TOF_ERROR_MODEL_HPP
