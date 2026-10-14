/** @file color_threshold.hpp
 * @brief 8 位通道严格比例判据的最小通过值，保留浮点乘法在整数边界处的取值。
 */
#pragma once
#include <array>
#include <cstdint>

namespace companion::detail {
/** @param ratio 有限正比例；返回 256 表示任意 8 位通道值都不能通过。 */
constexpr std::array<uint16_t,256> colorThresholds(double ratio){
  std::array<uint16_t,256> result{};
  for(unsigned channel=0;channel<result.size();++channel){
    if(channel&&ratio>=256){result[channel]=256;continue;}
    const double product=ratio*channel;
    result[channel]=product<256?static_cast<uint16_t>(product)+1:256;
  }
  return result;
}
}
