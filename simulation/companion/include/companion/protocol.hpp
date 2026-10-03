/**
 * @file protocol.hpp
 * @brief 传感器、队友估计和控制意图的 JSON 数据报接口。
 */
#pragma once

#include <optional>
#include <string>

#include "companion/types.hpp"

namespace companion {

/**
 * @brief 解析 schema=1 的 JSON 8×8 ToF 深度报文。
 * @param payload 完整 JSON 文本。
 * @param expected_vehicle 接收车辆编号，报文 vehicle 必须与之相同。
 * @return 解码样本；格式、字段检查失败或转换异常时返回 nullopt。
 * @note 测量时效与融合门限由 Estimator 进一步检查。
 */
std::optional<TofFrame> parseTofDatagram(const std::string& payload, int expected_vehicle);
/**
 * @brief 解析 schema=1 的 JSON 光流机体速度报文。
 * @param payload 完整 JSON 文本。
 * @param expected_vehicle 接收车辆编号，报文 vehicle 必须与之相同。
 * @return 解码样本；格式、字段检查失败或转换异常时返回 nullopt。
 * @note 测量时效与融合门限由 Estimator 进一步检查。
 */
std::optional<FlowSample> parseOpticalFlowDatagram(const std::string& payload,
                                                   int expected_vehicle);
/**
 * @brief 解析 schema=1 的 JSON 队友 UWB 距离和方位报文。
 * @param payload 完整 JSON 文本。
 * @param expected_vehicle 接收车辆编号，报文 vehicle 必须与之相同。
 * @return 解码样本；格式、字段检查失败或转换异常时返回 nullopt。
 * @note 测量时效与融合门限由 Estimator 进一步检查。
 */
std::optional<UwbPeerSample> parseUwbDatagram(const std::string& payload,
                                              int expected_vehicle);
/**
 * @brief 解析 schema=1 的 JSON 固定基站距离报文。
 * @param payload 完整 JSON 文本。
 * @param expected_vehicle 接收车辆编号，报文 vehicle 必须与之相同。
 * @return 解码样本；格式、字段检查失败或转换异常时返回 nullopt。
 * @note 测量时效与融合门限由 Estimator 进一步检查。
 */
std::optional<AnchorRangeSample> parseAnchorRangeDatagram(const std::string& payload,
                                                          int expected_vehicle);
/**
 * @brief 解析 schema=1 的 JSON SIM_HEIGHT_ASSIST 辅助高度报文。
 * @param payload 完整 JSON 文本。
 * @param expected_vehicle 接收车辆编号，报文 vehicle 必须与之相同。
 * @return 解码样本；格式、字段检查失败或转换异常时返回 nullopt。
 * @note 测量时效与融合门限由 Estimator 进一步检查。
 */
std::optional<HeightAssistSample> parseHeightAssistDatagram(
    const std::string& payload, int expected_vehicle);
/**
 * @brief 解析 schema=1 的 JSON 队友估计及参考传播路径报文。
 * @param payload 完整 JSON 文本。
 * @param expected_vehicle 本机编号；拒绝本机发出的估计，保留其他车辆。
 * @return 解码样本；格式、字段检查失败或转换异常时返回 nullopt。
 * @note 测量时效与融合门限由 Estimator 进一步检查。
 */
std::optional<PeerEstimate> parsePeerEstimateDatagram(const std::string& payload,
                                                      int expected_vehicle);
/**
 * @brief 序列化位置估计及参考信息为 schema=1 JSON。
 * @param estimate 待发送的 ENU 估计。
 * @return JSON 文本；pose_covariance 仅发送 [x,y,z] 的 3×3 块，按行展开。
 */
std::string serializeEstimateDatagram(const Estimate& estimate);
/**
 * @brief 序列化机体系控制意图为 schema=1 JSON。
 * @param intention FLU 速度、角速度和目标/航点状态。
 * @return 可作为一个数据报负载的 JSON 文本。
 */
std::string serializeControlDatagram(const ControlIntention& intention);

}  // namespace companion
