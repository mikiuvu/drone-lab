#pragma once

#include <optional>
#include <string>

#include "companion/types.hpp"

namespace companion {

std::optional<TofFrame> parseTofDatagram(const std::string& payload, int expected_vehicle);
std::optional<FlowSample> parseOpticalFlowDatagram(const std::string& payload,
                                                   int expected_vehicle);
std::optional<UwbPeerSample> parseUwbDatagram(const std::string& payload,
                                              int expected_vehicle);
std::optional<AnchorRangeSample> parseAnchorRangeDatagram(const std::string& payload,
                                                          int expected_vehicle);
std::optional<HeightAssistSample> parseHeightAssistDatagram(
    const std::string& payload, int expected_vehicle);
std::optional<PeerEstimate> parsePeerEstimateDatagram(const std::string& payload,
                                                      int expected_vehicle);
std::string serializeEstimateDatagram(const Estimate& estimate);
std::string serializeControlDatagram(const ControlIntention& intention);

}  // namespace companion
