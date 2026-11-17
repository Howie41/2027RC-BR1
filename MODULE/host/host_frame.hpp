#pragma once

#include "frame_codec.hpp"

namespace host_comm {

inline constexpr std::size_t kMaxPayload = 64;

using CheckedRegion = frame::RegionTag<0>;
using PayloadRegion = frame::RegionTag<1>;

/// @brief 保持原上下位机协议，载荷长度与 CRC 由 frame_codec 自动处理。
template <std::uint8_t Id, typename... Fields>
struct HostFrame
    : frame::Frame<
          frame::Header<0xFF, 0xFE>, frame::Begin<CheckedRegion>, frame::Header<Id>,
          frame::Length<std::uint8_t, frame::LengthOf<PayloadRegion>>, frame::Begin<PayloadRegion>,
          Fields..., frame::End<PayloadRegion>, frame::End<CheckedRegion>,
          frame::Crc<CheckedRegion, frame::CrcType::Crc16Modbus>, frame::Footer<0xFD, 0xFB>> {
    static_assert((std::size_t{0} + ... + Fields::size()) <= kMaxPayload,
                  "上下位机消息载荷不得超过 64 字节");
};

} // namespace host_comm
