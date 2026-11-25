#pragma once

#include "frame_codec.hpp"

#include <bit>
#include <cstdint>

namespace motor_detail {

// 角度、转速和最后三个字段的字节布局相同，反馈含义由各电调驱动解释。
using DjiMotorFeedback = frame::Frame<
    frame::Field<std::uint16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::uint8_t>, frame::Field<std::uint8_t>>;

using DjiMotorOutputs = frame::Frame<
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>>;

} // namespace motor_detail
