#pragma once

#include "host_comm.hpp"
#include "host_msgs.hpp"
#include "usb_port.hpp"

namespace robot::host {

inline constexpr char kVisionPoseTopic[] = "vision_pose";
using Communication = host_comm::HostComm<host_comm::VisionPose>;

/// @brief 将消息完整入队，成功仅表示接受发送，不表示上位机已收到。
template <typename Message> bool send(const Message& message) {
    const auto bytes = Communication::build_frame(message);
    return UsbPort::instance().write_async(bytes);
}

} // namespace robot::host
