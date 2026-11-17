#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace ibus {

/// @brief FlySky i-Bus 流式解析器：逐字节喂入，收齐并校验一帧后输出 14 通道。
class Parser {
public:
    static constexpr std::size_t kChannels = 14;
    static constexpr std::size_t kFrameLength = 32;

    /// @brief 喂入一个字节；整帧校验通过时写入 channels 并返回 true。
    bool parse(std::uint8_t byte, std::span<std::uint16_t, kChannels> channels) {
        if (length_ == 0 && byte != 0x20) return false;
        if (length_ == 1 && byte != 0x40) {
            length_ = 0;
            return false;
        }

        buffer_[length_++] = byte;
        if (length_ < kFrameLength) return false;
        length_ = 0;

        std::uint16_t sum = 0;
        for (std::size_t i = 0; i < kFrameLength - 2; ++i) {
            sum = static_cast<std::uint16_t>(sum + buffer_[i]);
        }
        const std::uint16_t checksum =
            static_cast<std::uint16_t>(buffer_[30] | (buffer_[31] << 8));
        if (static_cast<std::uint16_t>(0xFFFF - sum) != checksum) return false;

        for (std::size_t i = 0; i < kChannels; ++i) {
            channels[i] = static_cast<std::uint16_t>(
                buffer_[2 + i * 2] | (buffer_[3 + i * 2] << 8));
        }
        return true;
    }

private:
    std::array<std::uint8_t, kFrameLength> buffer_{};
    std::size_t length_{0};
};

}  // namespace ibus
