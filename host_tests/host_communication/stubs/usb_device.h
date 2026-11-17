#pragma once

#include <cstdint>

struct USBD_HandleTypeDef {
    void* pClassData{};
    std::uint8_t dev_state{};
};

inline constexpr std::uint8_t USBD_STATE_CONFIGURED = 3;
inline constexpr std::uint8_t USBD_OK = 0;
inline constexpr std::uint8_t USBD_BUSY = 1;
inline constexpr std::uint8_t USBD_FAIL = 2;

extern "C" USBD_HandleTypeDef hUsbDeviceHS;
