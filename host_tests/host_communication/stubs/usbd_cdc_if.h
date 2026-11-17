#pragma once

#include "usb_device.h"

struct USBD_CDC_HandleTypeDef {
    std::uint32_t TxState{};
};

extern "C" std::uint8_t CDC_Transmit_HS(std::uint8_t* data, std::uint16_t size);
