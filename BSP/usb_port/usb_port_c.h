#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_port_on_receive_from_isr(const uint8_t* data, uint32_t size);
void usb_port_on_session_changed_from_isr(void);

#ifdef __cplusplus
}
#endif
