#pragma once

#include <uxr/client/client.h>

#include "driver/gpio.h"
#include "driver/uart.h"

namespace dds_uart_transport
{

// Payload-controller wiring: UART1 transmits XRCE-DDS data on GPIO 39.
inline constexpr gpio_num_t TX_GPIO = GPIO_NUM_39;
// Payload-controller wiring: UART1 receives XRCE-DDS data on GPIO 38.
inline constexpr gpio_num_t RX_GPIO = GPIO_NUM_38;
// UART0 is reserved for the console, so XRCE-DDS uses the ESP32-S3's UART1 peripheral.
inline constexpr uart_port_t PORT = UART_NUM_1;
// This rate matches the companion computer's Micro-XRCE-DDS serial-agent configuration.
inline constexpr int BAUD_RATE = 115200;

bool initialize(uxrCustomTransport& transport);
bool close(uxrCustomTransport& transport);

}  // namespace dds_uart_transport
