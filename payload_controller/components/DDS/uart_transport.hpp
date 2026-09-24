/**
 * @file
 * @brief MicroXRCEDDSClient doesn't have a pre implemented serial transport backend for ESP-IDF, so we must implement a custom UART transport
 */

#pragma once
#include <uxr/client/client.h>



/// Configuration parameters for a UART-based transport for ESP-IDF.
struct UartTransportConfig {
  uint32_t uart_num;
  uint32_t tx_pin;
  uint32_t rx_pin;
  uint32_t baud_rate;
};

/// Opens and initializes the UART transport. Passed into uxr_set_custom_transport_callbacks()
bool uart_open(uxrCustomTransport* transport);

/// Writes a buffer of data to the UART transport. Passed into uxr_set_custom_transport_callbacks()
size_t uart_write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error_code);

/// Reads data from the UART transport into the provided buffer. Passed into uxr_set_custom_transport_callbacks()
size_t uart_read( uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout, uint8_t* error_code);

/// Closes and cleans up the UART transport. Passed into uxr_set_custom_transport_callbacks()
bool uart_close(uxrCustomTransport* transport);
