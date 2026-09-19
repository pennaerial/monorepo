#include "dds_uart_transport.hpp"

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"

namespace
{

// A byte-stream UART needs XRCE framing because it does not preserve packet boundaries.
constexpr bool USE_XRCE_FRAMING = true;
// Two maximum-sized XRCE frames allow one frame to arrive while the client consumes another.
constexpr int RX_BUFFER_SIZE_BYTES = 2 * UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
// Micro-XRCE-DDS custom transport callbacks use zero for success and nonzero for failure.
constexpr uint8_t TRANSPORT_SUCCESS = 0;
constexpr uint8_t TRANSPORT_ERROR = 1;

bool uart_open(uxrCustomTransport* transport)
{
  // Micro-XRCE-DDS invokes this callback when the client transport is initialized.
  (void)transport;
  const uart_config_t config{
      .baud_rate = dds_uart_transport::BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {},
  };

  if (uart_param_config(dds_uart_transport::PORT, &config) != ESP_OK ||
      uart_set_pin(
          dds_uart_transport::PORT, dds_uart_transport::TX_GPIO, dds_uart_transport::RX_GPIO, UART_PIN_NO_CHANGE,
          UART_PIN_NO_CHANGE
      ) != ESP_OK ||
      uart_driver_install(dds_uart_transport::PORT, RX_BUFFER_SIZE_BYTES, 0, 0, nullptr, 0) != ESP_OK) {
    return false;
  }
  if (uart_flush_input(dds_uart_transport::PORT) != ESP_OK) {
    uart_driver_delete(dds_uart_transport::PORT);
    return false;
  }
  return true;
}

bool uart_close(uxrCustomTransport* transport)
{
  (void)transport;
  return uart_driver_delete(dds_uart_transport::PORT) == ESP_OK;
}

size_t uart_write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error_code)
{
  (void)transport;
  const int written = uart_write_bytes(dds_uart_transport::PORT, buffer, length);
  const bool complete = written >= 0 && static_cast<size_t>(written) == length;
  *error_code = complete ? TRANSPORT_SUCCESS : TRANSPORT_ERROR;
  return written < 0 ? 0 : static_cast<size_t>(written);
}

size_t uart_read(uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout, uint8_t* error_code)
{
  (void)transport;
  const int read = uart_read_bytes(dds_uart_transport::PORT, buffer, length, pdMS_TO_TICKS(timeout));
  *error_code = read < 0 ? TRANSPORT_ERROR : TRANSPORT_SUCCESS;
  return read < 0 ? 0 : static_cast<size_t>(read);
}

}  // namespace

namespace dds_uart_transport
{

bool initialize(uxrCustomTransport& transport)
{
  // Register the ESP-IDF UART adapter before asking Micro-XRCE-DDS to open it.
  uxr_set_custom_transport_callbacks(&transport, USE_XRCE_FRAMING, uart_open, uart_close, uart_write, uart_read);
  return uxr_init_custom_transport(&transport, nullptr);
}

bool close(uxrCustomTransport& transport)
{
  return uxr_close_custom_transport(&transport);
}

}  // namespace dds_uart_transport
