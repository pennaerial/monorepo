#include "uart_transport.hpp"

bool uart_open(uxrCustomTransport* transport)
{
  // TODO
  return true;
}

size_t uart_write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error_code)
{
  // TODO
  return 0;
}

size_t uart_read(uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout, uint8_t* error_code)
{
  // TODO
  return 0;
}

bool uart_close(uxrCustomTransport* transport)
{
  // TODO
  return true;
}
