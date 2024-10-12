/**
 * @file serial_driver.hpp
 * @author Dmitry Ponomarev
 * @author xxlshi 
 */

#ifndef SERIAL_DRIVER_HPP
#define SERIAL_DRIVER_HPP

#include <stdint.h>
#include <string>

class SerialDriver
{
public:
  SerialDriver() = default;

  virtual ~SerialDriver();

  void init(const std::string & port, uint32_t baudrate);

  int spin(uint8_t recv_buf[], size_t max_buf_size);

private:
  int _fd_serial_port;
};

#endif  // SERIAL_DRIVER_HPP
