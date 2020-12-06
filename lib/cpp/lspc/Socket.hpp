#ifndef LSPC_BOOST_HPP
#define LSPC_BOOST_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>

#include "lspc/Packet.hpp"
#include "lspc/SocketBase.hpp"

namespace lspc {

class Socket : public SocketBase {
  // Buffer for receiving data on the serial link.
  std::array<uint8_t, 1> read_buffer;

  // Serial port
  boost::asio::io_service ioservice;
  boost::asio::serial_port controller_port =
      boost::asio::serial_port(ioservice);
  std::thread ioservice_thread;
  std::atomic<bool> serial_is_sending;

  // Process incoming data on serial link
  //
  // @brief Reads the serial buffer and dispatches the received payload to the
  // relevant message handling callback function.
  void processSerial(const boost::system::error_code &error,
                     std::size_t bytes_transferred);

  void serialWriteCallback(const boost::system::error_code &error,
                           size_t bytes_transferred);

 public:
  Socket();
  Socket(const std::string &com_port_name);

  ~Socket();

  void open(const std::string &com_port_name);

  bool isOpen();

  // Send a package with lspc
  //
  // @brief Sends a packaged buffer over the USB serial link.
  //
  // @param type The message type. This is user specific; any type between
  // 1-255.
  // @param payload A vector with the serialized payload to be sent.
  //
  // @return True if the packet was sent.
  bool send(uint8_t type, const std::vector<uint8_t> &payload) override;
};

}  // namespace lspc

#endif  // LSPC_HPP
