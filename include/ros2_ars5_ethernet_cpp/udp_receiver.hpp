#pragma once

#include <cstdint>
#include <string>
#include <vector>

class UdpReceiver
{
public:
  UdpReceiver(
    const std::string & listen_ip,
    int listen_port,
    const std::string & multicast_group,
    const std::string & interface_ip);

  ~UdpReceiver();

  bool receive(
    std::vector<uint8_t> & buffer,
    std::string & sender_ip,
    int & sender_port);

private:
  int socket_fd_;
};
