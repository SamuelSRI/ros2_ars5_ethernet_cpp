#include "ros2_ars5_ethernet_cpp/udp_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

UdpReceiver::UdpReceiver(
  const std::string & listen_ip,
  int listen_port,
  const std::string & multicast_group,
  const std::string & interface_ip)
{
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (socket_fd_ < 0) {
    throw std::runtime_error("Failed to create UDP socket");
  }

  int reuse = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

#ifdef SO_REUSEPORT
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

  int rcvbuf = 64 * 1024 * 1024;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(listen_port));

  if (listen_ip == "0.0.0.0") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    if (inet_pton(AF_INET, listen_ip.c_str(), &addr.sin_addr) <= 0) {
      close(socket_fd_);
      throw std::runtime_error("Invalid listen IP");
    }
  }

  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(socket_fd_);
    throw std::runtime_error("Failed to bind UDP socket");
  }

  ip_mreq mreq {};
  mreq.imr_multiaddr.s_addr = inet_addr(multicast_group.c_str());
  mreq.imr_interface.s_addr = inet_addr(interface_ip.c_str());

  if (setsockopt(socket_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    close(socket_fd_);
    throw std::runtime_error("Failed to join multicast group");
  }

  timeval timeout {};
  timeout.tv_sec = 0;
  timeout.tv_usec = 10000;

  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

UdpReceiver::~UdpReceiver()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

bool UdpReceiver::receive(
  std::vector<uint8_t> & buffer,
  std::string & sender_ip,
  int & sender_port)
{
  buffer.resize(65535);

  sockaddr_in sender {};
  socklen_t sender_len = sizeof(sender);

  const ssize_t received = recvfrom(
    socket_fd_,
    buffer.data(),
    buffer.size(),
    0,
    reinterpret_cast<sockaddr *>(&sender),
    &sender_len
  );

  if (received <= 0) {
    return false;
  }

  buffer.resize(static_cast<size_t>(received));

  char ip_buffer[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &sender.sin_addr, ip_buffer, sizeof(ip_buffer));

  sender_ip = std::string(ip_buffer);
  sender_port = ntohs(sender.sin_port);

  return true;
}
