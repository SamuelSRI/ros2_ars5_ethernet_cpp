#include "ros2_ars5_ethernet_cpp/ars5_decoder.hpp"

#include <cmath>
#include <cstring>

uint32_t Ars5Decoder::read_be_u32(const std::vector<uint8_t> & data, size_t offset) const
{
  return
    (static_cast<uint32_t>(data[offset + 0]) << 24) |
    (static_cast<uint32_t>(data[offset + 1]) << 16) |
    (static_cast<uint32_t>(data[offset + 2]) << 8) |
    (static_cast<uint32_t>(data[offset + 3]));
}

float Ars5Decoder::read_be_float(const std::vector<uint8_t> & data, size_t offset) const
{
  const uint32_t raw = read_be_u32(data, offset);

  float value;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

std::vector<RadarPoint> Ars5Decoder::decode_packet(const std::vector<uint8_t> & packet)
{
  std::vector<RadarPoint> points;

  // ARS5-A SOME/IP packet observed:
  // total size = 35336 bytes
  // SOME/IP header = 16 bytes
  // payload length = 35328 bytes
  if (packet.size() != 35336) {
    return points;
  }

  const uint32_t someip_message_id = read_be_u32(packet, 0);
  const uint32_t someip_length = read_be_u32(packet, 4);

  const uint8_t protocol_version = packet[12];
  const uint8_t interface_version = packet[13];
  const uint8_t message_type = packet[14];
  const uint8_t return_code = packet[15];

  if (someip_message_id != 0x00000150) {
    return points;
  }

  if (someip_length + 8 != packet.size()) {
    return points;
  }

  if (protocol_version != 1 || interface_version != 1 || message_type != 2 || return_code != 0) {
    return points;
  }

  // Confirmed from binary dump:
  // 0x88 + 800 * 44 = 35336
  constexpr size_t detection_start = 0x88;
  constexpr size_t detection_stride = 44;
  constexpr size_t detection_count = 800;

  points.reserve(64);

  for (size_t i = 0; i < detection_count; ++i) {
    const size_t o = detection_start + i * detection_stride;

    if (o + detection_stride > packet.size()) {
      break;
    }

    // Current decoded offsets, validated from .bin test.
    const float azimuth = read_be_float(packet, o + 9);
    const float elevation = read_be_float(packet, o + 18);
    const float range = read_be_float(packet, o + 26);
    const float velocity = read_be_float(packet, o + 34);

    if (!std::isfinite(azimuth) ||
        !std::isfinite(elevation) ||
        !std::isfinite(range) ||
        !std::isfinite(velocity)) {
      continue;
    }

    // Lower filtering for debug / visualization.
    if (range < 0.1f || range > 300.0f) {
      continue;
    }

    // Wider azimuth range: about +/- 100 degrees.
    // You can remove this block completely if you want all azimuths.
    if (std::abs(azimuth) > 1.75f) {
      continue;
    }

    // Allow wider elevation, but still reject extreme false values.
    // 0.7 rad ~= 40 degrees.
    if (std::abs(elevation) > 0.7f) {
      continue;
    }

    RadarPoint p;
    p.detection_id = static_cast<uint32_t>(i);
    p.range = range;
    p.azimuth = azimuth;
    p.elevation = elevation;
    p.velocity = velocity;

    // 3D projection with elevation enabled.
    p.x = range * std::cos(elevation) * std::cos(azimuth);
    p.y = range * std::cos(elevation) * std::sin(azimuth);
    p.z = range * std::sin(elevation);

    points.push_back(p);
  }

  return points;
}
