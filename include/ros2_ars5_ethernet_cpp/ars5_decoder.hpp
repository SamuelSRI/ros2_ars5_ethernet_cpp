#pragma once

#include <cstdint>
#include <vector>

struct RadarPoint
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float range = 0.0f;
  float azimuth = 0.0f;
  float elevation = 0.0f;
  float velocity = 0.0f;

  uint32_t detection_id = 0;
};

class Ars5Decoder
{
public:
  std::vector<RadarPoint> decode_packet(const std::vector<uint8_t> & packet);

private:
  uint32_t read_be_u32(const std::vector<uint8_t> & data, size_t offset) const;
  float read_be_float(const std::vector<uint8_t> & data, size_t offset) const;
};
