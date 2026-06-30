#include "ros2_ars5_ethernet_cpp/ars5_decoder.hpp"
#include "ros2_ars5_ethernet_cpp/udp_receiver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

class Ars5Node : public rclcpp::Node
{
public:
  Ars5Node()
  : Node("ars5_node")
  {
    listen_ip_ = declare_parameter<std::string>("listen_ip", "0.0.0.0");
    listen_port_ = declare_parameter<int>("listen_port", 42102);
    multicast_group_ = declare_parameter<std::string>("multicast_group", "224.0.2.2");
    interface_ip_ = declare_parameter<std::string>("interface_ip", "10.13.1.100");
    max_packets_per_cycle_ = declare_parameter<int>("max_packets_per_cycle", 64);

    receiver_ = std::make_unique<UdpReceiver>(
      listen_ip_,
      listen_port_,
      multicast_group_,
      interface_ip_
    );

    timer_ = create_wall_timer(
      1ms,
      std::bind(&Ars5Node::read_loop, this)
    );

    RCLCPP_INFO(
      get_logger(),
      "ARS5 node listening on %s:%d multicast=%s interface_ip=%s",
      listen_ip_.c_str(),
      listen_port_,
      multicast_group_.c_str(),
      interface_ip_.c_str()
    );
  }

private:
  using PointCloudPublisher =
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

  void read_loop()
  {
    std::vector<uint8_t> packet;
    std::string sender_ip;
    int sender_port = 0;

    int packets_read = 0;

    while (packets_read < max_packets_per_cycle_ &&
           receiver_->receive(packet, sender_ip, sender_port))
    {
      packets_read++;

      const uint32_t radar_id = radar_id_from_ip(sender_ip);
      auto points = decoder_.decode_packet(packet);

      if (!points.empty()) {
        publish_pointcloud(points, radar_id);
      }
    }
  }

  uint32_t radar_id_from_ip(const std::string & ip) const
  {
    const auto pos = ip.find_last_of('.');

    if (pos == std::string::npos) {
      return 0;
    }

    try {
      return static_cast<uint32_t>(std::stoi(ip.substr(pos + 1)));
    } catch (...) {
      return 0;
    }
  }

  std::string topic_name_from_radar_id(uint32_t radar_id) const
  {
    return "/radar/r" + std::to_string(radar_id) + "/points";
  }

  std::string frame_name_from_radar_id(uint32_t radar_id) const
  {
    return "radar_" + std::to_string(radar_id) + "_link";
  }

  PointCloudPublisher get_or_create_publisher(uint32_t radar_id)
  {
    const auto it = publishers_.find(radar_id);

    if (it != publishers_.end()) {
      return it->second;
    }

    const std::string topic_name = topic_name_from_radar_id(radar_id);

    auto pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_name,
      rclcpp::SensorDataQoS()
    );

    publishers_[radar_id] = pub;

    RCLCPP_INFO(
      get_logger(),
      "Created publisher for radar_id=%u on topic %s with frame %s",
      radar_id,
      topic_name.c_str(),
      frame_name_from_radar_id(radar_id).c_str()
    );

    return pub;
  }

  void write_float(std::vector<uint8_t> & data, size_t offset, float value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(float));
  }

  void write_uint32(std::vector<uint8_t> & data, size_t offset, uint32_t value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(uint32_t));
  }

  void publish_pointcloud(const std::vector<RadarPoint> & points, uint32_t radar_id)
  {
    auto pub = get_or_create_publisher(radar_id);

    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_name_from_radar_id(radar_id);

    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = false;

    msg.fields.resize(8);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "velocity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.fields[4].name = "range";
    msg.fields[4].offset = 16;
    msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[4].count = 1;

    msg.fields[5].name = "azimuth";
    msg.fields[5].offset = 20;
    msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[5].count = 1;

    msg.fields[6].name = "elevation";
    msg.fields[6].offset = 24;
    msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[6].count = 1;

    msg.fields[7].name = "radar_id";
    msg.fields[7].offset = 28;
    msg.fields[7].datatype = sensor_msgs::msg::PointField::UINT32;
    msg.fields[7].count = 1;

    msg.point_step = 32;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    for (size_t i = 0; i < points.size(); ++i) {
      const size_t offset = i * msg.point_step;

      write_float(msg.data, offset + 0, points[i].x);
      write_float(msg.data, offset + 4, points[i].y);
      write_float(msg.data, offset + 8, points[i].z);
      write_float(msg.data, offset + 12, points[i].velocity);
      write_float(msg.data, offset + 16, points[i].range);
      write_float(msg.data, offset + 20, points[i].azimuth);
      write_float(msg.data, offset + 24, points[i].elevation);
      write_uint32(msg.data, offset + 28, radar_id);
    }

    pub->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "published %zu points on /radar/r%u/points frame=radar_%u_link",
      points.size(),
      radar_id,
      radar_id
    );
  }

  std::string listen_ip_;
  int listen_port_;
  std::string multicast_group_;
  std::string interface_ip_;
  int max_packets_per_cycle_;

  std::unique_ptr<UdpReceiver> receiver_;
  Ars5Decoder decoder_;

  std::unordered_map<uint32_t, PointCloudPublisher> publishers_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Ars5Node>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}