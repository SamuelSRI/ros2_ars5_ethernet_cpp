#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Ars5FusionNode : public rclcpp::Node
{
public:
  Ars5FusionNode()
  : Node("ars5_fusion_node")
  {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "base_link");
    output_topic_ = declare_parameter<std::string>("output_topic", "/radar/points_fused");
    radar_id_start_ = declare_parameter<int>("radar_id_start", 110);
    radar_id_end_ = declare_parameter<int>("radar_id_end", 129);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", 0.3);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    fused_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS()
    );

    for (int id = radar_id_start_; id <= radar_id_end_; ++id) {
      const std::string topic = "/radar/r" + std::to_string(id) + "/points";

      auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic,
        rclcpp::SensorDataQoS(),
        [this, id](sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          this->cloud_callback(id, msg);
        }
      );

      subscribers_.push_back(sub);

      RCLCPP_INFO(
        get_logger(),
        "Subscribed to %s",
        topic.c_str()
      );
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Ars5FusionNode::publish_fused_cloud, this)
    );

    RCLCPP_INFO(
      get_logger(),
      "ARS5 fusion node started. fixed_frame=%s output=%s",
      fixed_frame_.c_str(),
      output_topic_.c_str()
    );
  }

private:
  struct StoredCloud
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    rclcpp::Time stamp;
  };

  struct SimplePoint
  {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float velocity = 0.0f;
    float range = 0.0f;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    uint32_t radar_id = 0;
  };

  void cloud_callback(
    int radar_id,
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    StoredCloud stored;
    stored.cloud = msg;
    stored.stamp = now();

    latest_clouds_[radar_id] = stored;
  }

  int field_offset(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & name) const
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name) {
        return static_cast<int>(field.offset);
      }
    }

    return -1;
  }

  float read_float(
    const std::vector<uint8_t> & data,
    size_t offset) const
  {
    float value;
    std::memcpy(&value, data.data() + offset, sizeof(float));
    return value;
  }

  uint32_t read_uint32(
    const std::vector<uint8_t> & data,
    size_t offset) const
  {
    uint32_t value;
    std::memcpy(&value, data.data() + offset, sizeof(uint32_t));
    return value;
  }

  void write_float(
    std::vector<uint8_t> & data,
    size_t offset,
    float value) const
  {
    std::memcpy(data.data() + offset, &value, sizeof(float));
  }

  void write_uint32(
    std::vector<uint8_t> & data,
    size_t offset,
    uint32_t value) const
  {
    std::memcpy(data.data() + offset, &value, sizeof(uint32_t));
  }

  SimplePoint transform_point(
    const SimplePoint & p,
    const geometry_msgs::msg::TransformStamped & tf) const
  {
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;

    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;

    const double x = p.x;
    const double y = p.y;
    const double z = p.z;

    // Rotation matrix from quaternion.
    const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r01 = 2.0 * (qx * qy - qz * qw);
    const double r02 = 2.0 * (qx * qz + qy * qw);

    const double r10 = 2.0 * (qx * qy + qz * qw);
    const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r12 = 2.0 * (qy * qz - qx * qw);

    const double r20 = 2.0 * (qx * qz - qy * qw);
    const double r21 = 2.0 * (qy * qz + qx * qw);
    const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

    SimplePoint out = p;

    out.x = static_cast<float>(r00 * x + r01 * y + r02 * z + t.x);
    out.y = static_cast<float>(r10 * x + r11 * y + r12 * z + t.y);
    out.z = static_cast<float>(r20 * x + r21 * y + r22 * z + t.z);

    return out;
  }

  std::vector<SimplePoint> extract_and_transform_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    std::vector<SimplePoint> output;

    if (cloud.width == 0 || cloud.point_step == 0) {
      return output;
    }

    const int x_off = field_offset(cloud, "x");
    const int y_off = field_offset(cloud, "y");
    const int z_off = field_offset(cloud, "z");
    const int velocity_off = field_offset(cloud, "velocity");
    const int range_off = field_offset(cloud, "range");
    const int azimuth_off = field_offset(cloud, "azimuth");
    const int elevation_off = field_offset(cloud, "elevation");
    const int radar_id_off = field_offset(cloud, "radar_id");

    if (x_off < 0 || y_off < 0 || z_off < 0) {
      return output;
    }

    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_->lookupTransform(
        fixed_frame_,
        cloud.header.frame_id,
        rclcpp::Time(0)
      );
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "No TF from %s to %s: %s",
        cloud.header.frame_id.c_str(),
        fixed_frame_.c_str(),
        e.what()
      );

      return output;
    }

    const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    output.reserve(point_count);

    for (size_t i = 0; i < point_count; ++i) {
      const size_t base = i * cloud.point_step;

      if (base + cloud.point_step > cloud.data.size()) {
        break;
      }

      SimplePoint p;

      p.x = read_float(cloud.data, base + x_off);
      p.y = read_float(cloud.data, base + y_off);
      p.z = read_float(cloud.data, base + z_off);

      if (velocity_off >= 0) {
        p.velocity = read_float(cloud.data, base + velocity_off);
      }

      if (range_off >= 0) {
        p.range = read_float(cloud.data, base + range_off);
      }

      if (azimuth_off >= 0) {
        p.azimuth = read_float(cloud.data, base + azimuth_off);
      }

      if (elevation_off >= 0) {
        p.elevation = read_float(cloud.data, base + elevation_off);
      }

      if (radar_id_off >= 0) {
        p.radar_id = read_uint32(cloud.data, base + radar_id_off);
      }

      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      output.push_back(transform_point(p, tf));
    }

    return output;
  }

  void publish_fused_cloud()
  {
    std::vector<SimplePoint> fused_points;

    const rclcpp::Time current_time = now();

    for (const auto & kv : latest_clouds_) {
      const auto & stored = kv.second;

      if (!stored.cloud) {
        continue;
      }

      const double age = (current_time - stored.stamp).seconds();

      if (age > max_cloud_age_sec_) {
        continue;
      }

      auto transformed = extract_and_transform_cloud(*stored.cloud);

      fused_points.insert(
        fused_points.end(),
        transformed.begin(),
        transformed.end()
      );
    }

    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = current_time;
    msg.header.frame_id = fixed_frame_;

    msg.height = 1;
    msg.width = static_cast<uint32_t>(fused_points.size());
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

    for (size_t i = 0; i < fused_points.size(); ++i) {
      const size_t offset = i * msg.point_step;

      write_float(msg.data, offset + 0, fused_points[i].x);
      write_float(msg.data, offset + 4, fused_points[i].y);
      write_float(msg.data, offset + 8, fused_points[i].z);
      write_float(msg.data, offset + 12, fused_points[i].velocity);
      write_float(msg.data, offset + 16, fused_points[i].range);
      write_float(msg.data, offset + 20, fused_points[i].azimuth);
      write_float(msg.data, offset + 24, fused_points[i].elevation);
      write_uint32(msg.data, offset + 28, fused_points[i].radar_id);
    }

    fused_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "published fused cloud with %zu points in frame %s",
      fused_points.size(),
      fixed_frame_.c_str()
    );
  }

  std::string fixed_frame_;
  std::string output_topic_;
  int radar_id_start_;
  int radar_id_end_;
  double publish_rate_hz_;
  double max_cloud_age_sec_;

  std::unordered_map<int, StoredCloud> latest_clouds_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Ars5FusionNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
