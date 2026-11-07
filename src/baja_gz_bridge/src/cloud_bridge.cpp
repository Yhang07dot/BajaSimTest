#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using Cloud = sensor_msgs::msg::PointCloud2;

// Keep native transport lifetime inside main, before ROS objects are destroyed.
// No Python callback or interpreter finalization is involved.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cloud_bridge");
  auto pub = node->create_publisher<Cloud>("/sensors/lidar/points", rclcpp::SensorDataQoS());
  const std::string topic = "/sensors/lidar/scan/points";
  {
    gz::transport::Node transport;
    std::function<void(const gz::msgs::PointCloudPacked &)> receive =
      [pub](const gz::msgs::PointCloudPacked &input) {
        if (!rclcpp::ok() || !input.width() || !input.height()) return;
        const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
        std::array<int, 4> offsets{-1, -1, -1, -1};
        for (const auto &field : input.field()) {
          for (std::size_t i = 0; i < names.size(); ++i) {
            if (field.name() == names[i]) offsets[i] = field.offset();
          }
        }
        for (int offset : offsets) {
          if (offset < 0 || static_cast<uint32_t>(offset + 4) > input.point_step()) return;
        }
        if (input.row_step() < input.width() * input.point_step() ||
            input.data().size() < static_cast<std::size_t>(input.row_step()) * input.height()) return;
        Cloud out;
        out.header.stamp.sec = input.header().stamp().sec();
        out.header.stamp.nanosec = input.header().stamp().nsec();
        out.header.frame_id = "lidar_link";
        out.width = input.width(); out.height = input.height();
        out.point_step = 20; out.row_step = 20 * out.width;
        out.is_bigendian = input.is_bigendian(); out.is_dense = false;
        for (std::size_t i = 0; i < names.size(); ++i) {
          sensor_msgs::msg::PointField field;
          field.name = names[i]; field.offset = 4 * i;
          // Gazebo 8 may label float bytes as UINT32; standardize known fields.
          field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
          out.fields.push_back(field);
        }
        sensor_msgs::msg::PointField ring;
        ring.name = "ring"; ring.offset = 16;
        ring.datatype = sensor_msgs::msg::PointField::UINT16; ring.count = 1;
        out.fields.push_back(ring);
        out.data.resize(static_cast<std::size_t>(out.row_step) * out.height, 0);
        for (uint32_t row = 0; row < out.height; ++row) {
          for (uint32_t column = 0; column < out.width; ++column) {
            const char *source = input.data().data() + row * input.row_step() + column * input.point_step();
            uint8_t *target = out.data.data() + row * out.row_step + column * out.point_step;
            for (std::size_t i = 0; i < names.size(); ++i) std::memcpy(target + 4 * i, source + offsets[i], 4);
            target[16] = input.is_bigendian() ? row >> 8 : row & 0xff;
            target[17] = input.is_bigendian() ? row & 0xff : row >> 8;
          }
        }
        if (rclcpp::ok()) pub->publish(std::move(out));
      };
    if (!transport.Subscribe(topic, receive)) {
      RCLCPP_ERROR(node->get_logger(), "Could not subscribe Gazebo cloud");
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::spin(node);
    transport.Unsubscribe(topic);
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
