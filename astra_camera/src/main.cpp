#include <rclcpp/rclcpp.hpp>

#include "astra_camera/ob_camera_node_factory.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<astra_camera::OBCameraNodeFactory>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
