#pragma once
#include <atomic>
#include <thread>

#include <magic_enum.hpp>

#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>
#include "dynamic_params.h"
#include "ob_camera_node.h"
#include "uvc_camara_driver.h"
#include "device_listener.h"

namespace astra_camera {
class OBCameraNodeFactory : public rclcpp::Node {
 public:
  explicit OBCameraNodeFactory(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  OBCameraNodeFactory(const std::string& node_name, const std::string& ns,
                      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  ~OBCameraNodeFactory() override;

 private:
  void init();

  void startDevice();

  void onDeviceConnected(const openni::DeviceInfo* device_info);

  void onDeviceDisconnected(const openni::DeviceInfo* device_info);

  void checkConnectionTimer();

  void setupUVCCameraConfig();

 private:
  rclcpp::Logger logger_;
  std::unique_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::shared_ptr<openni::Device> device_ = nullptr;
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::shared_ptr<UVCCameraDriver> uvc_camera_driver_ = nullptr;
  std::shared_ptr<openni::DeviceInfo> device_info_ = nullptr;
  bool use_uvc_camera_;
  UVCCameraConfig uvc_config_;
  std::unique_ptr<DeviceListener> device_listener_ = nullptr;
  std::string serial_number_;
  std::string device_type_;
  std::string device_uri_;
  rclcpp::TimerBase::SharedPtr check_connection_timer_;
  std::atomic_bool device_connected_{false};
  int number_of_devices_;
};

}  // namespace astra_camera

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::OBCameraNodeFactory)
