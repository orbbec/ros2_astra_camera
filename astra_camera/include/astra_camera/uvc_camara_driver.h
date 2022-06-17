#pragma once
#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>
#include <libuvc/libuvc.h>

namespace astra_camera {
struct UVCCameraConfig {
  int vendor_id;
  int product_id;
  std::string serial_number;
  std::string format;
  uint32_t width;
  uint32_t height;
  uint32_t fps;
  UVCCameraConfig(const UVCCameraConfig&) = default;
  UVCCameraConfig(UVCCameraConfig&&) = default;
};

class UVCCameraDriver {
 public:
  explicit UVCCameraDriver(rclcpp::Node* node, const UVCCameraConfig& config);
  ~UVCCameraDriver();

  void openCamera();

  void closeCamera();

  void updateConfig(const UVCCameraConfig& config);

 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  UVCCameraConfig config_;
  uvc_context_t* ctx_;
  uvc_device_t* device_;
  uvc_device_handle_t* device_handle_;
  uvc_frame_t* frame_buffer_;
};

}  // namespace astra_camera
