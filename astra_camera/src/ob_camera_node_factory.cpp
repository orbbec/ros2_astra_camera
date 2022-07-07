#include <sys/shm.h>
#include <unistd.h>

#include "astra_camera/ob_camera_node_factory.h"

namespace astra_camera {
OBCameraNodeFactory::OBCameraNodeFactory(const rclcpp::NodeOptions& node_options)
    : Node("astra_camera_node", "/", node_options), logger_(get_logger()) {
  init();
}

OBCameraNodeFactory::OBCameraNodeFactory(const std::string& node_name, const std::string& ns,
                                         const rclcpp::NodeOptions& node_options)
    : Node(node_name, ns, node_options), logger_(get_logger()) {
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() { openni::OpenNI::shutdown(); }

void OBCameraNodeFactory::init() {
  auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR(logger_, "Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(-1);
  }
  device_connected_.store(false);
  parameters_ = std::make_shared<Parameters>(this);
  use_uvc_camera_ = declare_parameter<bool>("uvc_camera.enable", false);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  number_of_devices_ = static_cast<int>(declare_parameter<int>("number_of_devices", 1));
  setupUVCCameraConfig();
  auto connected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceConnected(device_info);
  };
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceDisconnected(device_info);
  };
  device_listener_ =
      std::make_unique<DeviceListener>(connected_cb, disconnected_cb, number_of_devices_);
  using namespace std::chrono_literals;
  check_connection_timer_ = this->create_wall_timer(1s, [this] { checkConnectionTimer(); });
}

void OBCameraNodeFactory::setupUVCCameraConfig() {
  uvc_config_.vendor_id = static_cast<int>(declare_parameter<int>("uvc_camera.vid", 0));
  uvc_config_.product_id = static_cast<int>(declare_parameter<int>("uvc_camera.pid", 0));
  uvc_config_.width = static_cast<int>(declare_parameter<int>("uvc_camera.weight", 640));
  uvc_config_.height = static_cast<int>(declare_parameter<int>("uvc_camera.height", 480));
  uvc_config_.fps = static_cast<int>(declare_parameter<int>("uvc_camera.fps", 30));
  uvc_config_.format = declare_parameter<std::string>("uvc_camera.format", "mjpeg");
}

void OBCameraNodeFactory::startDevice() {
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(parameters_);
  if (use_uvc_camera_) {
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    uvc_config_.serial_number = serial_number_;
    uvc_camera_driver_ = std::make_shared<UVCCameraDriver>(this, uvc_config_);
    ob_camera_node_ =
        std::make_unique<OBCameraNode>(this, device_, parameters_, uvc_camera_driver_);
  } else {
    ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  }
  device_connected_.store(true);
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "Device connected: " << device_info->getName());
}

void OBCameraNodeFactory::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  if (device_uri_ == device_info->getUri()) {
    device_uri_.clear();
    if (ob_camera_node_) {
      ob_camera_node_.reset();
    }
    if (device_) {
      device_->close();
      device_.reset();
    }
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    device_connected_.store(false);
  }
}

void OBCameraNodeFactory::checkConnectionTimer() {
  if (!device_connected_) {
    RCLCPP_INFO_STREAM(logger_, "wait for device connect...");
    RCLCPP_INFO_STREAM(logger_, "check device connection...");
    if (device_listener_->hasDevice(serial_number_)) {
      RCLCPP_INFO_STREAM(logger_, "device " << serial_number_ << " connected");
      auto device_info = device_listener_->getDeviceInfo(serial_number_);
      device_uri_ = device_info->getUri();
      device_ = std::make_unique<openni::Device>();
      if (device_->open(device_uri_.c_str()) != openni::STATUS_OK) {
        RCLCPP_ERROR_STREAM(logger_, "Could not open device: " << device_uri_);
        return;
      } else {
        startDevice();
      }
    }
  }
}

}  // namespace astra_camera
