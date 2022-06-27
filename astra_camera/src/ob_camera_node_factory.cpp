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

OBCameraNodeFactory::~OBCameraNodeFactory() = default;

void OBCameraNodeFactory::init() {
  device_connected_.store(false);
  parameters_ = std::make_shared<Parameters>(this);
  use_uvc_camera_ = declare_parameter<bool>("use_uvc_camera", false);
  device_uri_ = declare_parameter<std::string>("device_uri", "");
  device_listener_ = std::make_unique<DeviceListener>();
  auto connected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceConnected(device_info);
  };
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceDisconnected(device_info);
  };
  device_listener_->setDeviceListenerCallback(connected_cb, disconnected_cb);
  using namespace std::chrono_literals;
  check_connection_timer_ = this->create_wall_timer(1s, [this] { checkConnectionTimer(); });
}

void OBCameraNodeFactory::setupUVCCameraConfig() {}

void OBCameraNodeFactory::startDevice() {
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(parameters_);
  if (use_uvc_camera_) {
    setupUVCCameraConfig();
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    uvc_camera_driver_ = std::make_shared<UVCCameraDriver>(this, uvc_config_);
    ob_camera_node_ =
        std::make_unique<OBCameraNode>(this, device_, parameters_, uvc_camera_driver_);
  } else {
    ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  }
  device_connected_.store(true);
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  if (!device_uri_.empty() || device_uri_ == device_info->getUri()) {
    device_info = device_info_;
    auto uri = device_info->getUri();
    if (device_) {
      device_.reset();
    }
    device_ = std::make_shared<openni::Device>();
    device_->open(uri);
    startDevice();
  }
}

void OBCameraNodeFactory::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  if (!device_uri_.empty() || device_uri_ == device_info->getUri()) {
    device_info = nullptr;
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
  }
}

}  // namespace astra_camera
