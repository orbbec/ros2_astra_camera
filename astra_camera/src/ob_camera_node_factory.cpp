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
  device_uri_ = declare_parameter<std::string>("device_uri", "");
  setupUVCCameraConfig();
  auto connected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceConnected(device_info);
  };
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceDisconnected(device_info);
  };
  device_listener_ = std::make_unique<DeviceListener>(connected_cb, disconnected_cb);
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
    uvc_camera_driver_ = std::make_shared<UVCCameraDriver>(this, uvc_config_);
    ob_camera_node_ =
        std::make_unique<OBCameraNode>(this, device_, parameters_, uvc_camera_driver_);
  } else {
    ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  }
  device_connected_.store(true);
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "device connect..." << device_info->getUri());
  if (device_uri_.empty() || device_uri_ == device_info->getUri()) {
    auto uri = device_info->getUri();
    if (uri == nullptr) {
      RCLCPP_ERROR_STREAM(logger_, "device " << device_info->getUsbProductId() << "uri is empty");
      return;
    }
    if (device_) {
      device_->close();
      device_.reset();
    }
    device_ = std::make_shared<openni::Device>();
    device_uri_ = uri;
    auto status = device_->open(uri);
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_,
                          "open device " << uri << " error " << openni::OpenNI::getExtendedError());
      exit(-1);
    }
    CHECK(device_->hasSensor(openni::SENSOR_DEPTH));
    startDevice();
  }
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
  }
}

}  // namespace astra_camera
