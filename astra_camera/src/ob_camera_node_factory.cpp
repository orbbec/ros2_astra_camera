#include <sys/shm.h>
#include <unistd.h>
#include <fcntl.h>
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
  openni::OpenNI::setLogMinSeverity(0);
  openni::OpenNI::setLogFileOutput(true);
  auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR(logger_, "Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(-1);
  }
  device_connected_.store(false);
  parameters_ = std::make_shared<Parameters>(this);
  use_uvc_camera_ = declare_parameter<bool>("uvc_camera.enable", false);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  number_of_devices_ = declare_parameter<int>("number_of_devices", 1);
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
  if (device_info->getUri() == nullptr) {
    RCLCPP_ERROR_STREAM(logger_, "Device connected: " << device_info->getName() << " uri is null");
    return;
  }
  auto device_sem = sem_open("astra_device_sem", O_CREAT, 0644, 1);
  if (device_sem == (void*)SEM_FAILED) {
    RCLCPP_ERROR(logger_, "Failed to create semaphore");
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "Waiting for device to be ready");
  int ret = sem_wait(device_sem);
  if (!ret) {
    auto device = std::make_shared<openni::Device>();
    RCLCPP_INFO_STREAM(logger_, "Trying to open device: " << device_info->getUri());
    auto rc = device->open(device_info->getUri());
    if (rc != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to open device: " << device_info->getUri() << " error: "
                                                             << openni::OpenNI::getExtendedError());
      if (rc == openni::STATUS_DEVICE_IS_ALREADY_OPENED) {
        RCLCPP_ERROR_STREAM(logger_, "Device is already opened");
        connected_devices_[device_info->getUri()] = *device_info;
      }
    }
    char serial_number[64];
    int data_size = sizeof(serial_number);
    rc = device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
    if (rc != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get serial number: " << openni::OpenNI::getExtendedError());
    } else if (serial_number_.empty() || serial_number == serial_number_) {
      RCLCPP_INFO_STREAM(logger_, "Device connected: " << device_info->getName()
                                                       << " serial number: " << serial_number);
      device_uri_ = device_info->getUri();
      connected_devices_[device_uri_] = *device_info;
      device_ = device;
      startDevice();
    }
    if (!device_connected_) {
      device->close();
    }
  }
  sem_post(device_sem);
  if (connected_devices_.size() == number_of_devices_) {
    RCLCPP_INFO(logger_, "All devices connected");
    sem_unlink("astra_device_sem");
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
