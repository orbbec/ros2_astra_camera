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
  boot_order_ = static_cast<int>(declare_parameter<int>("boot_order", 0));
  number_of_device_ = static_cast<int>(declare_parameter<int>("number_of_device", 1));
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
    if (number_of_device_ > 1) {
      char serial_number[64];
      int data_size = sizeof(serial_number);
      memset(serial_number, 0, data_size);
      device_->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, (uint8_t*)&serial_number,
                           &data_size);
      serial_number_ = serial_number;
      uvc_config_.serial_number = serial_number_;
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
  if (!device_connected_ && device_info->getUri() != nullptr) {
    int shm_id;
    char* shm = nullptr;
    shm_id = shmget((key_t)0401, 1, 0666 | IPC_CREAT);
    if (shm_id == -1) {
      RCLCPP_ERROR_STREAM(logger_, "Create Share Memory Error " << strerror(errno));
      exit(-1);
    }
    shm = (char*)shmat(shm_id, nullptr, 0);
    if (boot_order_ > 0) {
      while (*shm != boot_order_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    auto uri = device_info->getUri();
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
    device_connected_.store(true);
    *shm = static_cast<char>(boot_order_ + 1);
    if (shmdt(shm) == -1) {
      RCLCPP_ERROR_STREAM(logger_, "shm detach failed");
      exit(-1);
    }
    if (boot_order_ + 1 == number_of_device_) {
      if (shmctl(shm_id, IPC_RMID, nullptr) == -1) {
        RCLCPP_ERROR_STREAM(logger_, "remove shm identifier failed\n");
      }
    }
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
