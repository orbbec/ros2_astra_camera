
#include <utility>

#include "astra_camera/uvc_camara_driver.h"
#include "astra_camera/utils.h"

namespace astra_camera {
std::ostream& operator<<(std::ostream& os, const UVCCameraConfig& config) {
  os << "vendor_id: " << std::hex << config.vendor_id << std::endl;
  os << "product_id: " << std::hex << config.product_id << std::endl;
  os << "width: " << config.width << std::endl;
  os << "height: " << config.height << std::endl;
  os << "fps: " << config.fps << std::endl;
  os << "index: " << config.index << std::endl;
  os << "serial_number: " << config.serial_number << std::endl;
  os << "format: " << config.format << std::endl;
  return os;
}
UVCCameraDriver::UVCCameraDriver(rclcpp::Node* node, UVCCameraConfig config)
    : node_(node), logger_(node_->get_logger()), config_(std::move(config)) {
  frame_id_ = getNoSlashNamespace() + "_color_optical_frame";
  auto err = uvc_init(&ctx_, nullptr);
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    RCLCPP_ERROR_STREAM(logger_, "init uvc context failed, exit");
    exit(err);
  }
  setupCameraControlService();
  image_publisher_ =
      image_transport::create_publisher(node_, "color/image_raw", rmw_qos_profile_sensor_data);
  camera_info_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "color/camera_info", rclcpp::QoS{1}.best_effort());
  openCamera();
}

UVCCameraDriver::~UVCCameraDriver() {
  uvc_close(device_handle_);
  device_handle_ = nullptr;
  uvc_unref_device(device_);
  device_ = nullptr;
  uvc_exit(ctx_);
  ctx_ = nullptr;
  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
  }
}

void UVCCameraDriver::openCamera() {
  RCLCPP_INFO_STREAM(logger_, "open uvc camera");
  uvc_error_t err;
  auto serial_number = config_.serial_number.empty() ? nullptr : config_.serial_number.c_str();
  CHECK(device_ == nullptr);
  err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, nullptr);
  RCLCPP_INFO_STREAM(logger_, "uvc config: " << config_);
  if (err != UVC_SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "Find device error " << uvc_strerror(err));
    uvc_unref_device(device_);
    exit(-1);
  }
  CHECK(device_handle_ == nullptr);
  err = uvc_open(device_, &device_handle_);
  if (err != UVC_SUCCESS) {
    if (UVC_ERROR_ACCESS == err) {
      RCLCPP_ERROR(logger_, "Permission denied opening /dev/bus/usb/%03d/%03d",
                   uvc_get_bus_number(device_), uvc_get_device_address(device_));
    } else {
      RCLCPP_ERROR(logger_, "Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                   uvc_get_bus_number(device_), uvc_get_device_address(device_), uvc_strerror(err),
                   err);
    }
    uvc_unref_device(device_);
    return;
  }
  uvc_set_status_callback(device_handle_, &UVCCameraDriver::autoControlsCallbackWrapper, this);
}

void UVCCameraDriver::setVideoMode() {
  auto uvc_format = UVCFrameFormatString(config_.format);
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;
  uvc_error_t err;
  for (int i = 0; i < 5; i++) {
    err = uvc_get_stream_ctrl_format_size(device_handle_, &ctrl_, uvc_format, width, height, fps);
    if (err == UVC_SUCCESS) {
      break;
    }
  }
  RCLCPP_INFO_STREAM(logger_, "set uvc mode" << width << "x" << height << "@" << fps << " format "
                                             << magic_enum::enum_name(uvc_format));
  if (err != UVC_SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "set uvc ctrl error " << uvc_strerror(err));
    uvc_close(device_handle_);
    device_handle_ = nullptr;
    uvc_unref_device(device_);
    device_ = nullptr;
    return;
  }
}

void UVCCameraDriver::startStreaming() {
  if (is_streaming_started) {
    return;
  }
  setVideoMode();
  uvc_error_t stream_err =
      uvc_start_streaming(device_handle_, &ctrl_, &UVCCameraDriver::frameCallbackWrapper, this, 0);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(device_handle_);
    uvc_unref_device(device_);
    return;
  }

  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
  }

  frame_buffer_ = uvc_allocate_frame(config_.width * config_.height * 3);
  CHECK_NOTNULL(frame_buffer_);
  is_streaming_started.store(true);
}

void UVCCameraDriver::stopStreaming() {
  if (!is_streaming_started) {
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "stop uvc streaming");
  uvc_stop_streaming(device_handle_);
  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
    frame_buffer_ = nullptr;
  }
  is_streaming_started.store(false);
}

void UVCCameraDriver::setupCameraControlService() {
  get_uvc_exposure_srv_ = node_->create_service<GetInt32>(
      "get_uvc_exposure", [this](const std::shared_ptr<GetInt32::Request> request,
                                 std::shared_ptr<GetInt32::Response> response) {
        return getUVCExposureCb(request, response);
      });
  set_uvc_exposure_srv_ = node_->create_service<SetInt32>(
      "set_uvc_exposure", [this](const std::shared_ptr<SetInt32::Request> request,
                                 std::shared_ptr<SetInt32::Response> response) {
        return setUVCExposureCb(request, response);
      });
  get_uvc_gain_srv_ = node_->create_service<GetInt32>(
      "get_uvc_gain", [this](const std::shared_ptr<GetInt32::Request> request,
                             std::shared_ptr<GetInt32::Response> response) {
        return getUVCGainCb(request, response);
      });
  set_uvc_gain_srv_ = node_->create_service<SetInt32>(
      "set_uvc_gain", [this](const std::shared_ptr<SetInt32::Request> request,
                             std::shared_ptr<SetInt32::Response> response) {
        return setUVCGainCb(request, response);
      });
  get_uvc_white_balance_srv_ = node_->create_service<GetInt32>(
      "get_uvc_white_balance", [this](const std::shared_ptr<GetInt32::Request> request,
                                      std::shared_ptr<GetInt32::Response> response) {
        return getUVCWhiteBalanceCb(request, response);
      });
  set_uvc_white_balance_srv_ = node_->create_service<SetInt32>(
      "set_uvc_white_balance", [this](const std::shared_ptr<SetInt32::Request> request,
                                      std::shared_ptr<SetInt32::Response> response) {
        return setUVCWhiteBalanceCb(request, response);
      });
  set_uvc_auto_exposure_srv_ = node_->create_service<SetBool>(
      "set_uvc_auto_exposure", [this](const std::shared_ptr<SetBool::Request> request,
                                      std::shared_ptr<SetBool::Response> response) {
        return setUVCAutoExposureCb(request, response);
      });
  set_uvc_auto_white_balance_srv_ = node_->create_service<SetBool>(
      "set_uvc_auto_white_balance", [this](const std::shared_ptr<SetBool::Request> request,
                                           std::shared_ptr<SetBool::Response> response) {
        return setUVCAutoWhiteBalanceCb(request, response);
      });

  get_uvc_mirror_srv_ = node_->create_service<GetInt32>(
      "get_uvc_mirror", [this](const std::shared_ptr<GetInt32::Request> request,
                               std::shared_ptr<GetInt32::Response> response) {
        return getUVCMirrorCb(request, response);
      });
  set_uvc_mirror_srv_ = node_->create_service<SetBool>(
      "set_uvc_mirror", [this](const std::shared_ptr<SetBool::Request> request,
                               std::shared_ptr<SetBool::Response> response) {
        return setUVCMirrorCb(request, response);
      });

  toggle_uvc_camera_srv_ = node_->create_service<SetBool>(
      "toggle_uvc_camera", [this](const std::shared_ptr<SetBool::Request> request,
                                  std::shared_ptr<SetBool::Response> response) {
        return toggleUVCCamera(request, response);
      });
  get_camera_info_cli_ = node_->create_client<GetCameraInfo>("get_camera_info");
}

sensor_msgs::msg::CameraInfo UVCCameraDriver::getCameraInfo() {
  using namespace std::chrono_literals;
  while (!get_camera_info_cli_->wait_for_service(1s)) {
    CHECK(rclcpp::ok()) << "Interrupted while waiting for the service. Exiting.";
  }
  using ServiceResponseFuture = rclcpp::Client<GetCameraInfo>::SharedFuture;
  auto request = std::make_shared<GetCameraInfo::Request>();
  auto future = get_camera_info_cli_->async_send_request(request);
  const auto& result = future.get();
  camera_info_ = result->info;
  camera_info_->header.frame_id = frame_id_;
  return result->info;
}

void UVCCameraDriver::updateConfig(const UVCCameraConfig& config) { config_ = config; }

enum uvc_frame_format UVCCameraDriver::UVCFrameFormatString(const std::string& format) {
  if (format == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (format == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (format == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (format == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (format == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (format == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (format == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (format == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    RCLCPP_WARN_STREAM(logger_, "Invalid Video Mode: " << format);
    RCLCPP_WARN_STREAM(logger_, "Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
}

void UVCCameraDriver::frameCallback(uvc_frame_t* frame) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(frame_buffer_);
  static constexpr int unit_step = 3;
  sensor_msgs::msg::Image image;
  image.width = frame->width;
  image.height = frame->height;
  image.step = image.width * unit_step;
  image.header.frame_id = frame_id_;
  image.header.stamp = node_->now();
  image.data.resize(image.height * image.step);
  if (frame->frame_format == UVC_FRAME_FORMAT_BGR) {
    image.encoding = "bgr8";
    memcpy(&(image.data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB) {
    image.encoding = "rgb8";
    memcpy(&(image.data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    image.encoding = "yuv422";
    memcpy(&(image.data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image.encoding = "bgr8";
    memcpy(&(image.data[0]), frame_buffer_->data, frame_buffer_->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Enable mjpeg support despite uvs_any2bgr shortcoming
    //  https://github.com/ros-drivers/libuvc_ros/commit/7508a09f
    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image.encoding = "rgb8";
    memcpy(&(image.data[0]), frame_buffer_->data, frame_buffer_->data_bytes);
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image.encoding = "bgr8";
    memcpy(&(image.data[0]), frame_buffer_->data, frame_buffer_->data_bytes);
  }
  if (!camera_info_.has_value()) {
    getCameraInfo();
  }
  if (camera_info_.has_value()) {
    camera_info_->header.stamp = node_->now();
    camera_info_publisher_->publish(*camera_info_);
  }
  image_publisher_.publish(image);
}

void UVCCameraDriver::frameCallbackWrapper(uvc_frame_t* frame, void* ptr) {
  CHECK_NOTNULL(ptr);
  auto driver = static_cast<UVCCameraDriver*>(ptr);
  driver->frameCallback(frame);
}

std::string UVCCameraDriver::getNoSlashNamespace() {
  auto ns = node_->get_effective_namespace();
  auto ns_vec = split(ns, '/');
  return ns_vec.empty() ? "camera" : ns_vec.front();
}

void UVCCameraDriver::autoControlsCallbackWrapper(enum uvc_status_class status_class, int event,
                                                  int selector,
                                                  enum uvc_status_attribute status_attribute,
                                                  void* data, size_t data_len, void* ptr) {
  CHECK_NOTNULL(ptr);
  auto driver = static_cast<UVCCameraDriver*>(ptr);
  driver->autoControlsCallback(status_class, event, selector, status_attribute, data, data_len);
}

void UVCCameraDriver::autoControlsCallback(enum uvc_status_class status_class, int event,
                                           int selector, enum uvc_status_attribute status_attribute,
                                           void* data, size_t data_len) {
  char buff[256];
  CHECK(data_len < 256);
  sprintf(buff, "Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %zu\n",
          status_class, event, selector, status_attribute, data_len);
  RCLCPP_INFO_STREAM(logger_, buff);
}

bool UVCCameraDriver::getUVCExposureCb(const std::shared_ptr<GetInt32::Request>& request,
                                       std::shared_ptr<GetInt32::Response>& response) {
  uint32_t data;
  uvc_error_t err = uvc_get_exposure_abs(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  response->data = static_cast<int>(data);
  return true;
}

bool UVCCameraDriver::setUVCExposureCb(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response) {
  uvc_set_ae_mode(device_handle_, 1);
  if (request->data > 330) {
    std::string msg = "Please set exposure lower than 330";
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  auto err = uvc_set_exposure_abs(device_handle_, request->data);
  if (err != UVC_SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << uvc_strerror(err));
    response->success = false;
    response->message = uvc_strerror(err);
    return false;
  }
  return true;
}

bool UVCCameraDriver::getUVCGainCb(const std::shared_ptr<GetInt32::Request>& request,
                                   std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  uint16_t data;
  auto err = uvc_get_gain(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  response->data = data;
  return true;
}

bool UVCCameraDriver::setUVCGainCb(const std::shared_ptr<SetInt32::Request>& request,
                                   std::shared_ptr<SetInt32::Response>& response) {
  auto err = uvc_set_gain(device_handle_, request->data);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::getUVCWhiteBalanceCb(const std::shared_ptr<GetInt32::Request>& request,
                                           std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  uint16_t data;
  auto err = uvc_get_white_balance_temperature(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  response->data = data;
  return true;
}

bool UVCCameraDriver::setUVCWhiteBalanceCb(const std::shared_ptr<SetInt32::Request>& request,
                                           std::shared_ptr<SetInt32::Response>& response) {
  uvc_set_white_balance_temperature_auto(device_handle_, 0);
  auto err = uvc_set_white_balance_temperature(device_handle_, request->data);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoExposureCb(const std::shared_ptr<SetBool::Request>& request,
                                           std::shared_ptr<SetBool::Response>& response) {
  uint8_t mode = request->data ? 8 : 1;  // 8 auto, 1manual
  auto err = uvc_set_ae_mode(device_handle_, mode);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoWhiteBalanceCb(const std::shared_ptr<SetBool::Request>& request,
                                               std::shared_ptr<SetBool::Response>& response) {
  auto err = uvc_set_white_balance_temperature_auto(device_handle_, request->data);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::getUVCMirrorCb(const std::shared_ptr<GetInt32::Request>& request,
                                     std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  int16_t roll;
  auto err = uvc_get_roll_abs(device_handle_, &roll, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCMirrorCb(const std::shared_ptr<SetBool::Request>& request,
                                     std::shared_ptr<SetBool::Response>& response) {
  int16_t roll;
  if (request->data) {
    roll = config_.product_id == DEEYEA_PID ? 3 : 1;
  } else {
    roll = 0;
  }
  CHECK_NOTNULL(device_handle_);
  auto err = uvc_set_roll_abs(device_handle_, roll);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::toggleUVCCamera(const std::shared_ptr<SetBool::Request>& request,
                                      std::shared_ptr<SetBool::Response>& response) {
  (void)response;
  if (request->data) {
    startStreaming();
  } else {
    stopStreaming();
  }
}
}  // namespace astra_camera
