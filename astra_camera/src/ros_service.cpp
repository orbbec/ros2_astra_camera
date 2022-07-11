
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <thread>

#include "astra_camera/utils.h"
#include "astra_camera/ob_camera_node.h"

namespace astra_camera {

void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::srv::SetBool;
  for (auto stream_index : IMAGE_STREAMS) {
    auto stream_name = stream_name_[stream_index];
    std::string service_name = "get_" + stream_name + "_exposure";
    get_exposure_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          return getExposureCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_exposure";
    set_exposure_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          return setExposureCallback(request, response, stream_index);
        });
    service_name = "get_" + stream_name + "_gain";
    get_gain_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          return getGainCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_gain";
    set_gain_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          return setGainCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_auto_exposure";
    set_auto_exposure_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          return setAutoExposureCallback(request, response, stream_index);
        });

    service_name = "toggle_" + stream_name;

    toggle_sensor_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          return toggleSensorCallback(request, response, stream_index);
        });
    service_name = "get_" + stream_name + "_white_balance";
    get_white_balance_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          return getAutoWhiteBalanceEnabledCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_white_balance";

    set_white_balance_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          return setAutoWhiteBalanceEnabledCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_mirror";
    set_mirror_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          return setMirrorCallback(request, response, stream_index);
        });
  }
  set_fan_mode_srv_ = node_->create_service<SetInt32>(
      "set_fan_mode", [this](const std::shared_ptr<SetInt32::Request> request,
                             std::shared_ptr<SetInt32::Response> response) {
        return setFanModeCallback(request, response);
      });

  set_laser_enable_srv_ = node_->create_service<SetInt32>(
      "set_laser_enable", [this](const std::shared_ptr<SetInt32::Request> request,
                                 std::shared_ptr<SetInt32::Response> response) {
        return setLaserEnableCallback(request, response);
      });
  set_ldp_enable_srv_ = node_->create_service<SetInt32>(
      "set_ldp_enable", [this](const std::shared_ptr<SetInt32::Request> request,
                               std::shared_ptr<SetInt32::Response> response) {
        setLdpEnableCallback(request, response);
      });

  get_device_srv_ = node_->create_service<GetDeviceInfo>(
      "get_device_info", [this](const std::shared_ptr<GetDeviceInfo::Request> request,
                                std::shared_ptr<GetDeviceInfo::Response> response) {
        return getDeviceInfoCallback(request, response);
      });
  get_sdk_version_srv_ = node_->create_service<GetString>(
      "get_sdk_version", [](const std::shared_ptr<GetString::Request> request,
                            std::shared_ptr<GetString::Response> response) {
        return getSDKVersion(request, response);
      });
  get_camera_info_srv_ = node_->create_service<GetCameraInfo>(
      "get_camera_info", [this](const std::shared_ptr<GetCameraInfo::Request> request,
                                std::shared_ptr<GetCameraInfo::Response> response) {
        return getCameraInfoCallback(request, response);
      });
}

bool OBCameraNode::setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response,
                                       const stream_index_pair& stream_index) {
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  auto rc = camera_settings->setExposure(request->data);
  std::stringstream ss;
  if (rc != openni::STATUS_OK) {
    ss << "Couldn't set color exposure: " << openni::OpenNI::getExtendedError();
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, response->message);
    return false;
  } else {
    return true;
  }
}

bool OBCameraNode::getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                                   std::shared_ptr<GetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  (void)request;
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  response->data = camera_settings->getGain();
  return true;
}

bool OBCameraNode::setGainCallback(const std::shared_ptr<SetInt32 ::Request>& request,
                                   std::shared_ptr<SetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  camera_settings->setGain(request->data);
  return true;
}

bool OBCameraNode::getAutoWhiteBalanceEnabledCallback(
    const std::shared_ptr<GetInt32::Request>& request,
    std::shared_ptr<GetInt32::Response>& response, const stream_index_pair& stream_index) {
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  response->data = camera_settings->getAutoWhiteBalanceEnabled();
  return true;
}

bool OBCameraNode::setAutoWhiteBalanceEnabledCallback(
    const std::shared_ptr<SetInt32 ::Request>& request,
    std::shared_ptr<SetInt32 ::Response>& response, const stream_index_pair& stream_index) {
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  auto data = request->data;
  auto rc = camera_settings->setAutoWhiteBalanceEnabled(data);
  if (rc != openni::STATUS_OK) {
    std::stringstream ss;
    ss << " Couldn't set auto white balance: " << openni::OpenNI::getExtendedError();
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    return false;
  } else {
    return true;
  }
}

bool OBCameraNode::setAutoExposureCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
    const stream_index_pair& stream_index) {
  openni::Status status;
  if (stream_index == COLOR || stream_index == DEPTH) {
    auto stream = streams_.at(stream_index);
    auto camera_settings = stream->getCameraSettings();
    status = camera_settings->setAutoExposureEnabled(request->data);
  } else if (stream_index == INFRA1) {
    status = device_->setProperty(XN_MODULE_PROPERTY_AE, request->data);
  }

  if (status != openni::STATUS_OK) {
    std::stringstream ss;
    ss << "Couldn't set auto exposure: " << openni::OpenNI::getExtendedError();
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    return false;
  } else {
    return true;
  }
}

bool OBCameraNode::setFanModeCallback(const std::shared_ptr<SetInt32::Request>& request,
                                      std::shared_ptr<SetInt32::Response>& response) {
  device_->setProperty(XN_MODULE_PROPERTY_FAN_ENABLE, request->data);
  response->success = true;
}

bool OBCameraNode::setLaserEnableCallback(const std::shared_ptr<SetInt32::Request>& request,
                                          std::shared_ptr<SetInt32::Response>& response) {
  device_->setProperty(openni::OBEXTENSION_ID_LASER_EN, request->data);
  device_->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, request->data);
  response->success = true;
}

bool OBCameraNode::setLdpEnableCallback(const std::shared_ptr<SetInt32::Request>& request,
                                        std::shared_ptr<SetInt32::Response>& response) {
  (void)response;
  stopStreams();
  device_->setProperty(openni::OBEXTENSION_ID_LDP_EN, request->data);
  startStreams();
}

bool OBCameraNode::setMirrorCallback(const std::shared_ptr<SetBool::Request>& request,
                                     std::shared_ptr<SetBool::Response>& response,
                                     const stream_index_pair& stream_index) {
  (void)response;
  auto stream = streams_.at(stream_index);
  stream->setMirroringEnabled(request->data);
  return true;
}

bool OBCameraNode::getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                                       std::shared_ptr<GetInt32 ::Response>& response,
                                       const stream_index_pair& stream_index) {
  (void)request;
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  response->data = camera_settings->getExposure();
  return true;
}

bool OBCameraNode::getDeviceInfoCallback(const std::shared_ptr<GetDeviceInfo::Request>& request,
                                         std::shared_ptr<GetDeviceInfo::Response>& response) {
  (void)request;
  auto device_info = device_->getDeviceInfo();
  return true;
}

bool OBCameraNode::getCameraInfoCallback(const std::shared_ptr<GetCameraInfo::Request>& request,
                                         std::shared_ptr<GetCameraInfo::Response>& response) {
  (void)request;
  auto camera_info = getColorCameraInfo();
  response->info = *camera_info;
  return true;
}

bool OBCameraNode::getSDKVersion(const std::shared_ptr<GetString::Request>& request,
                                 std::shared_ptr<GetString::Response>& response) {
  (void)request;
  nlohmann::json data;
  data["ros_sdk_version"] = OB_ROS_VERSION_STR;
  data["openni_version"] = ONI_VERSION_STRING;
  response->data = data.dump(2);
  return true;
}

bool OBCameraNode::toggleSensorCallback(const std::shared_ptr<SetBool::Request>& request,
                                        std::shared_ptr<SetBool::Response>& response,
                                        const stream_index_pair& stream_index) {
  if (request->data) {
    RCLCPP_INFO_STREAM(logger_, streams_[stream_index] << " ON");
  } else {
    RCLCPP_INFO_STREAM(logger_, streams_[stream_index] << " OFF");
  }
  toggleSensor(stream_index, request->data, response->message);
  return true;
}

bool OBCameraNode::toggleSensor(const stream_index_pair& stream_index, bool enabled,
                                std::string& msg) {
  if (!device_->hasSensor(stream_index.first)) {
    std::stringstream ss;

    ss << "doesn't  have " << stream_name_[stream_index];
    msg = ss.str();
    RCLCPP_WARN_STREAM(logger_, msg);
    return false;
  }

  if (enabled) {
    enable_[stream_index] = true;
  } else {
    encoding_[stream_index];
  }
  return true;
}
}  // namespace astra_camera
