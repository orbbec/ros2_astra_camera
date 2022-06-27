#include "astra_camera/device_listener.h"
#include <magic_enum.hpp>

namespace astra_camera {
DeviceListener::DeviceListener()
    : logger_(rclcpp::get_logger("device_listener")) {
  openni::OpenNI::addDeviceConnectedListener(this);
  openni::OpenNI::addDeviceDisconnectedListener(this);
  openni::OpenNI::addDeviceStateChangedListener(this);
}

DeviceListener::~DeviceListener() {
  openni::OpenNI::removeDeviceConnectedListener(this);
  openni::OpenNI::removeDeviceDisconnectedListener(this);
  openni::OpenNI::removeDeviceStateChangedListener(this);
}

void DeviceListener::onDeviceStateChanged(const openni::DeviceInfo* pInfo,
                                          openni::DeviceState state) {
  RCLCPP_INFO_STREAM(logger_, "Device " << pInfo->getUri() << " state changed to "
                                        << magic_enum::enum_name(state));
}

void DeviceListener::onDeviceConnected(const openni::DeviceInfo* pInfo) { connected_cb_(pInfo); }

void DeviceListener::onDeviceDisconnected(const openni::DeviceInfo* pInfo) {
  disconnected_cb_(pInfo);
}

void DeviceListener::setDeviceListenerCallback(const DeviceConnectedCb& connected_cb,
                                               const DeviceDisconnectedCb& disconnected_cb) {
  connected_cb_ = connected_cb;
  disconnected_cb_ = disconnected_cb;
}

}  // namespace astra_camera
