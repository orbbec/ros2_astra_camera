#pragma once

#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>

namespace astra_camera {

using DeviceConnectedCb = std::function<void(const openni::DeviceInfo*)>;

using DeviceDisconnectedCb = std::function<void(const openni::DeviceInfo*)>;

class DeviceListener : public openni::OpenNI::DeviceConnectedListener,
                       public openni::OpenNI::DeviceDisconnectedListener,
                       public openni::OpenNI::DeviceStateChangedListener {
 public:
  DeviceListener(DeviceConnectedCb connected_cb, DeviceDisconnectedCb disconnected_cb);

  ~DeviceListener() override;

  void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state) override;

  void onDeviceConnected(const openni::DeviceInfo* pInfo) override;

  void onDeviceDisconnected(const openni::DeviceInfo* pInfo) override;

 private:
  rclcpp::Logger logger_;
  DeviceConnectedCb connected_cb_;
  DeviceDisconnectedCb disconnected_cb_;
};

}  // namespace astra_camera
