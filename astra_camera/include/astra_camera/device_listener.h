#pragma once

#include <semaphore.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/shm.h>
#include <cerrno>
#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>

namespace astra_camera {

using DeviceConnectedCb = std::function<void(const openni::DeviceInfo*)>;

using DeviceDisconnectedCb = std::function<void(const openni::DeviceInfo*)>;

class DeviceListener : public openni::OpenNI::DeviceConnectedListener,
                       public openni::OpenNI::DeviceDisconnectedListener,
                       public openni::OpenNI::DeviceStateChangedListener {
 public:
  DeviceListener(DeviceConnectedCb connected_cb, DeviceDisconnectedCb disconnected_cb,
                 int num_of_devices = 1);

  ~DeviceListener() override;

  void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state) override;

  void onDeviceConnected(const openni::DeviceInfo* pInfo) override;

  void onDeviceDisconnected(const openni::DeviceInfo* pInfo) override;

  std::vector<std::string> getConnectedDeviceURIs();

  std::vector<openni::DeviceInfo> getConnectedDeviceInfo();

  bool hasDevice(const std::string& serial_number) const;

  // get device info by serial number
  std::optional<openni::DeviceInfo> getDeviceInfo(const std::string& serial_number) const;

 private:
  rclcpp::Logger logger_;
  DeviceConnectedCb connected_cb_;
  DeviceDisconnectedCb disconnected_cb_;
  size_t num_of_devices_ = 1;

  // serial number -> device info
  std::unordered_map<std::string, openni::DeviceInfo> connected_devices_;
  // uri -> serial number
  std::unordered_map<std::string, std::string> devices_serial_number_;
  sem_t* device_sem_;
};

}  // namespace astra_camera
