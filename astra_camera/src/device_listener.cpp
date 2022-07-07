#include "astra_camera/device_listener.h"
#include <magic_enum.hpp>
#include <fcntl.h>

namespace astra_camera {

DeviceListener::DeviceListener(DeviceConnectedCb connected_cb, DeviceDisconnectedCb disconnected_cb,
                               int num_of_devices)
    : openni::OpenNI::DeviceConnectedListener(),
      openni::OpenNI::DeviceDisconnectedListener(),
      openni::OpenNI::DeviceStateChangedListener(),
      logger_(rclcpp::get_logger("device_listener")),
      connected_cb_(std::move(connected_cb)),
      disconnected_cb_(std::move(disconnected_cb)),
      num_of_devices_(num_of_devices) {
  openni::OpenNI::addDeviceConnectedListener(this);
  openni::OpenNI::addDeviceDisconnectedListener(this);
  openni::OpenNI::addDeviceStateChangedListener(this);
  // get list of currently connected devices
  openni::Array<openni::DeviceInfo> device_info_list;
  openni::OpenNI::enumerateDevices(&device_info_list);
  for (int i = 0; i < device_info_list.getSize(); ++i) {
    onDeviceConnected(&device_info_list[i]);
  }
}

DeviceListener::~DeviceListener() {
  openni::OpenNI::removeDeviceConnectedListener(this);
  openni::OpenNI::removeDeviceDisconnectedListener(this);
  openni::OpenNI::removeDeviceStateChangedListener(this);
}

void DeviceListener::onDeviceStateChanged(const openni::DeviceInfo* device_info,
                                          openni::DeviceState state) {
  RCLCPP_INFO_STREAM(logger_, "Device " << device_info->getUri() << " state changed to "
                                        << magic_enum::enum_name(state));
}

void DeviceListener::onDeviceConnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "onDeviceConnected");
  device_sem_ = sem_open("astra_device_sem", O_CREAT | O_EXCL, 0644, 1);
  if (device_sem_ == (void*)SEM_FAILED) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create semaphore: " << strerror(errno));
    exit(-1);
  }
  RCLCPP_INFO_STREAM(logger_, "Created semaphore " << std::hex << device_sem_);
  auto ret = sem_wait(device_sem_);
  std::cout << "after sem_wait" << std::endl;
  if (!ret) {
    std::cout << "Device " << device_info->getUri() << " connected" << std::endl;
    auto device = std::make_unique<openni::Device>();
    auto status = device->open(device_info->getUri());
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "open device error " << openni::OpenNI::getExtendedError());
    } else {
      char serial_number_str[64];
      int data_size = sizeof(serial_number_str);
      device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, (uint8_t*)&serial_number_str,
                          &data_size);
      std::string serial_number(serial_number_str);
      devices_serial_number_[device_info->getUri()] = serial_number;
      connected_devices_[serial_number] = *device_info;
      RCLCPP_INFO_STREAM(logger_, "open device with serial number: " << serial_number);
    }
    device->close();
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Failed to wait for device semaphore: " << strerror(errno));
  }
  if (connected_devices_.size() == num_of_devices_) {
    sem_unlink("astra_device_sem");
  }
  sem_post(device_sem_);
  connected_cb_(device_info);
}

void DeviceListener::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected");
  disconnected_cb_(device_info);
  auto serial_number = devices_serial_number_[device_info->getUri()];
  devices_serial_number_.erase(device_info->getUri());
  connected_devices_.erase(serial_number);
}

std::vector<std::string> DeviceListener::getConnectedDeviceURIs() {
  std::vector<std::string> uris;
  for (const auto& device : connected_devices_) {
    uris.emplace_back(device.first);
  }
  return uris;
}

std::vector<openni::DeviceInfo> DeviceListener::getConnectedDeviceInfo() {
  std::vector<openni::DeviceInfo> device_info_list;
  for (const auto& device : connected_devices_) {
    device_info_list.emplace_back(device.second);
  }
  return device_info_list;
}

bool DeviceListener::hasDevice(const std::string& serial_number) const {
  if (serial_number.empty()) {
    return devices_serial_number_.size() == 1;
  }
  return std::any_of(devices_serial_number_.begin(), devices_serial_number_.end(),
                     [&serial_number](const auto& pair) { return pair.second == serial_number; });
}

std::optional<openni::DeviceInfo> DeviceListener::getDeviceInfo(
    const std::string& serial_number) const {
  if (serial_number.empty()) {
    if (!connected_devices_.empty()) {
      return connected_devices_.begin()->second;
    } else {
      return std::nullopt;
    }
  }
  for (const auto& device : connected_devices_) {
    if (device.first == serial_number) {
      return device.second;
    }
  }
  return std::nullopt;
}
}  // namespace astra_camera
