#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>
#include <glog/logging.h>

#include "utils.h"
#include "ob_timer_filter.h"
#include "constants.h"

namespace astra_camera {
class OBFrameListener : public openni::VideoStream::NewFrameListener {
 public:
  OBFrameListener();

  ~OBFrameListener() override = default;

  void onNewFrame(openni::VideoStream& stream) override;

  void setUseDeviceTimer(bool enable);

  void setCallback(FrameCallbackFunction callback);

 private:
  openni::VideoFrameRef frame_;
  rclcpp::Logger logger_;
  FrameCallbackFunction callback_;
  std::mutex callback_lock_;
  std::atomic_bool have_callback_;
  bool user_device_timer_;
  std::shared_ptr<OBTimerFilter> timer_filter_;
};

}  // namespace astra_camera
