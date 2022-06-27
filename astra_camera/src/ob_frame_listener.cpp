
#include "astra_camera/ob_frame_listener.h"
#include <sensor_msgs/image_encodings.hpp>

namespace astra_camera {
OBFrameListener::OBFrameListener()
    : logger_(rclcpp::get_logger("frame_listener")),
      have_callback_(false),
      user_device_timer_(false),
      timer_filter_(new OBTimerFilter(TIME_FILTER_LENGTH)),
      prev_time_stamp_(0.0) {}

void OBFrameListener::setUseDeviceTimer(bool enable) {
  user_device_timer_ = enable;

  if (user_device_timer_) {
    timer_filter_->clear();
  }
}
void OBFrameListener::setCallback(FrameCallbackFunction callback) {
  callback_ = std::move(callback);
  have_callback_ = true;
}

void OBFrameListener::onNewFrame(openni::VideoStream& stream) {
  stream.readFrame(&frame_);

  if (frame_.isValid() && have_callback_) {
    callback_(frame_);
  }
}
}  // namespace astra_camera
