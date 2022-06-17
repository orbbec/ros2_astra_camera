
#include "astra_camera/ob_frame_listener.h"
#include <sensor_msgs/image_encodings.hpp>

namespace astra_camera {
OBFrameListener::OBFrameListener()
    : logger_(rclcpp::get_logger("frame_listener")),
      callback_(0),
      user_device_timer_(false),
      timer_filter_(new OBTimerFilter(TIME_FILTER_LENGTH)),
      prev_time_stamp_(0.0) {}

void OBFrameListener::setUseDeviceTimer(bool enable) {
  user_device_timer_ = enable;

  if (user_device_timer_) {
    timer_filter_->clear();
  }
}

void OBFrameListener::onNewFrame(openni::VideoStream& stream) {
  stream.readFrame(&frame_);

  if (frame_.isValid() && callback_) {
    sensor_msgs::msg::Image image;
    auto ros_now = rclcpp::Clock().now();
    if (!user_device_timer_) {
      image.header.stamp = ros_now;
      prev_time_stamp_ = ros_now.seconds();
    } else {
      uint64_t device_time = frame_.getTimestamp();

      double device_time_in_sec = static_cast<double>(device_time) / 1000000.0;
      double ros_time_in_sec = ros_now.seconds();

      double time_diff = ros_time_in_sec - device_time_in_sec;

      timer_filter_->addSample(time_diff);

      double filtered_time_diff = timer_filter_->getMedian();

      double corrected_timestamp = device_time_in_sec + filtered_time_diff;
      // FIXME:
      image.header.stamp.sec = corrected_timestamp;

      RCLCPP_DEBUG_STREAM(logger_, "Time interval between frames: "
                                       << ((corrected_timestamp - prev_time_stamp_) * 1000.0));

      prev_time_stamp_ = corrected_timestamp;
    }

    image.width = frame_.getWidth();
    image.height = frame_.getHeight();

    std::size_t data_size = frame_.getDataSize();

    image.data.resize(data_size);
    memcpy(&image.data[0], frame_.getData(), data_size);

    image.is_bigendian = 0;

    const openni::VideoMode& video_mode = frame_.getVideoMode();
    switch (video_mode.getPixelFormat()) {
      case openni::PIXEL_FORMAT_DEPTH_1_MM:
      case openni::PIXEL_FORMAT_DEPTH_100_UM:
      case openni::PIXEL_FORMAT_SHIFT_9_2:
      case openni::PIXEL_FORMAT_SHIFT_9_3:
        image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image.step = sizeof(unsigned char) * 2 * image.width;
        break;

      case openni::PIXEL_FORMAT_RGB888:
        image.encoding = sensor_msgs::image_encodings::RGB8;
        image.step = sizeof(unsigned char) * 3 * image.width;
        break;
      case openni::PIXEL_FORMAT_YUV422:
        image.encoding = sensor_msgs::image_encodings::YUV422;
        image.step = sizeof(unsigned char) * 4 * image.width;
        break;
      case openni::PIXEL_FORMAT_GRAY8:
        image.encoding = sensor_msgs::image_encodings::MONO8;
        image.step = sizeof(unsigned char) * 1 * image.width;
        break;
      case openni::PIXEL_FORMAT_GRAY16:
        image.encoding = sensor_msgs::image_encodings::MONO16;
        image.step = sizeof(unsigned char) * 2 * image.width;
        break;
      case openni::PIXEL_FORMAT_JPEG:
      default:
        RCLCPP_ERROR(logger_, "Invalid image encoding");
        break;
    }
    if (callback_) {
      callback_(image);
    }
  }
}
}  // namespace astra_camera
