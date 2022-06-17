#pragma once
#include <cstdlib>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace astra_camera {
using FrameCallbackFunction = std::function<void(const sensor_msgs::msg::Image& image)>;

typedef enum {
  RGBResolution4_3 = 0,   // 4:3分辨率 如：640x480
  RGBResolution16_9 = 1,  // 16:9分辨率 如：1920x1280
} RgbResolution;

typedef enum {
  OB_STREAM_VIDEO = 0,
  OB_STREAM_IR = 1,
  OB_STREAM_COLOR = 2,
  OB_STREAM_DEPTH = 3,
  OB_STREAM_ACCEL = 4,
  OB_STREAM_GYRO = 5,
} OBStreamType;

}  // namespace astra_camera
