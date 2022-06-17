#pragma once
#include <functional>

#include <glog/logging.h>

#include <sensor_msgs/msg/image.hpp>

#include "constants.h"
#include "types.h"

namespace astra_camera {

bool operator==(const openni::VideoMode& lhs, const openni::VideoMode& rhs);

}  // namespace astra_camera
