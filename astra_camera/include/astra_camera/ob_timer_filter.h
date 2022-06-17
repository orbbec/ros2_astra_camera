#pragma once
#include <deque>

#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>
#include <glog/logging.h>

class OBTimerFilter {
 public:
  explicit OBTimerFilter(std::size_t len);
  ~OBTimerFilter();

  void addSample(double sample);

  double getMedian();

  double getMovingAvg();

  void clear();

 private:
  std::size_t len_;
  rclcpp::Logger logger_;

  std::deque<double> buffer_;
};
