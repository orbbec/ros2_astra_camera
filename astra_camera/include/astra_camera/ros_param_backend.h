/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2023 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

namespace astra_camera {
#if RCLCPP_VERSION_GTE(20, 0, 0) // Iron changed the name, just moving 'Set' forwards
  using OnSetCallback = rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
#else
  using OnSetCallback = rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;
#endif

class ParametersBackend {
 public:
  explicit ParametersBackend(rclcpp::Node* node);
  ~ParametersBackend();
  void addOnSetParametersCallback(
       OnSetCallback callback);

 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::shared_ptr<void> ros_callback_;
};
}  // namespace astra_camera
