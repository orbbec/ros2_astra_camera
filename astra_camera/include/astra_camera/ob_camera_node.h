#pragma once

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

#include <openni2/OpenNI.h>
#include <openni2/PS1080.h>

#include "astra_camera_msgs/msg/device_info.hpp"
#include "astra_camera_msgs/srv/get_device_info.hpp"
#include "astra_camera_msgs/msg/extrinsics.hpp"
#include "astra_camera_msgs/msg/metadata.hpp"
#include "astra_camera_msgs/srv/get_int32.hpp"
#include "astra_camera_msgs/srv/get_string.hpp"
#include "astra_camera_msgs/srv/set_int32.hpp"

#include "constants.h"
#include "utils.h"
#include "uvc_camara_driver.h"
#include "dynamic_params.h"
#include "types.h"

namespace astra_camera {
using GetDeviceInfo = astra_camera_msgs::srv::GetDeviceInfo;
using Extrinsics = astra_camera_msgs::msg::Extrinsics;
using SetInt32 = astra_camera_msgs::srv::SetInt32;
using GetInt32 = astra_camera_msgs::srv::GetInt32;
using GetString = astra_camera_msgs::srv::GetString;
using SetBool = std_srvs::srv::SetBool;

typedef std::pair<OBStreamType, int> stream_index_pair;

const stream_index_pair COLOR{OB_STREAM_COLOR, 0};
const stream_index_pair DEPTH{OB_STREAM_DEPTH, 0};
const stream_index_pair INFRA0{OB_STREAM_IR, 0};

const stream_index_pair GYRO{OB_STREAM_GYRO, 0};
const stream_index_pair ACCEL{OB_STREAM_ACCEL, 0};

const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, COLOR};

const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL};

class OBCameraNode {
 public:
  OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
               std::shared_ptr<Parameters> parameters);

  OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
               std::shared_ptr<Parameters> parameters,
               std::shared_ptr<UVCCameraDriver> uvc_camera_driver);

  ~OBCameraNode();

  void clean();

 private:
  void setupCameraCtrlServices();

  void setupDevices();

  void getParameters();

  void setupTopics();

  void setupPublishers();

  void publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                       const tf2::Quaternion& q, const std::string& from, const std::string& to);

  void calcAndPublishStaticTransform();

  void publishDynamicTransforms();

  void publishStaticTransforms();

  void getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                           std::shared_ptr<GetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  void setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                           std::shared_ptr<SetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  void getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                       std::shared_ptr<GetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  void setGainCallback(const std::shared_ptr<SetInt32::Request>& request,
                       std::shared_ptr<SetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  void getWhiteBalanceCallback(const std::shared_ptr<GetInt32::Request>& request,
                               std::shared_ptr<GetInt32::Response>& response);

  void setWhiteBalanceCallback(const std::shared_ptr<SetInt32 ::Request>& request,
                               std::shared_ptr<SetInt32 ::Response>& response);

  void setAutoExposureCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
                               const stream_index_pair& stream_index);

  void setLaserEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setFloorEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setLdpEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setFanModeCallback(const std::shared_ptr<SetInt32::Request>& request,
                          std::shared_ptr<SetInt32::Response>& response);

  void getDeviceInfoCallback(const std::shared_ptr<GetDeviceInfo::Request>& request,
                             std::shared_ptr<GetDeviceInfo::Response>& response);

  void getSDKVersion(const std::shared_ptr<GetString::Request>& request,
                     std::shared_ptr<GetString::Response>& response);

  void toggleSensorCallback(const std::shared_ptr<SetBool::Request>& request,
                            std::shared_ptr<SetBool::Response>& response,
                            const stream_index_pair& stream_index);

  bool toggleSensor(const stream_index_pair& stream_index, bool enabled, std::string& msg);

 private:
  rclcpp::Node* node_;
  std::shared_ptr<openni::Device> device_;
  std::shared_ptr<Parameters> parameters_;
  std::shared_ptr<UVCCameraDriver> uvc_camera_driver_ = nullptr;
  rclcpp::Logger logger_;
  bool use_uvc_camera_ = false;
  openni::DeviceInfo device_info_;
  std::map<stream_index_pair, bool> enable_;
  std::map<stream_index_pair, std::string> stream_name_;
  std::map<stream_index_pair, std::shared_ptr<openni::VideoStream>> streams_;
  std::map<stream_index_pair, image_transport::Publisher> image_publishers_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      camera_info_publishers_;

  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetBool>::SharedPtr> toggle_sensor_srv_;
  rclcpp::Service<GetInt32>::SharedPtr get_white_balance_srv_;  // only rgb
  rclcpp::Service<SetInt32>::SharedPtr set_white_balance_srv_;
  rclcpp::Service<GetString>::SharedPtr get_sdk_version_srv_;
  std::map<stream_index_pair, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr>
      set_auto_exposure_srv_;  // only rgb color
  rclcpp::Service<GetDeviceInfo>::SharedPtr get_device_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_laser_enable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ldp_enable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_floor_enable_srv_;
  rclcpp::Service<SetInt32>::SharedPtr set_fan_mode_srv_;

  bool publish_tf_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
  std::shared_ptr<std::thread> tf_thread_;
  std::condition_variable tf_cv_;
  double tf_publish_rate_ = 10.0;
};

}  // namespace astra_camera
