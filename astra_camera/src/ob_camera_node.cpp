/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#include "astra_camera/ob_camera_node.h"
#include "astra_camera/utils.h"
#include <cv_bridge/cv_bridge.h>
namespace astra_camera {

void OBCameraNode::init() {
  is_running_.store(true);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  setupConfig();
  setupTopics();
  startStreams();
}

OBCameraNode::OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
                           std::shared_ptr<Parameters> parameters)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      logger_(node->get_logger()),
      use_uvc_camera_(false) {
  init();
}

OBCameraNode::OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
                           std::shared_ptr<Parameters> parameters,
                           std::shared_ptr<UVCCameraDriver> uvc_camera_driver)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      uvc_camera_driver_(std::move(uvc_camera_driver)),
      logger_(node->get_logger()),
      use_uvc_camera_(true) {
  init();
}

OBCameraNode::~OBCameraNode() { clean(); }

void OBCameraNode::clean() {
  is_running_.store(false);
  if (tf_thread_->joinable()) {
    tf_thread_->join();
  }
  stopStreams();
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (streams_[stream_index]) {
      streams_[stream_index]->destroy();
      streams_[stream_index].reset();
    }
  }
}

void OBCameraNode::stopStreams() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (stream_started_[stream_index]) {
      streams_[stream_index]->stop();
      auto listener = stream_frame_listener_[stream_index];
      streams_[stream_index]->removeNewFrameListener(listener.get());
      RCLCPP_INFO_STREAM(logger_, "Stopped stream " << stream_name_[stream_index]);
      stream_started_[stream_index] = false;
    }
  }

  if (use_uvc_camera_) {
    CHECK_NOTNULL(uvc_camera_driver_);
    uvc_camera_driver_->stopStreaming();
  }
}

template <class T>
void OBCameraNode::setAndGetNodeParameter(
    T& param, const std::string& param_name, const T& default_value,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor) {
  try {
    param = parameters_
                ->setParam(param_name, rclcpp::ParameterValue(default_value),
                           std::function<void(const rclcpp::Parameter&)>(), parameter_descriptor)
                .get<T>();
  } catch (const rclcpp::ParameterTypeException& ex) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to set parameter: " << param_name << ". " << ex.what());
    throw;
  }
}

void OBCameraNode::setupDevices() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    stream_started_[stream_index] = false;
    if (enable_[stream_index] && device_->hasSensor(stream_index.first)) {
      auto stream = std::make_shared<openni::VideoStream>();
      auto status = stream->create(*device_, stream_index.first);
      if (status != openni::STATUS_OK) {
        RCLCPP_ERROR_STREAM(
            logger_, "Couldn't create depth video stream: " << openni::OpenNI::getExtendedError());
      }
      CHECK_EQ(status, openni::STATUS_OK);
      streams_[stream_index] = stream;
      auto frame_listener = std::make_shared<OBFrameListener>();
      frame_listener->setCallback(stream_frame_callback_[stream_index]);
      stream_frame_listener_[stream_index] = frame_listener;
    } else {
      if (streams_[stream_index]) {
        streams_[stream_index].reset();
      }
      enable_[stream_index] = false;
      if (stream_frame_listener_[stream_index]) {
        stream_frame_listener_[stream_index].reset();
      }
    }
  }
  device_info_ = device_->getDeviceInfo();
}

void OBCameraNode::setupFrameCallback() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    auto frame_callback = [this, stream_index = stream_index](const openni::VideoFrameRef& frame) {
      onNewFrameCallback(frame, stream_index);
    };
    stream_frame_callback_[stream_index] = frame_callback;
  }
}

void OBCameraNode::setupVideoMode() {
  if (enable_[INFRA1] && enable_[COLOR]) {
    RCLCPP_WARN_STREAM(logger_,
                       "Infrared and Color streams are enabled. "
                       "Infrared stream will be disabled.");
    enable_[INFRA1] = false;
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    supported_video_modes_[stream_index] = std::vector<openni::VideoMode>();
    if (device_->hasSensor(stream_index.first) && enable_[stream_index]) {
      auto stream = streams_[stream_index];
      const auto& sensor_info = stream->getSensorInfo();
      const auto& supported_video_modes = sensor_info.getSupportedVideoModes();
      int size = supported_video_modes.getSize();
      for (int i = 0; i < size; i++) {
        supported_video_modes_[stream_index].emplace_back(supported_video_modes[i]);
      }
      openni::VideoMode video_mode;
      video_mode.setResolution(width_[stream_index], height_[stream_index]);
      video_mode.setFps(fps_[stream_index]);
      video_mode.setPixelFormat(format_[stream_index]);
      bool is_supported_mode = false;
      for (const auto& item : supported_video_modes_[stream_index]) {
        if (video_mode == item) {
          is_supported_mode = true;
          stream_video_mode_[stream_index] = video_mode;
          break;
        }
      }
      if (!is_supported_mode) {
        enable_[stream_index] = false;
        RCLCPP_WARN_STREAM(logger_, "Video mode " << video_mode
                                                  << "is not supported. "
                                                     "Stream will be disabled.");
        RCLCPP_INFO_STREAM(logger_, "Supported video modes: ");
        for (const auto& item : supported_video_modes_[stream_index]) {
          RCLCPP_INFO_STREAM(logger_, item);
        }
      }
      if (is_supported_mode) {
        RCLCPP_INFO_STREAM(logger_,
                           "set " << stream_name_[stream_index] << " video mode " << video_mode);
        images_[stream_index] = cv::Mat(height_[stream_index], width_[stream_index],
                                        image_format_[stream_index], cv::Scalar(0, 0, 0));
      }
    }
  }
}

void OBCameraNode::startStreams() {
  setupVideoMode();
  int color_width = 0;
  int color_height = 0;
  if (use_uvc_camera_) {
    CHECK_NOTNULL(uvc_camera_driver_);
    color_width = uvc_camera_driver_->getResolutionX();
    color_height = uvc_camera_driver_->getResolutionY();
  } else {
    color_width = stream_video_mode_[COLOR].getResolutionX();
    color_height = stream_video_mode_[COLOR].getResolutionY();
  }
  setImageRegistrationMode(depth_registration_);
  setDepthColorSync(color_depth_synchronization_);
  if (depth_registration_) {
    setDepthToColorResolution(color_width, color_height);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_[stream_index] && !stream_started_[stream_index]) {
      CHECK(stream_video_mode_.count(stream_index));
      auto video_mode = stream_video_mode_.at(stream_index);
      CHECK(streams_.count(stream_index));
      streams_[stream_index]->setVideoMode(video_mode);
      streams_[stream_index]->setMirroringEnabled(false);
      CHECK(stream_frame_listener_.count(stream_index));
      CHECK_NOTNULL(stream_frame_listener_[stream_index]);
      streams_[stream_index]->addNewFrameListener(stream_frame_listener_[stream_index].get());
      CHECK_EQ(streams_[stream_index]->start(), openni::STATUS_OK);
      stream_started_[stream_index] = true;
      RCLCPP_INFO_STREAM(logger_, magic_enum::enum_name(stream_index.first) << " is started");
    }
  }
  if (use_uvc_camera_) {
    uvc_camera_driver_->startStreaming();
  }
}

void OBCameraNode::setupConfig() {
  stream_name_[DEPTH] = "depth";
  unit_step_size_[DEPTH] = sizeof(uint16_t);
  format_[DEPTH] = openni::PIXEL_FORMAT_DEPTH_1_MM;
  image_format_[DEPTH] = CV_16UC1;
  encoding_[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

  stream_name_[COLOR] = "color";
  unit_step_size_[COLOR] = 3;
  format_[COLOR] = openni::PIXEL_FORMAT_RGB888;
  image_format_[COLOR] = CV_8UC3;
  encoding_[COLOR] = sensor_msgs::image_encodings::RGB8;

  stream_name_[INFRA1] = "ir";
  unit_step_size_[INFRA1] = sizeof(uint8_t);
  format_[INFRA1] = openni::PIXEL_FORMAT_GRAY16;
  image_format_[INFRA1] = CV_16UC1;
  encoding_[INFRA1] = sensor_msgs::image_encodings::MONO16;
  for (const auto& stream_index : IMAGE_STREAMS) {
    stream_started_[stream_index] = false;
  }
}

void OBCameraNode::getParameters() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_width";
    setAndGetNodeParameter(width_[stream_index], param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index] + "_height";
    setAndGetNodeParameter(height_[stream_index], param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index] + "_fps";
    setAndGetNodeParameter(fps_[stream_index], param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index];
    setAndGetNodeParameter(enable_[stream_index], param_name, false);
    param_name = stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = "camera_" + stream_name_[stream_index] + "_frame";
    setAndGetNodeParameter(frame_id_[stream_index], param_name, default_frame_id);
    std::string default_optical_frame_id =
        "camera_optical_" + stream_name_[stream_index] + "_frame";
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    setAndGetNodeParameter(optical_frame_id_[stream_index], param_name, default_optical_frame_id);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    depth_aligned_frame_id_[stream_index] = optical_frame_id_[COLOR];
  }
  setAndGetNodeParameter(publish_tf_, "publish_tf", true);
  setAndGetNodeParameter(tf_publish_rate_, "tf_publish_rate", 10.0);
  setAndGetNodeParameter(camera_link_frame_id_, "camera_link_frame_id", DEFAULT_BASE_FRAME_ID);
  setAndGetNodeParameter(depth_registration_, "depth_registration", false);
  setAndGetNodeParameter(color_depth_synchronization_, "color_depth_synchronization", false);
}

void OBCameraNode::setupTopics() {
  getParameters();
  setupFrameCallback();
  setupDevices();
  setupCameraCtrlServices();
  setupPublishers();
  setupVideoMode();
  getCameraParams();
  publishStaticTransforms();
}

void OBCameraNode::setupPublishers() {
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string name = stream_name_[stream_index];
    std::string topic = name + "/image_raw";
    image_publishers_[stream_index] =
        image_transport::create_publisher(node_, topic, rmw_qos_profile_sensor_data);
    topic = name + "/camera_info";
    camera_info_publishers_[stream_index] =
        node_->create_publisher<CameraInfo>(topic, rclcpp::QoS{1}.best_effort());
  }
  extrinsics_publisher_ = node_->create_publisher<Extrinsics>("extrinsic/depth_to_color",
                                                              rclcpp::QoS{1}.transient_local());
}

void OBCameraNode::publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                                   const tf2::Quaternion& q, const std::string& from,
                                   const std::string& to) {
  CHECK_EQ(trans.size(), 3u);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans.at(2) / 1000.0;
  msg.transform.translation.y = -trans.at(0) / 1000.0;
  msg.transform.translation.z = -trans.at(1) / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_msgs_.push_back(msg);
}

void OBCameraNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  std::vector<float> zero_trans = {0, 0, 0};

  auto rotation = camera_params_->r2l_r;
  auto transition = camera_params_->r2l_t;
  auto Q = rotationMatrixToQuaternion(rotation);
  for (int i = 0; i < 9; i++) {
    if (std::isnan(rotation[i])) {
      Q.setRPY(0, 0, 0);
      break;
    }
  }
  Q = quaternion_optical * Q * quaternion_optical.inverse();
  std::vector<float> trans = {transition[0], transition[1], transition[2]};
  if (std::isnan(transition[0]) || std::isnan(transition[1]) || std::isnan(transition[2])) {
    trans[0] = 0;
    trans[1] = 0;
    trans[2] = 0;
  }
  rclcpp::Time tf_timestamp = node_->now();

  publishStaticTF(tf_timestamp, trans, Q, frame_id_[DEPTH], frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, trans, Q, camera_link_frame_id_, frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[COLOR],
                  optical_frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[DEPTH],
                  optical_frame_id_[DEPTH]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[DEPTH]);
  auto ex_msg = obExtrinsicsToMsg(rotation, transition, "depth_to_color_extrinsics");
  ex_msg.header.stamp = node_->now();
  extrinsics_publisher_->publish(ex_msg);
}

void OBCameraNode::publishDynamicTransforms() {
  RCLCPP_WARN(logger_, "Publishing dynamic camera transforms (/tf) at %g Hz", tf_publish_rate_);
  static std::mutex mu;
  std::unique_lock<std::mutex> lock(mu);
  while (rclcpp::ok() && is_running_) {
    tf_cv_.wait_for(lock, std::chrono::milliseconds((int)(1000.0 / tf_publish_rate_)),
                    [this] { return (!(is_running_)); });
    {
      rclcpp::Time t = node_->now();
      for (auto& msg : static_tf_msgs_) {
        msg.header.stamp = t;
      }
      dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
    }
  }
}

void OBCameraNode::publishStaticTransforms() {
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

void OBCameraNode::setImageRegistrationMode(bool data) {
  if (!device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
    RCLCPP_WARN_STREAM(logger_, "Current do not support " << magic_enum::enum_name(
                                    openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR));
    return;
  }
  auto mode = data ? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF;
  auto rc = device_->setImageRegistrationMode(mode);
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR(logger_, "Enabling image registration mode failed: \n%s\n",
                 openni::OpenNI::getExtendedError());
  }
}

void OBCameraNode::onNewFrameCallback(const openni::VideoFrameRef& frame,
                                      const stream_index_pair& stream_index) {
  int width = frame.getWidth();
  int height = frame.getHeight();
  CHECK(images_.count(stream_index));
  auto& image = images_.at(stream_index);
  if (image.size() != cv::Size(width, height)) {
    image.create(height, width, image.type());
  }
  image.data = (uint8_t*)frame.getData();
  auto& camera_info_publisher = camera_info_publishers_.at(stream_index);
  auto& image_publisher = image_publishers_.at(stream_index);

  auto camera_info = getColorCameraInfo();
  if (camera_info->width != static_cast<uint32_t>(width) ||
      camera_info->height != static_cast<uint32_t>(height)) {
    camera_info->width = width;
    camera_info->height = height;
  }
  auto timestamp = node_->now();
  camera_info->header.stamp = timestamp;
  camera_info->header.frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];

  camera_info_publisher->publish(std::move(camera_info));

  auto image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), encoding_.at(stream_index), image).toImageMsg();
  CHECK_NOTNULL(image_msg);
  image_msg->header.stamp = timestamp;
  image_msg->header.frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];
  image_msg->width = width;
  image_msg->height = height;
  image_msg->step = width * unit_step_size_[stream_index];
  image_msg->is_bigendian = false;
  image_publisher.publish(image_msg);
}

void OBCameraNode::setDepthColorSync(bool data) {
  auto rc = device_->setDepthColorSyncEnabled(data);
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR_STREAM(logger_, "Enabling depth color synchronization failed: "
                                     << openni::OpenNI::getExtendedError());
  }
}

void OBCameraNode::setDepthToColorResolution(int width, int height) {
  const auto pid = device_info_.getUsbProductId();
  if (pid != DABAI_DCW_DEPTH_PID && pid != DABAI_DW_PID) {
    return;
  }
  if (!depth_registration_) {
    return;
  }
  if (width * 9 == height * 16) {
    // 16:9
    auto status = device_->setProperty(XN_MODULE_PROPERTY_D2C_RESOLUTION, RGBResolution16_9);
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "setProperty XN_MODULE_PROPERTY_D2C_RESOLUTION "
                                       << openni::OpenNI::getExtendedError());
    }
  } else if (width * 3 == height * 4) {
    // 4:3
    auto status = device_->setProperty(XN_MODULE_PROPERTY_D2C_RESOLUTION, RGBResolution4_3);
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "setProperty XN_MODULE_PROPERTY_D2C_RESOLUTION "
                                       << openni::OpenNI::getExtendedError());
    }
  } else {
    RCLCPP_ERROR_STREAM(logger_, "NOT 16x9 or 4x3 resolution");
  }
}

}  // namespace astra_camera
