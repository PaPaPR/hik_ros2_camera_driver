#include "camera_info_manager/camera_info_manager.hpp"
#include "hik_camera.hpp"
#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

class HikCameraNode : public rclcpp::Node {
 public:
  explicit HikCameraNode(const rclcpp::NodeOptions& options);
  ~HikCameraNode() override;
  void CameraInit();
  void ImgPubLoop();
  void ParamsDecl();
  rcl_interfaces::msg::SetParametersResult ParamsCB(
      const std::vector<rclcpp::Parameter>& parameters);
  template <typename Func, typename... Args>
  auto callWithLog(Func func, Args&&... args) -> decltype(func(args...));

 private:
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfom_;
  std::unique_ptr<hik_camera> camera_;
  image_transport::CameraPublisher camera_pub_;
  OnSetParametersCallbackHandle::SharedPtr params_cb_hl_;
  std::string frame_id_;
  std::thread img_grab_thread_;
  bool is_cinfo_validated_{false};
};

template <typename Func, typename... Args>
auto HikCameraNode::callWithLog(Func func,
                                Args&&... args) -> decltype(func(args...)) {
  try {
    return func(std::forward<Args>(args)...);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), e.what());
    return decltype(func(args...))(-1);
  }
}

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
    : Node("hik_camera_node", options) {
  auto cinfo_path = this->declare_parameter("camera_info_path", "");
  frame_id_ = this->declare_parameter("camera_frame_id", "camera_link");
  auto camera_name = this->declare_parameter("camera_name", "camera_hik");
  auto camera_type = this->declare_parameter("camera_type", "GigE");
  auto camera_serial = this->declare_parameter("camera_serial", "");
  bool use_sensor_data_qos =
      this->declare_parameter("use_sensor_data_qos", true);
  auto qos_profile = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                         : rmw_qos_profile_default;

  camera_ = std::make_unique<hik_camera>(camera_type, camera_serial);
  cinfom_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name);
  camera_pub_ =
      image_transport::create_camera_publisher(this, "hik_camera", qos_profile);

  RCLCPP_INFO(get_logger(),
              "Camera type: %s, serial: %s, info name: %s, info path: %s.",
              camera_type.c_str(), camera_serial.c_str(), camera_name.c_str(),
              cinfo_path.c_str());

  if (cinfom_->validateURL(cinfo_path)) {
    if (cinfom_->loadCameraInfo(cinfo_path)) is_cinfo_validated_ = true;
  } else {
    RCLCPP_WARN(get_logger(), "Invalid camera info path: %s",
                cinfo_path.c_str());
  }

  CameraInit();
  ParamsDecl();
  img_grab_thread_ = std::thread([this]() { ImgPubLoop(); });
}

HikCameraNode::~HikCameraNode() {
  callWithLog([&]() { camera_->stopCapture(); });
  callWithLog([&]() { camera_->release(); });
  RCLCPP_INFO(get_logger(), "Camera released");
}

void HikCameraNode::CameraInit() {
  while (rclcpp::ok()) {
    try {
      RCLCPP_INFO(get_logger(), "Camera initializing.");
      camera_->initialize();
      camera_->startCapture();
      RCLCPP_INFO(get_logger(), "Camera initialized.");
      break;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Camera initialize failed.");
      RCLCPP_ERROR(get_logger(), e.what());
      rclcpp::Rate(1).sleep();
    }
  }
}

void HikCameraNode::ImgPubLoop() {
  int failed_retry_times{-1};

  while (rclcpp::ok()) {
    sensor_msgs::msg::Image img_msg;
    img_msg.header.frame_id = frame_id_;

    try {
      auto frame = camera_->captureImage();
      img_msg.height = frame->height;
      img_msg.width = frame->width;
      img_msg.step = frame->width * frame->channels;
      img_msg.encoding = frame->format;
      img_msg.data = std::move(frame->imageData);
      sensor_msgs::msg::CameraInfo cinfo_msg = cinfom_->getCameraInfo();
      cinfo_msg.header.stamp = img_msg.header.stamp;
      cinfo_msg.header.frame_id = img_msg.header.frame_id;
      camera_pub_.publish(img_msg, cinfo_msg);
    } catch (const std::exception& e) {
      failed_retry_times++;
      RCLCPP_ERROR(get_logger(), e.what());
      if (failed_retry_times > 0)
        RCLCPP_ERROR(get_logger(), "Camera capture image failed, retrying...");
      if (failed_retry_times >= 5) {
        RCLCPP_ERROR(get_logger(), "Camera reinitializing...");
        callWithLog([&]() { camera_->stopCapture(); });
        callWithLog([&]() { camera_->release(); });
        CameraInit();
        failed_retry_times = 0;
      }
    }
  }
}

void HikCameraNode::ParamsDecl() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  unsigned int exp_min, exp_max{0};
  camera_->getExposureRange(exp_min, exp_max);
  int exposure_time = static_cast<int>(camera_->getExposureTime());
  param_desc.description = "Exposure time";
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = exp_min;
  param_desc.integer_range[0].to_value = exp_max > 1000000 ? 1000000 : exp_max;
  this->declare_parameter("exposure_time", exposure_time, param_desc);

  exposure_time = get_parameter("exposure_time").as_int();
  auto exposure_auto = this->declare_parameter("exposure_auto", false);
  callWithLog([&]() { camera_->setExposureAuto(exposure_auto); });
  if (!callWithLog([&]() { return camera_->getExposureAuto(); }))
    callWithLog([&]() { camera_->setExposureTime(exposure_time); });

  unsigned int gain_min, gain_max, gain{0};
  camera_->getGainRange(gain_min, gain_max);
  gain = camera_->getGain();
  param_desc.description = "Gain";
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = gain_min;
  param_desc.integer_range[0].to_value = gain_max;
  this->declare_parameter("gain", static_cast<int>(gain), param_desc);

  gain = get_parameter("gain").as_int();
  auto gain_auto = this->declare_parameter("gain_auto", true);
  callWithLog([&]() { camera_->setGainAuto(gain_auto); });
  if (!callWithLog([&]() { return camera_->getGainAuto(); }))
    callWithLog([&]() { camera_->setGain(gain); });

  unsigned int gamma_min, gamma_max, gamma{0};
  camera_->getGammaRange(gamma_min, gamma_max);
  gamma = camera_->getGamma();
  param_desc.description = "Gamma";
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = gamma_min;
  param_desc.integer_range[0].to_value = gamma_max;
  this->declare_parameter("gamma", static_cast<int>(gamma), param_desc);

  gamma = get_parameter("gamma").as_int();
  auto gamma_enable = this->declare_parameter("gamma_enable", false);
  callWithLog([&]() { camera_->setGammaEnable(gamma_enable); });
  if (callWithLog([&]() { return camera_->getGammaEnable(); }))
    callWithLog([&]() { camera_->setGamma(gamma); });

  RCLCPP_INFO(get_logger(),
              "Camera exposure_auto: %d, gain_auto: %d, gamma_enable: %d",
              exposure_auto, gain_auto, gamma_enable);
  RCLCPP_INFO(get_logger(), "Camera exposure time: %d, gain: %d, gamma: %d",
              exposure_time, gain, gamma);

  params_cb_hl_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::ParamsCB, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::ParamsCB(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& param : parameters) {
    if (param.get_name() == "exposure_auto") {
      callWithLog([&]() { camera_->setExposureAuto(param.as_bool()); });
      if (callWithLog([&]() { return camera_->getExposureAuto(); }) !=
          param.as_bool())
        result.successful = false;
    } else if (param.get_name() == "gain_auto") {
      callWithLog([&]() { camera_->setGainAuto(param.as_bool()); });
      if (callWithLog([&]() { return camera_->getGainAuto(); }) !=
          param.as_bool())
        result.successful = false;
    } else if (param.get_name() == "gamma_enable") {
      callWithLog([&]() { camera_->setGammaEnable(param.as_bool()); });
      if (callWithLog([&]() { return camera_->getGammaEnable(); }) !=
          param.as_bool())
        result.successful = false;
    } else if (param.get_name() == "exposure_time") {
      if (!callWithLog([&]() { return camera_->getExposureAuto(); }))
        callWithLog([&]() { camera_->setExposureTime(param.as_int()); });
      if (callWithLog([&]() { return camera_->getExposureTime(); }) !=
          param.as_int())
        result.successful = false;
    } else if (param.get_name() == "gain") {
      if (!callWithLog([&]() { return camera_->getGainAuto(); }))
        callWithLog([&]() { camera_->setGain(param.as_int()); });
      if (callWithLog([&]() { return camera_->getGain(); }) !=
          param.as_int())
        result.successful = false;
    } else if (param.get_name() == "gamma") {
      if (callWithLog([&]() { return camera_->getGammaEnable(); }))
        callWithLog([&]() { camera_->setGamma(param.as_int()); });
      if (callWithLog([&]() { return camera_->getGamma(); }) !=
          param.as_int())
        result.successful = false;
    }
  }
  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(HikCameraNode)