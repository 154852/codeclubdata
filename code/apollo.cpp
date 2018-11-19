/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/localization/msf/local_integ/localization_integ_process.h"

#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/time/timer.h"
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {
namespace msf {

using apollo::common::Status;

LocalizationIntegProcess::LocalizationIntegProcess()
    : sins_(new Sins()),
      gnss_antenna_extrinsic_(TransformD::Identity()),
      integ_state_(IntegState::NOT_INIT),
      ins_pva_(),
      pva_covariance_{0.0},
      keep_running_(false),
      measure_data_thread_(),
      measure_data_queue_size_(150),
      delay_output_counter_(0) {}

LocalizationIntegProcess::~LocalizationIntegProcess() {
  StopThreadLoop();

  delete sins_;
  sins_ = nullptr;
}

Status LocalizationIntegProcess::Init(const LocalizationIntegParam &param) {
  // sins init
  sins_->Init(param.is_ins_can_self_align);
  sins_->SetSinsAlignFromVel(param.is_sins_align_with_vel);

  sins_->SetResetSinsPoseStd(param.sins_state_pos_std);
  sins_->SetResetSinsMeasSpanTime(param.sins_state_span_time);

  if (param.is_using_raw_gnsspos) {
    gnss_antenna_extrinsic_.translation()(0) = param.imu_to_ant_offset.offset_x;
    gnss_antenna_extrinsic_.translation()(1) = param.imu_to_ant_offset.offset_y;
    gnss_antenna_extrinsic_.translation()(2) = param.imu_to_ant_offset.offset_z;
  } else {
    gnss_antenna_extrinsic_ = TransformD::Identity();
  }
  AINFO << "gnss and imu lever arm: "
        << gnss_antenna_extrinsic_.translation()(0) << " "
        << gnss_antenna_extrinsic_.translation()(1) << " "
        << gnss_antenna_extrinsic_.translation()(2);

  sins_->SetImuAntennaLeverArm(gnss_antenna_extrinsic_.translation()(0),
                               gnss_antenna_extrinsic_.translation()(1),
                               gnss_antenna_extrinsic_.translation()(2));

  sins_->SetVelThresholdGetYaw(param.vel_threshold_get_yaw);

  StartThreadLoop();

  return Status::OK();
}

void LocalizationIntegProcess::RawImuProcess(const ImuData &imu_msg) {
  integ_state_ = IntegState::NOT_INIT;
  double cur_imu_time = imu_msg.measurement_time;

  if (cur_imu_time < 3000) {
    AERROR << "the imu time is error: " << cur_imu_time;
    return;
  }

  static double pre_imu_time = cur_imu_time;
  double delta_time = cur_imu_time - pre_imu_time;
  if (delta_time > 0.1) {
    ADEBUG << std::setprecision(16) << "the imu message loss more than 10, "
           << "the pre time and current time: " << pre_imu_time << " "
           << cur_imu_time;
  } else if (delta_time < 0.0) {
    ADEBUG << std::setprecision(16)
           << "received imu message's time is eary than last imu message, "
           << "the pre time and current time: " << pre_imu_time << " "
           << cur_imu_time;
  }

  // add imu msg and get current predict pose
  sins_->AddImu(imu_msg);
  sins_->GetPose(&ins_pva_, pva_covariance_);

  if (sins_->IsSinsAligned()) {
    integ_state_ = IntegState::NOT_STABLE;
    if (delay_output_counter_ < 3000) {
      ++delay_output_counter_;
    } else {
      integ_state_ = IntegState::OK;
      GetValidFromOK();
    }

    if (cur_imu_time - 0.5 > pre_imu_time) {
      AINFO << "SINS has completed alignment!" << std::endl;
      pre_imu_time = cur_imu_time;
    }
  } else {
    delay_output_counter_ = 0;
    if (cur_imu_time - 0.5 > pre_imu_time) {
      AINFO << "SINS is aligning!" << std::endl;
      pre_imu_time = cur_imu_time;
    }
  }

  pre_imu_time = cur_imu_time;

  return;
}

void LocalizationIntegProcess::GetValidFromOK() {
  if (integ_state_ != IntegState::OK) {
    return;
  }

  // AERROR << pva_covariance_[0][0] << " " << pva_covariance_[1][1]
  //     << " " << pva_covariance_[2][2] << " " << pva_covariance_[8][8];
  if (pva_covariance_[0][0] < 0.3 * 0.3 && pva_covariance_[1][1] < 0.3 * 0.3 &&
      pva_covariance_[2][2] < 0.3 * 0.3 && pva_covariance_[8][8] < 0.1 * 0.1) {
    integ_state_ = IntegState::VALID;
  }
  return;
}

void LocalizationIntegProcess::GetState(IntegState *state) {
  CHECK_NOTNULL(state);

  *state = integ_state_;
  return;
}

void LocalizationIntegProcess::GetResult(IntegState *state,
                                         LocalizationEstimate *localization) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(localization);

  // state
  *state = integ_state_;

  // // IntegSinsPva
  // *sins_pva = ins_pva_;

  if (*state != IntegState::NOT_INIT) {
    ADEBUG << std::setprecision(16)
           << "IntegratedLocalization Debug Log: integ_pose msg: "
           << "[time:" << ins_pva_.time << "]"
           << "[x:" << ins_pva_.pos.longitude * 57.295779513082323 << "]"
           << "[y:" << ins_pva_.pos.latitude * 57.295779513082323 << "]"
           << "[z:" << ins_pva_.pos.height << "]"
           << "[ve:" << ins_pva_.vel.ve << "]"
           << "[vn:" << ins_pva_.vel.vn << "]"
           << "[vu:" << ins_pva_.vel.vu << "]"
           << "[pitch: " << ins_pva_.att.pitch * 57.295779513082323 << "]"
           << "[roll:" << ins_pva_.att.roll * 57.295779513082323 << "]"
           << "[yaw:" << ins_pva_.att.yaw * 57.295779513082323 << "]";
  }

  // LocalizationEstimation
  apollo::common::Header *headerpb_loc = localization->mutable_header();
  apollo::localization::Pose *posepb_loc = localization->mutable_pose();

  localization->set_measurement_time(ins_pva_.time);
  headerpb_loc->set_timestamp_sec(apollo::common::time::Clock::NowInSeconds());
  // headerpb_loc->set_module_name(_param.publish_frame_id);

  apollo::common::PointENU *position_loc = posepb_loc->mutable_position();
  apollo::common::Quaternion *quaternion = posepb_loc->mutable_orientation();
  UTMCoor utm_xy;
  LatlonToUtmXY(ins_pva_.pos.longitude, ins_pva_.pos.latitude, &utm_xy);
  position_loc->set_x(utm_xy.x);
  position_loc->set_y(utm_xy.y);
  position_loc->set_z(ins_pva_.pos.height);

  quaternion->set_qx(ins_pva_.qbn[1]);
  quaternion->set_qy(ins_pva_.qbn[2]);
  quaternion->set_qz(ins_pva_.qbn[3]);
  quaternion->set_qw(ins_pva_.qbn[0]);

  apollo::common::Point3D *velocitylinear =
      posepb_loc->mutable_linear_velocity();
  velocitylinear->set_x(ins_pva_.vel.ve);
  velocitylinear->set_y(ins_pva_.vel.vn);
  velocitylinear->set_z(ins_pva_.vel.vu);

  apollo::common::Point3D *eulerangles = posepb_loc->mutable_euler_angles();
  eulerangles->set_x(ins_pva_.att.pitch);
  eulerangles->set_y(ins_pva_.att.roll);
  eulerangles->set_z(ins_pva_.att.yaw);

  posepb_loc->set_heading(ins_pva_.att.yaw);

  apollo::localization::Uncertainty *uncertainty =
      localization->mutable_uncertainty();
  apollo::common::Point3D *position_std_dev =
      uncertainty->mutable_position_std_dev();
  position_std_dev->set_x(std::sqrt(pva_covariance_[0][0]));
  position_std_dev->set_y(std::sqrt(pva_covariance_[1][1]));
  position_std_dev->set_z(std::sqrt(pva_covariance_[2][2]));

  apollo::common::Point3D *linear_velocity_std_dev =
      uncertainty->mutable_linear_velocity_std_dev();
  linear_velocity_std_dev->set_x(std::sqrt(pva_covariance_[3][3]));
  linear_velocity_std_dev->set_y(std::sqrt(pva_covariance_[4][4]));
  linear_velocity_std_dev->set_z(std::sqrt(pva_covariance_[5][5]));

  apollo::common::Point3D *orientation_std_dev =
      uncertainty->mutable_orientation_std_dev();
  orientation_std_dev->set_x(std::sqrt(pva_covariance_[6][6]));
  orientation_std_dev->set_y(std::sqrt(pva_covariance_[7][7]));
  orientation_std_dev->set_z(std::sqrt(pva_covariance_[8][8]));
  return;
}

void LocalizationIntegProcess::GetResult(IntegState *state, InsPva *sins_pva,
                                         double pva_covariance[9][9]) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(sins_pva);
  CHECK_NOTNULL(pva_covariance);

  *state = integ_state_;
  *sins_pva = ins_pva_;
  memcpy(pva_covariance, pva_covariance_, sizeof(double) * 9 * 9);
  return;
}

void LocalizationIntegProcess::MeasureDataProcess(
    const MeasureData &measure_msg) {
  measure_data_queue_mutex_.lock();
  measure_data_queue_.push(measure_msg);
  new_measure_data_signal_.notify_one();
  measure_data_queue_mutex_.unlock();
}

void LocalizationIntegProcess::StartThreadLoop() {
  keep_running_ = true;
  measure_data_queue_size_ = 150;
  const auto &loop_func = [this] { MeasureDataThreadLoop(); };
  measure_data_thread_ = std::thread(loop_func);
}

void LocalizationIntegProcess::StopThreadLoop() {
  if (keep_running_.load()) {
    keep_running_ = false;
    new_measure_data_signal_.notify_one();
    measure_data_thread_.join();
  }
}

void LocalizationIntegProcess::MeasureDataThreadLoop() {
  AINFO << "Started measure data process thread";
  while (keep_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(measure_data_queue_mutex_);
      int size = measure_data_queue_.size();
      while (size > measure_data_queue_size_) {
        measure_data_queue_.pop();
        --size;
      }
      if (measure_data_queue_.size() == 0) {
        new_measure_data_signal_.wait(lock);
        continue;
      }
    }

    MeasureData measure;
    {
      std::unique_lock<std::mutex> lock(measure_data_queue_mutex_);
      measure = measure_data_queue_.front();
      measure_data_queue_.pop();
    }
    MeasureDataProcessImpl(measure);
  }
  AINFO << "Exited measure data process thread";
}

void LocalizationIntegProcess::MeasureDataProcessImpl(
    const MeasureData &measure_msg) {
  common::time::Timer timer;
  timer.Start();

  if (!CheckIntegMeasureData(measure_msg)) {
    return;
  }

  sins_->AddMeasurement(measure_msg);

  timer.End("time of integrated navigation measure update");
  return;
}

bool LocalizationIntegProcess::CheckIntegMeasureData(
    const MeasureData &measure_data) {
  if (measure_data.measure_type == MeasureType::ODOMETER_VEL_ONLY) {
    AERROR << "receive a new odometry measurement!!!\n";
  }

  ADEBUG << std::setprecision(16)
         << "IntegratedLocalization Debug Log: measure data: "
         << "[time:" << measure_data.time << "]"
         << "[x:" << measure_data.gnss_pos.longitude * 57.295779513082323 << "]"
         << "[y:" << measure_data.gnss_pos.latitude * 57.295779513082323 << "]"
         << "[z:" << measure_data.gnss_pos.height << "]"
         << "[ve:" << measure_data.gnss_vel.ve << "]"
         << "[vn:" << measure_data.gnss_vel.vn << "]"
         << "[vu:" << measure_data.gnss_vel.vu << "]"
         << "[pitch:" << measure_data.gnss_att.pitch * 57.295779513082323 << "]"
         << "[roll:" << measure_data.gnss_att.roll * 57.295779513082323 << "]"
         << "[yaw:" << measure_data.gnss_att.yaw * 57.295779513082323 << "]"
         << "[measure type:" << int(measure_data.measure_type) << "]";

  return true;
}

bool LocalizationIntegProcess::LoadGnssAntennaExtrinsic(
    const std::string &file_path, TransformD *extrinsic) const {
  CHECK_NOTNULL(extrinsic);

  YAML::Node confige = YAML::LoadFile(file_path);
  if (confige["leverarm"]) {
    if (confige["leverarm"]["primary"]["offset"]) {
      extrinsic->translation()(0) =
          confige["leverarm"]["primary"]["offset"]["x"].as<double>();
      extrinsic->translation()(1) =
          confige["leverarm"]["primary"]["offset"]["y"].as<double>();
      extrinsic->translation()(2) =
          confige["leverarm"]["primary"]["offset"]["z"].as<double>();
      return true;
    }
  }

  return false;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#include <time.h>   /* for clock_gettime */

#include "image_transport/camera_common.h"
#include "image_transport/publisher_plugin.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/drivers/camera/common/camera_gflags.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include "modules/drivers/camera/nodes/usb_cam_wrapper.h"

namespace apollo {
namespace drivers {
namespace camera {

UsbCamWrapper::UsbCamWrapper(
    ros::NodeHandle node, ros::NodeHandle private_nh, CameraConf config) :
    node_(node), priv_node_(private_nh), config_(config), last_stamp_(0) {
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_camera_config_file,
                                                &config_)) {
    AERROR << "Unable to load camera conf file: " << FLAGS_camera_config_file;
    return;
  }

  // pb
  // http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
  sensor_image_.mutable_header()->set_camera_timestamp(
      img_.header.stamp.toSec());
  sensor_image_.set_frame_id(img_.header.frame_id);
  // TODO(all) sensor_image_.set_measurement_time();
  // image height, that is, number of rows
  sensor_image_.set_height(img_.height);
  // image width, that is, number of columns
  sensor_image_.set_width(img_.width);
  sensor_image_.set_encoding(img_.encoding);
  // Full row length in bytes
  sensor_image_.set_step(img_.step);
  // actual matrix data, size is (step * rows)
  size_t data_length = img_.step * img_.height;
  std::string image_data;
  image_data.assign(reinterpret_cast<const char *>(img_.data.data()),
                    data_length);
  sensor_image_.set_data(image_data);

  cinfo_.reset(new camera_info_manager::CameraInfoManager(
      node_, config_.camera_name(), config_.camera_info_url()));

  // default 3000 ms


  // Warning when diff with last > 1.5* interval
  frame_warning_interval_ = 1.5 / config_.frame_rate();
  // now max fps 30, we use a appox time 0.9 to drop image.
  frame_drop_interval_ = 0.9 / config_.frame_rate();

  // advertise the main image topic
  // image_transport::ImageTransport it(node_);
  // image_pub_ = it.advertiseCamera(topic_name_, 1);
  // Load transport publish plugin
  std::string image_topic = node_.resolveName(config_.topic_name());
  pub_loader_ = boost::make_shared<image_transport::PubLoader>(
      "image_transport", "image_transport::PublisherPlugin");
  std::string lookup_name = image_transport::PublisherPlugin::getLookupName(
      std::string("raw"));
  image_pub_plugin_ = pub_loader_->createInstance(lookup_name);
  if (image_pub_plugin_ != nullptr) {
    image_pub_plugin_->advertise(node_, image_topic, 1,
                                 image_transport::SubscriberStatusCallback(),
           image_transport::SubscriberStatusCallback(), ros::VoidPtr(), false);
  } else {
    AERROR << "create image publish plugin error. lookup_name: " << lookup_name;
    node_.shutdown();
    return;
  }

  // camera info publish
  std::string cam_info_topic =
      image_transport::getCameraInfoTopic(image_topic);
  cam_info_pub_ = node_.advertise<sensor_msgs::CameraInfo>(
      cam_info_topic, 1, ros::SubscriberStatusCallback(),
      ros::SubscriberStatusCallback(), ros::VoidPtr(), false);

  // create Services
  service_start_ = node_.advertiseService(
      "start_capture", &UsbCamWrapper::service_start_cap, this);
  service_stop_ = node_.advertiseService(
      "stop_capture", &UsbCamWrapper::service_stop_cap, this);

  // check for default camera info
  if (!cinfo_->isCalibrated()) {
    cinfo_->setCameraName(config_.video_device());
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = config_.image_width();
    camera_info.height = config_.image_height();
    cinfo_->setCameraInfo(camera_info);
  }
  // get the camera basical infomation
  cam_info_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

  AINFO << "Starting '" << config_.camera_name()
      << "' (" << config_.video_device()
      << ") at " << config_.image_width() << "x" << config_.image_height()
      << " via " << config_.io_method()  << " (" << config_.pixel_format()
      << ") at %" << config_.frame_rate() << " FPS";

  // set the IO method
  UsbCam::io_method io_method =
      UsbCam::io_method_from_string(config_.io_method());

  if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
    AFATAL << "Unknown IO method '" << config_.io_method() << "'";
    node_.shutdown();
    return;
  }

  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(
      config_.pixel_format());

  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
    AFATAL << "Unknown pixel format '" << config_.pixel_format() << "'";
    node_.shutdown();
    return;
  }

  // start the camera
  cam_.start(config_.video_device(),
             io_method, pixel_format,
             config_.image_width(), config_.image_height(),
             config_.frame_rate());

  // set camera parameters
  if (config_.brightness() >= 0) {
    cam_.set_v4l_parameter("brightness", config_.brightness());
  }

  if (config_.contrast() >= 0) {
    cam_.set_v4l_parameter("contrast", config_.contrast());
  }

  if (config_.saturation() >= 0) {
    cam_.set_v4l_parameter("saturation", config_.saturation());
  }

  if (config_.sharpness() >= 0) {
    cam_.set_v4l_parameter("sharpness", config_.sharpness());
  }

  if (config_.gain() >= 0) {
    cam_.set_v4l_parameter("gain", config_.gain());
  }

  // check auto white balance
  if (config_.auto_white_balance()) {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
  } else {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
    cam_.set_v4l_parameter("white_balance_temperature",
                           config_.white_balance());
  }

  // check auto exposure
  if (!config_.autoexposure()) {
    // turn down exposure control (from max of 3)
    cam_.set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    cam_.set_v4l_parameter("exposure_absolute", config_.exposure());
  }

  // check auto focus
  if (config_.autofocus()) {
    cam_.set_auto_focus(1);
    cam_.set_v4l_parameter("focus_auto", 1);
  } else {
    cam_.set_v4l_parameter("focus_auto", 0);

    if (config_.focus() >= 0) {
      cam_.set_v4l_parameter("focus_absolute", config_.focus());
    }
  }

  // trigger enable
  int trigger_ret = cam_.trigger_enable(config_.trigger_fps(),
                                        config_.trigger_internal());
  if (0 != trigger_ret) {
    AWARN << "Camera trigger Fail ret: " << trigger_ret;
    // node_.shutdown();
    // return;
  }
}

UsbCamWrapper::~UsbCamWrapper() {
  cam_.shutdown();
}

bool UsbCamWrapper::service_start_cap(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res) {
  cam_.start_capturing();
  return true;
}

bool UsbCamWrapper::service_stop_cap(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res) {
  cam_.stop_capturing();
  return true;
}

bool UsbCamWrapper::take_and_send_image() {
  // grab the image
  bool get_new_image = cam_.grab_image(&img_, config_.camera_timeout());

  if (!get_new_image) {
    return false;
  }

  // grab the camera info
  // cam_info_ = sensor_msgs::CameraInfo(cinfo_->getCameraInfo());
  cam_info_->header.frame_id = img_.header.frame_id;
  cam_info_->header.stamp = img_.header.stamp;

  if (last_stamp_ == ros::Time(0)) {
    last_stamp_ = img_.header.stamp;
  } else {
    auto diff = (img_.header.stamp - last_stamp_).toSec();
    // drop image by frame_rate
    if (diff < frame_drop_interval_) {
      ROS_INFO_STREAM("drop image:" << img_.header.stamp);
      return true;
    }
    if (frame_warning_interval_ < diff) {
      ROS_WARN_STREAM("stamp jump.last stamp:" << last_stamp_
          << " current stamp:" << img_.header.stamp);
    }
    last_stamp_ = img_.header.stamp;
  }

  // publish the image
  image_pub_plugin_->publish(img_);
  cam_info_pub_.publish(cam_info_);

  return true;
}

bool UsbCamWrapper::spin() {
  // spin loop rate should be in accord with the trigger frequence
  ros::Duration loop_interval(config_.spin_interval());

  while (node_.ok()) {
    if (cam_.is_capturing()) {
      if (!take_and_send_image()) {
        AERROR << "USB camera did not respond in time.";
      }
    }
    // ros::spinOnce();
    loop_interval.sleep();
  }
  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/guardian/guardian.h"

#include <cmath>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/guardian/common/guardian_gflags.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace guardian {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::control::ControlCommand;
using apollo::guardian::GuardianCommand;
using apollo::monitor::SystemStatus;

std::string Guardian::Name() const { return FLAGS_module_name; }

Status Guardian::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  CHECK(AdapterManager::GetSystemStatus())
      << "SystemStatus is not initialized.";
  CHECK(AdapterManager::GetControlCommand()) << "Control is not initialized.";
  return Status::OK();
}

Status Guardian::Start() {
  AdapterManager::AddChassisCallback(&Guardian::OnChassis, this);
  AdapterManager::AddSystemStatusCallback(&Guardian::OnSystemStatus, this);
  AdapterManager::AddControlCommandCallback(&Guardian::OnControl, this);
  const double duration = 1.0 / FLAGS_guardian_cmd_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &Guardian::OnTimer, this);

  return Status::OK();
}

void Guardian::Stop() { timer_.stop(); }

void Guardian::OnTimer(const ros::TimerEvent&) {
  ADEBUG << "Timer is triggered: publish Guardian result";
  bool safety_mode_triggered = false;
  if (FLAGS_guardian_enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    safety_mode_triggered = system_status_.has_safety_mode_trigger_time();
  }

  if (safety_mode_triggered) {
    ADEBUG << "Safety mode triggered, enable safty mode";
    TriggerSafetyMode();
  } else {
    ADEBUG << "Safety mode not triggered, bypass control command";
    PassThroughControlCommand();
  }

  AdapterManager::FillGuardianHeader(FLAGS_node_name, &guardian_cmd_);
  AdapterManager::PublishGuardian(guardian_cmd_);
}

void Guardian::OnChassis(const Chassis& message) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  chassis_.CopyFrom(message);
}

void Guardian::OnSystemStatus(const SystemStatus& message) {
  ADEBUG << "Received monitor data: run monitor callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  system_status_.CopyFrom(message);
}

void Guardian::OnControl(const ControlCommand& message) {
  ADEBUG << "Received control data: run control command callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  control_cmd_.CopyFrom(message);
}

void Guardian::PassThroughControlCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  guardian_cmd_.mutable_control_command()->CopyFrom(control_cmd_);
}

void Guardian::TriggerSafetyMode() {
  AINFO << "Safety state triggered, with system safety mode trigger time : "
        << system_status_.safety_mode_trigger_time();
  std::lock_guard<std::mutex> lock(mutex_);
  bool sensor_malfunction = false, obstacle_detected = false;
  if (!chassis_.surround().sonar_enabled() ||
      chassis_.surround().sonar_fault()) {
    AINFO << "Ultrasonic sensor not enabled for faulted, will do emergency "
             "stop!";
    sensor_malfunction = true;
  } else {
    // TODO(QiL) : Load for config
    for (int i = 0; i < chassis_.surround().sonar_range_size(); ++i) {
      if ((chassis_.surround().sonar_range(i) > 0.0 &&
           chassis_.surround().sonar_range(i) < 2.5) ||
          chassis_.surround().sonar_range(i) > 30) {
        AINFO << "Object detected or ultrasonic sensor fault output, will do "
                 "emergency stop!";
        obstacle_detected = true;
      }
    }
  }

  guardian_cmd_.mutable_control_command()->set_throttle(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_target(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_rate(25.0);
  guardian_cmd_.mutable_control_command()->set_is_in_safe_mode(true);

  // TODO(QiL) : Remove this one once hardware re-alignment is done.
  sensor_malfunction = false;
  obstacle_detected = false;
  AINFO << "Temporarily ignore the ultrasonic sensor output during hardware "
           "re-alignment!";

  if (system_status_.require_emergency_stop() || sensor_malfunction ||
      obstacle_detected) {
    AINFO << "Emergency stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        FLAGS_guardian_cmd_emergency_stop_percentage);
  } else {
    AINFO << "Soft stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        FLAGS_guardian_cmd_soft_stop_percentage);
  }
}

}  // namespace guardian
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/recognizer/classify.h"

#include <vector>

#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

ClassifyBySimple::ClassifyBySimple(const std::string &class_net_,
                                   const std::string &class_model_,
                                   float threshold, unsigned int resize_width,
                                   unsigned int resize_height) {
  Init(class_net_, class_model_, threshold, resize_width, resize_height);
}

void ClassifyBySimple::SetCropBox(const cv::Rect &box) { crop_box_ = box; }
void ClassifyBySimple::Init(const std::string &class_net_,
                            const std::string &class_model_, float threshold,
                            unsigned int resize_width,
                            unsigned int resize_height) {
  AINFO << "Creating testing net...";
  classify_net_ptr_.reset(new caffe::Net<float>(class_net_, caffe::TEST));

  AINFO << "restore parameters...";
  classify_net_ptr_->CopyTrainedLayersFrom(class_model_);

  resize_height_ = resize_height;
  resize_width_ = resize_width;
  unknown_threshold_ = threshold;

  AINFO << "Init Done";
}

void ClassifyBySimple::Perform(const cv::Mat &ros_image,
                               std::vector<LightPtr> *lights) {
  caffe::Blob<float> *input_blob_recog = classify_net_ptr_->input_blobs()[0];
  caffe::Blob<float> *output_blob_recog =
      classify_net_ptr_
          ->top_vecs()[classify_net_ptr_->top_vecs().size() - 1][0];
  cv::Mat img = ros_image(crop_box_);
  for (LightPtr light : *lights) {
    if (!light->region.is_detected ||
        !BoxIsValid(light->region.rectified_roi, ros_image.size())) {
      continue;
    }

    cv::Mat img_light = img(light->region.rectified_roi).clone();
    assert(img_light.rows > 0);
    assert(img_light.cols > 0);

    cv::resize(img_light, img_light, cv::Size(resize_width_, resize_height_));
    float *data = input_blob_recog->mutable_cpu_data();
    uchar *pdata = img_light.data;
    for (int h = 0; h < resize_height_; ++h) {
      pdata = img_light.data + h * img_light.step;
      for (int w = 0; w < resize_width_; ++w) {
        for (int channel = 0; channel < 3; channel++) {
          int index = (channel * resize_height_ + h) * resize_width_ + w;
          data[index] = static_cast<float>((*pdata));
          ++pdata;
        }
      }
    }

    classify_net_ptr_->ForwardFrom(0);
    float *out_put_data = output_blob_recog->mutable_cpu_data();
    ProbToColor(out_put_data, unknown_threshold_, light);
  }
}

void ClassifyBySimple::ProbToColor(const float *out_put_data, float threshold,
                                   LightPtr light) {
  int max_color_id = 0;
  std::vector<TLColor> status_map = {BLACK, RED, YELLOW, GREEN};
  std::vector<std::string> name_map = {"Black", "Red", "Yellow", "Green"};
  std::vector<float> prob(out_put_data, out_put_data + status_map.size());
  auto max_prob = std::max_element(prob.begin(), prob.end());
  max_color_id = (*max_prob > threshold)
                     ? static_cast<int>(std::distance(prob.begin(), max_prob))
                     : 0;

  light->status.color = status_map[max_color_id];
  light->status.confidence = out_put_data[max_color_id];
  AINFO << "Light status recognized as " << name_map[max_color_id];
  AINFO << "Color Prob:";
  for (size_t j = 0; j < status_map.size(); ++j) {
    AINFO << out_put_data[j];
  }
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/recognizer/unity_recognize.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/traffic_light/recognizer/classify.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;

bool UnityRecognize::Init() {
  if (!GetProtoFromFile(FLAGS_traffic_light_recognizer_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_recognizer_config;
    return false;
  }

  if (config_.recognizer_config_size() != 2) {
    AERROR << "RecognizeConfig size should be 2.";
    return false;
  }
  for (const auto &recognizer_config : config_.recognizer_config()) {
    if (recognizer_config.name() == "UnityRecognizeNight") {
      classify_night_ = std::make_shared<ClassifyBySimple>(
          recognizer_config.classify_net(), recognizer_config.classify_model(),
          recognizer_config.classify_threshold(),
          static_cast<unsigned int>(recognizer_config.classify_resize_width()),
          static_cast<unsigned int>(
              recognizer_config.classify_resize_height()));
    }
    if (recognizer_config.name() == "UnityRecognize") {
      classify_day_ = std::make_shared<ClassifyBySimple>(
          recognizer_config.classify_net(), recognizer_config.classify_model(),
          recognizer_config.classify_threshold(),
          static_cast<unsigned int>(recognizer_config.classify_resize_width()),
          static_cast<unsigned int>(
              recognizer_config.classify_resize_height()));
    }
  }
  return true;
}

bool UnityRecognize::RecognizeStatus(const Image &image,
                                     const RecognizeOption &option,
                                     std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  cv::Rect cbox;
  cbox = cv::Rect(0, 0, ros_image.cols, ros_image.rows);
  classify_night_->SetCropBox(cbox);
  classify_day_->SetCropBox(cbox);
  std::vector<LightPtr> candidate(1);
  for (LightPtr light : *lights) {
    if (light->region.is_detected) {
      candidate[0] = light;
      if (light->region.detect_class_id == QUADRATE_CLASS) {
        AINFO << "Recognize Use Night Model!";
        classify_night_->Perform(ros_image, &candidate);
      } else if (light->region.detect_class_id == VERTICAL_CLASS) {
        AINFO << "Recognize Use Day Model!";
        classify_day_->Perform(ros_image, &candidate);
      } else {
        AINFO << "Not support yet!";
      }
    } else {
      light->status.color = UNKNOWN_COLOR;
      light->status.confidence = 0;
      AINFO << "Unknown Detection Class: " << light->region.detect_class_id
            << ". region.is_detected: " << light->region.is_detected
            << ". Not perform recognition.";
    }
  }
  return true;
}

std::string UnityRecognize::name() const { return "UnityRecognize"; }

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "modules/common/util/file.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;

bool MultiCamerasProjection::Init() {
  if (!GetProtoFromFile(FLAGS_traffic_light_multi_camera_projection_config,
                        &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_multi_camera_projection_config;
    return false;
  }
  // Read camera names from config file
  const std::string &single_projection_name =
      config_.multi_camera_projection_config().single_projection();

  // Read each camera's config
  std::unordered_map<std::string, CameraCoeffient> camera_coeffients;
  for (const auto &camera_focus_config :
       config_.multi_camera_projection_config().camera_focus_config()) {
    const auto &camera_model_name = camera_focus_config.name();
    CameraCoeffient camera_coeffient;
    if (!camera_coeffient.init(camera_model_name,
                               camera_focus_config.camera_extrinsic_file(),
                               camera_focus_config.camera_intrinsic_file())) {
      AERROR << camera_model_name << " Projection init failed.";
      return false;
    }
    camera_coeffients[camera_model_name] = camera_coeffient;
    camera_names_.push_back(camera_model_name);
  }

  projection_.reset(
      BaseProjectionRegisterer::GetInstanceByName(single_projection_name));
  if (projection_ == nullptr) {
    AERROR << "MultiCamerasProjection new projection failed. name:"
           << single_projection_name;
    return false;
  }
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    auto &camera_coeffient = camera_coeffients[camera_names_[i]];
    camera_coeffient.camera_extrinsic =
        camera_coeffient.camera_extrinsic.inverse().eval();

    AINFO << "Lidar to " << camera_names_[i] << " transform: ";
    AINFO << camera_coeffient.camera_extrinsic;
  }
  camera_coeffient_.resize(camera_names_.size());
  camera_coeffient_[kLongFocusIdx] = camera_coeffients["camera_25mm_focus"];
  camera_coeffient_[kShortFocusIdx] = camera_coeffients["camera_6mm_focus"];
  // auto &short_focus_camera_coeffient = camera_coeffients["camera_6mm_focus"];
  // auto &long_focus_camera_coeffient = camera_coeffients["camera_25mm_focus"];
  camera_coeffient_[kLongFocusIdx].camera_extrinsic =
      camera_coeffient_[kLongFocusIdx].camera_extrinsic *
      camera_coeffient_[kShortFocusIdx].camera_extrinsic;
  AINFO << "Lidar to long(25mm): ";
  AINFO << camera_coeffient_[kLongFocusIdx].camera_extrinsic;
  return true;
}

bool MultiCamerasProjection::Project(const CarPose &pose,
                                     const ProjectOption &option,
                                     Light *light) const {
  const Eigen::Matrix4d mpose = pose.pose();
  const apollo::hdmap::Signal &tl_info = light->info;
  bool ret = true;

  auto camera_id = static_cast<int>(option.camera_id);
  if (camera_id < 0 || camera_id >= kCountCameraId) {
    AERROR << "Projection get invalid camera_id: " << camera_id
           << ", check camera parameters file.";
    return false;
  }
  AINFO << "Begin project camera: " << option.camera_id;
  ret =
      projection_->Project(camera_coeffient_[camera_id], mpose, tl_info, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_id: " << camera_id;
    return false;
  }
  return true;
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/projection/projection.h"

#include <algorithm>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool BoundaryProjection::Project(const CameraCoeffient &camera_coeffient,
                                 const Eigen::Matrix4d &pose,
                                 const apollo::hdmap::Signal &tl_info,
                                 Light *light) const {
  int bound_size = tl_info.boundary().point_size();
  if (bound_size < 4) {
    AERROR << "Light boundary should be rectangle, which has four points! Got :"
           << bound_size;
    return false;
  }

  std::vector<int> x(bound_size);
  std::vector<int> y(bound_size);

  for (int i = 0; i < bound_size; ++i) {
    if (!ProjectPointDistort(camera_coeffient, pose,
                             tl_info.boundary().point(i), &x[i], &y[i])) {
      return false;
    }
  }
  int minx = std::min(x[0], x[2]);
  int miny = std::min(y[0], y[2]);
  int maxx = std::max(x[0], x[2]);
  int maxy = std::max(y[0], y[2]);

  cv::Rect roi(minx, miny, maxx - minx, maxy - miny);
  AINFO << "projection get ROI:" << roi;
  if (minx < 0 || miny < 0 ||
      maxx >= static_cast<int>(camera_coeffient.image_width) ||
      maxy >= static_cast<int>(camera_coeffient.image_height)) {
    AWARN << "Projection get ROI outside the image. ";
    return false;
  }
  light->region.projection_roi = RefinedBox(
      roi,
      cv::Size(camera_coeffient.image_width, camera_coeffient.image_height));
  AINFO << "refined ROI:" << light->region.projection_roi;

  return true;
}
bool BoundaryProjection::ProjectPoint(const CameraCoeffient &coeffient,
                                      const Eigen::Matrix4d &pose,
                                      const apollo::common::Point3D &point,
                                      int *center_x, int *center_y) const {
  Eigen::Matrix<double, 4, 1> TL_loc_LTM;
  Eigen::Matrix<double, 3, 1> TL_loc_cam;

  TL_loc_LTM << point.x(), point.y(), point.z() + FLAGS_light_height_adjust,
      1.0;
  TL_loc_LTM = coeffient.camera_extrinsic * pose.inverse() * TL_loc_LTM;

  // The light may behind the car, we can't project them on the images.
  if (TL_loc_LTM(2) < 0) {
    AWARN << "Compute a light behind the car. light to car Pose:\n"
          << TL_loc_LTM;
    return false;
  }
  TL_loc_cam = coeffient.camera_intrinsic * TL_loc_LTM;

  TL_loc_cam /= TL_loc_cam(2, 0);
  *center_x = static_cast<int>(TL_loc_cam(0, 0));
  *center_y = static_cast<int>(TL_loc_cam(1, 0));

  return true;
}

bool BoundaryProjection::ProjectPointDistort(const CameraCoeffient &coeffient,
                                             const Eigen::Matrix4d &pose,
                                             const common::PointENU &point,
                                             int *center_x,
                                             int *center_y) const {
  Eigen::Matrix<double, 4, 1> TL_loc_LTM;
  Eigen::Matrix<double, 3, 1> TL_loc_cam;

  TL_loc_LTM << point.x(), point.y(), point.z() + FLAGS_light_height_adjust,
      1.0;
  TL_loc_LTM = coeffient.camera_extrinsic * pose.inverse() * TL_loc_LTM;

  if (TL_loc_LTM(2) < 0) {
    AWARN << "Compute a light behind the car. light to car Pose:\n"
          << TL_loc_LTM;
    return false;
  }

  Eigen::Matrix<double, 2, 1> pt2d;
  pt2d[0] = TL_loc_LTM[0] / TL_loc_LTM[2];
  pt2d[1] = TL_loc_LTM[1] / TL_loc_LTM[2];

  pt2d = PixelDenormalize(pt2d, coeffient.camera_intrinsic,
                          coeffient.distort_params);
  *center_x = pt2d[0];
  *center_y = pt2d[1];
  return true;
}

Eigen::Matrix<double, 2, 1> BoundaryProjection::PixelDenormalize(
    const Eigen::Matrix<double, 2, 1> &pt2d,
    const Eigen::Matrix<double, 3, 4> &camera_intrinsic,
    const Eigen::Matrix<double, 5, 1> &distort_params) const {
  // add distortion
  double r_sq = pt2d[0] * pt2d[0] + pt2d[1] * pt2d[1];
  Eigen::Matrix<double, 2, 1> pt2d_radial =
      pt2d * (1 + distort_params[0] * r_sq + distort_params[1] * r_sq * r_sq +
              distort_params[4] * r_sq * r_sq * r_sq);
  Eigen::Matrix<double, 2, 1> dpt2d;
  dpt2d[0] = 2 * distort_params[2] * pt2d[0] * pt2d[1] +
             distort_params[3] * (r_sq + 2 * pt2d[0] * pt2d[0]);
  dpt2d[1] = distort_params[2] * (r_sq + 2 * pt2d[1] * pt2d[1]) +
             2 * distort_params[3] * pt2d[0] * pt2d[1];

  Eigen::Matrix<double, 2, 1> pt2d_distort;
  pt2d_distort[0] = pt2d_radial[0] + dpt2d[0];
  pt2d_distort[1] = pt2d_radial[1] + dpt2d[1];

  // add intrinsic K
  double focal_length_x = camera_intrinsic(0, 0);
  double focal_length_y = camera_intrinsic(1, 1);
  double center_x = camera_intrinsic(0, 2);
  double center_y = camera_intrinsic(1, 2);

  Eigen::Matrix<double, 2, 1> pt;
  pt[0] = pt2d_distort[0] * focal_length_x + center_x;
  pt[1] = pt2d_distort[1] * focal_length_y + center_y;

  return pt;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/preprocessor/tl_preprocessor.h"

#include "modules/common/time/time_util.h"
#include "modules/common/util/file.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::time::TimeUtil;
using apollo::common::util::GetProtoFromFile;

bool TLPreprocessor::Init() {
  // Read parameters from config file
  if (!GetProtoFromFile(FLAGS_traffic_light_preprocessor_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_preprocessor_config;
    return false;
  }

  // init projection
  if (!projection_.Init()) {
    AERROR << "TLPreprocessor init projection failed.";
    return false;
  }
  return true;
}

bool TLPreprocessor::CacheLightsProjections(const CarPose &pose,
                                            const std::vector<Signal> &signals,
                                            const double timestamp) {
  MutexLock lock(&mutex_);
  PERF_FUNCTION();

  AINFO << "TLPreprocessor has " << cached_lights_.size()
        << " lights projections cached.";

  // pop front if cached array'size > FLAGS_max_cached_image_lights_array_size
  while (cached_lights_.size() >
         static_cast<size_t>(config_.max_cached_lights_size())) {
    cached_lights_.erase(cached_lights_.begin());
  }

  // lights projection info. to be added in cached array
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  // default select long focus camera
  image_lights->camera_id = SHORT_FOCUS;
  image_lights->timestamp = timestamp;
  image_lights->pose = pose;
  image_lights->is_pose_valid = true;

  AINFO << "TLPreprocessor Got signal number:" << signals.size()
        << ", ts: " << GLOG_TIMESTAMP(timestamp);
  for (const auto &signal : signals) {
    AINFO << "signal info:" << signal.ShortDebugString();
  }
  // lights projections info.

  std::vector<std::shared_ptr<LightPtrs>> lights_on_image(kCountCameraId);
  std::vector<std::shared_ptr<LightPtrs>> lights_outside_image(kCountCameraId);
  for (auto &light_ptrs : lights_on_image) {
    light_ptrs.reset(new LightPtrs);
  }
  for (auto &light_ptrs : lights_outside_image) {
    light_ptrs.reset(new LightPtrs);
  }
  if (signals.size() > 0) {
    // project light region on each camera's image plane
    for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
      if (!ProjectLights(pose, signals, static_cast<CameraId>(cam_id),
                         lights_on_image[cam_id].get(),
                         lights_outside_image[cam_id].get())) {
        AERROR << "add_cached_lights_projections project lights on "
               << kCameraIdToStr.at(static_cast<CameraId>(cam_id))
               << " image failed, "
               << "ts: " << GLOG_TIMESTAMP(timestamp) << ", camera_id: "
               << kCameraIdToStr.at(static_cast<CameraId>(cam_id));
        cached_lights_.push_back(image_lights);
        return false;
      }
    }

    // select which image to be used
    SelectImage(pose, lights_on_image, lights_outside_image,
                &(image_lights->camera_id));
    AINFO << "select camera: " << kCameraIdToStr.at(image_lights->camera_id);

  } else {
    last_no_signals_ts_ = timestamp;
  }
  image_lights->num_signals = signals.size();
  AINFO << "cached info with " << image_lights->num_signals << " signals";
  cached_lights_.push_back(image_lights);

  return true;
}

bool TLPreprocessor::SyncImage(ImageSharedPtr image,
                               ImageLightsPtr *image_lights, bool *should_pub) {
  MutexLock lock(&mutex_);
  PERF_FUNCTION();
  CameraId camera_id = image->camera_id();
  double image_ts = image->ts();
  bool sync_ok = false;

  PERF_FUNCTION();
  if (cached_lights_.size() == 0) {
    AINFO << "No cached light";
    return false;
  }
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "SyncImage failed, "
           << "get unknown CameraId: " << camera_id;
    return false;
  }

  // find close enough(by timestamp difference)
  // lights projection from back to front

  bool find_loc = false;  // if pose is found
  auto cached_lights_ptr = cached_lights_.rbegin();
  for (; cached_lights_ptr != cached_lights_.rend(); ++cached_lights_ptr) {
    double light_ts = (*cached_lights_ptr)->timestamp;
    if (fabs(light_ts - image_ts) < config_.sync_interval_seconds()) {
      find_loc = true;
      auto proj_cam_id = static_cast<int>((*cached_lights_ptr)->camera_id);
      auto image_cam_id = static_cast<int>(camera_id);
      auto proj_cam_id_str =
          (kCameraIdToStr.find(proj_cam_id) != kCameraIdToStr.end()
               ? kCameraIdToStr.at(proj_cam_id)
               : std::to_string(proj_cam_id));
      // found related pose but if camear ID doesn't match
      if (proj_cam_id != image_cam_id) {
        AWARN << "find appropriate localization, but camera_id not match"
              << ", cached projection's camera_id: " << proj_cam_id_str
              << " , image's camera_id: " << kCameraIdToStr.at(image_cam_id);
        continue;
      }
      if (image_ts < last_output_ts_) {
        AWARN << "TLPreprocessor reject the image pub ts:"
              << GLOG_TIMESTAMP(image_ts)
              << " which is earlier than last output ts:"
              << GLOG_TIMESTAMP(last_output_ts_)
              << ", image camera_id: " << kCameraIdToStr.at(image_cam_id);
        return false;
      }
      sync_ok = true;
      break;
    }
  }

  if (sync_ok && cached_lights_ptr != cached_lights_.rend()) {
    *image_lights = *cached_lights_ptr;
    (*image_lights)->diff_image_pose_ts =
        image_ts - (*cached_lights_ptr)->timestamp;
    (*image_lights)->diff_image_sys_ts = image_ts - TimeUtil::GetCurrentTime();

    (*image_lights)->image = image;
    (*image_lights)->timestamp = image_ts;
    AINFO << "TLPreprocessor sync ok ts: " << GLOG_TIMESTAMP(image_ts)
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    last_output_ts_ = image_ts;
    last_pub_camera_id_ = camera_id;
    *should_pub = true;
  } else {
    AINFO << "sync image with cached lights projection failed, "
          << "no valid pose, ts: " << GLOG_TIMESTAMP(image_ts)
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    std::string cached_array_str = "cached lights";
    if (fabs(image_ts - last_no_signals_ts_) <
        config_.no_signals_interval_seconds()) {
      AINFO << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << " last_no_signals_ts: " << GLOG_TIMESTAMP(last_no_signals_ts_)
            << " (sync_time - last_no_signals_ts): "
            << GLOG_TIMESTAMP(image_ts - last_no_signals_ts_)
            << " query /tf in low frequence because no signals forward "
            << " camera_id: " << kCameraIdToStr.at(camera_id);
    } else if (image_ts < cached_lights_.front()->timestamp) {
      double pose_ts = cached_lights_.front()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << ", which is earlier than " << cached_array_str
            << ".front() ts: " << GLOG_TIMESTAMP(pose_ts)
            << ", diff between image and pose ts: "
            << GLOG_TIMESTAMP(image_ts - pose_ts)
            << "; system ts: " << GLOG_TIMESTAMP(system_ts)
            << ", diff between image and system ts: "
            << GLOG_TIMESTAMP(image_ts - system_ts)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
    } else if (image_ts > cached_lights_.back()->timestamp) {
      double pose_ts = cached_lights_.back()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << ", which is older than " << cached_array_str
            << ".back() ts: " << GLOG_TIMESTAMP(pose_ts)
            << ", diff between image and pose ts: "
            << GLOG_TIMESTAMP(image_ts - pose_ts)
            << "; system ts: " << GLOG_TIMESTAMP(system_ts)
            << ", diff between image and system ts: "
            << GLOG_TIMESTAMP(image_ts - system_ts)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
    } else if (!find_loc) {
      // if no pose found, log warning msg
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << ", cannot find close enough timestamp, " << cached_array_str
            << ".front() ts: "
            << GLOG_TIMESTAMP(cached_lights_.front()->timestamp) << ", "
            << cached_array_str << ".back() ts: "
            << GLOG_TIMESTAMP(cached_lights_.back()->timestamp)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
    }
  }
  // sync fail may because:
  // 1. image is not selected
  // 2. timestamp drift
  // 3. [there is no tf]
  return sync_ok;
}

void TLPreprocessor::set_last_pub_camera_id(CameraId camera_id) {
  last_pub_camera_id_ = camera_id;
}

CameraId TLPreprocessor::last_pub_camera_id() const {
  return last_pub_camera_id_;
}

int TLPreprocessor::max_cached_lights_size() const {
  return config_.max_cached_lights_size();
}

void TLPreprocessor::SelectImage(const CarPose &pose,
                                 const LightsArray &lights_on_image_array,
                                 const LightsArray &lights_outside_image_array,
                                 CameraId *selection) {
  *selection = static_cast<CameraId>(kShortFocusIdx);

  // check from long focus to short focus
  for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
    if (!lights_outside_image_array[cam_id]->empty()) {
      continue;
    }
    bool ok = true;
    // find the short focus camera without range check
    if (cam_id != kShortFocusIdx) {
      for (LightPtr light : *(lights_on_image_array[cam_id])) {
        if (IsOnBorder(cv::Size(config_.projection_image_cols(),
                                config_.projection_image_rows()),
                       light->region.projection_roi,
                       image_border_size[cam_id])) {
          ok = false;
          AINFO << "light project on image border region, "
                << "CameraId: " << kCameraIdToStr.at(cam_id);
          break;
        }
      }
    }
    if (ok) {
      *selection = static_cast<CameraId>(cam_id);
      break;
    }
  }

  AINFO << "select_image selection: " << *selection;
}

bool TLPreprocessor::ProjectLights(const CarPose &pose,
                                   const std::vector<Signal> &signals,
                                   const CameraId &camera_id,
                                   LightPtrs *lights_on_image,
                                   LightPtrs *lights_outside_image) {
  if (signals.empty()) {
    ADEBUG << "project_lights get empty signals.";
    return true;
  }

  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "project_lights get invalid CameraId: " << camera_id;
    return false;
  }

  for (size_t i = 0; i < signals.size(); ++i) {
    LightPtr light(new Light);
    light->info = signals[i];
    if (!projection_.Project(pose, ProjectOption(camera_id), light.get())) {
      lights_outside_image->push_back(light);
    } else {
      lights_on_image->push_back(light);
    }
  }

  return true;
}

bool TLPreprocessor::IsOnBorder(const cv::Size size, const cv::Rect &roi,
                                const int border_size) const {
  if (roi.x < border_size || roi.y < border_size) {
    return true;
  }
  if (roi.x + roi.width + border_size >= size.width ||
      roi.y + roi.height + border_size >= size.height) {
    return true;
  }
  return false;
}

int TLPreprocessor::GetMinFocalLenCameraId() { return kShortFocusIdx; }

int TLPreprocessor::GetMaxFocalLenCameraId() { return kLongFocusIdx; }

REGISTER_PREPROCESSOR(TLPreprocessor);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/perception/traffic_light/base/image_lights.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using apollo::perception::TrafficLightDetection;
using apollo::perception::traffic_light::CameraId;
using apollo::perception::traffic_light::Image;

std::unordered_map<std::string, cv::Scalar> kColorTable = {
    {std::string("red_light_box"), cv::Scalar(0, 0, 255)},
    {std::string("green_light_box"), cv::Scalar(0, 255, 0)},
    {std::string("yellow_light_box"), cv::Scalar(0, 255, 255)},
    {std::string("black_light_box"), cv::Scalar(255, 90, 199)},
    {std::string("unknown_light_box"), cv::Scalar(0, 76, 153)},
    {std::string("projection_roi"), cv::Scalar(255, 255, 0)},
    {std::string("crop_roi"), cv::Scalar(0, 255, 255)},
    {std::string("debug_roi"), cv::Scalar(255, 169, 255)}};

std::vector<std::shared_ptr<Image>> cached_images;
const int kMaxCachedImageNum = 100;

void SubDebugCallback(const TrafficLightDetection &);
void SubLongFocusCallback(sensor_msgs::ImagePtr);
void SubShortFocusCallback(sensor_msgs::ImagePtr);

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "traffic_light_viz_listener");
  ros::NodeHandle n;

  ros::Subscriber sub_tl_debug =
      n.subscribe(FLAGS_traffic_light_detection_topic, 1000, SubDebugCallback);
  ros::Subscriber sub_tl_image_long =
      n.subscribe(FLAGS_image_long_topic, 1000, SubLongFocusCallback);
  ros::Subscriber sub_tl_image_short =
      n.subscribe(FLAGS_image_short_topic, 1000, SubShortFocusCallback);

  ros::spin();
  return 0;
}

void SubDebugCallback(const TrafficLightDetection &tl_result) {
  auto img_ts = tl_result.header().camera_timestamp() / 1e9;
  auto tl_debug_msg = tl_result.traffic_light_debug();
  auto signals_num = tl_debug_msg.signal_num();
  auto box_size = tl_debug_msg.box_size();
  auto camera_id = tl_debug_msg.camera_id();
  cv::Mat img;
  bool found_image = false;
  for (int i = cached_images.size() - 1; i >= 0; --i) {
    if (fabs(cached_images[i]->ts() - img_ts) < 0.005 &&
        camera_id == cached_images[i]->camera_id()) {
      cached_images[i]->GenerateMat();
      img = cached_images[i]->mat();
      found_image = true;
      break;
    }
  }
  if (!found_image) {
    return;
  }

  if (signals_num > 0 && tl_debug_msg.has_cropbox()) {
    // crop roi
    auto crop_box = tl_debug_msg.cropbox();
    cv::Rect cv_crop_rect(crop_box.x(), crop_box.y(), crop_box.width(),
                          crop_box.height());
    cv::rectangle(img, cv_crop_rect, kColorTable["crop_roi"], 2);

    // debug roi
    for (int box_idx = signals_num * 2; box_idx < box_size; ++box_idx) {
      auto debug_roi_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_debug_roi_box(debug_roi_box.x(), debug_roi_box.y(),
                                debug_roi_box.width(), debug_roi_box.height());
      cv::rectangle(img, cv_debug_roi_box, kColorTable["debug_roi"], 2);
    }

    // projection roi
    for (int box_idx = signals_num; box_idx < signals_num * 2; ++box_idx) {
      auto projection_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_projection_box(projection_box.x(), projection_box.y(),
                                 projection_box.width(),
                                 projection_box.height());
      cv::rectangle(img, cv_projection_box, kColorTable["projection_roi"], 2);
    }

    // rectified roi
    for (int box_idx = 0; box_idx < signals_num; ++box_idx) {
      auto rectified_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_rectified_box(rectified_box.x(), rectified_box.y(),
                                rectified_box.width(), rectified_box.height());
      cv::Scalar color;

      using apollo::perception::TrafficLight;
      switch (rectified_box.color()) {
        case TrafficLight::RED:
          color = kColorTable["red_light_box"];
          break;
        case TrafficLight::GREEN:
          color = kColorTable["green_light_box"];
          break;
        case TrafficLight::BLACK:
          color = kColorTable["black_light_box"];
          break;
        case TrafficLight::YELLOW:
          color = kColorTable["yellow_light_box"];
          break;
        default:
          color = kColorTable["unknown_light_box"];
          break;
      }

      cv::rectangle(img, cv_rectified_box, color, 2);
    }
  }

  // draw camera timestamp
  int pos_y = 40;
  std::string ts_text = cv::format("img ts=%lf", img_ts);
  cv::putText(img, ts_text, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN, 3.0,
              CV_RGB(128, 255, 0), 2);
  // draw distance to stopline
  pos_y += 50;
  double distance = tl_debug_msg.distance_to_stop_line();
  if (signals_num > 0) {
    std::string dis2sl_text = cv::format("dis2sl=%lf", distance);
    cv::putText(img, dis2sl_text, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(128, 255, 0), 2);
  }

  // draw "Signals Num"
  pos_y += 50;
  if (tl_debug_msg.valid_pos()) {
    std::string signal_txt = "Signals Num: " + std::to_string(signals_num);
    cv::putText(img, signal_txt, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(255, 0, 0), 2);
  }

  // draw "No Pose info."
  pos_y += 50;
  if (!tl_debug_msg.valid_pos()) {
    cv::putText(img, "No Valid Pose.", cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);

    // if image's timestamp is too early or too old
    // draw timestamp difference between image and pose
    pos_y += 50;
    std::string diff_img_pose_ts_str =
        "ts diff: " + std::to_string(tl_debug_msg.ts_diff_pos());
    cv::putText(img, diff_img_pose_ts_str, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);

    pos_y += 50;
    std::string diff_img_sys_ts_str =
        "ts diff sys: " + std::to_string(tl_debug_msg.ts_diff_sys());
    cv::putText(img, diff_img_sys_ts_str, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);
  }
  pos_y += 50;
  {
    std::string signal_txt =
        "camera id: " +
        apollo::perception::traffic_light::kCameraIdToStr.at(
            tl_debug_msg.camera_id());
    cv::putText(img, signal_txt, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(255, 0, 0), 2);
  }
  // draw image border size (offset between hdmap-box and detection-box)
  if (tl_debug_msg.project_error() > 100) {
    std::string img_border_txt =
        "Offset size: " + std::to_string(tl_debug_msg.project_error());
    constexpr int kPosYOffset = 1000;
    cv::putText(img, img_border_txt, cv::Point(30, kPosYOffset),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);
  }

  cv::resize(img, img, cv::Size(960, 540));
  cv::imshow("tl_debug_image", img);
  cv::waitKey(10);
}
void SubImage(CameraId camera_id, sensor_msgs::ImagePtr msg) {
  boost::shared_ptr<sensor_msgs::Image> img(new sensor_msgs::Image);
  *img = *msg;
  boost::shared_ptr<const sensor_msgs::Image> img_msg(img);
  std::shared_ptr<Image> image(new Image);
  if (!image->Init(img_msg->header.stamp.toSec(), camera_id, img_msg)) {
    std::cerr << "tl_visualizer load image failed.";
  }
  cached_images.push_back(image);

  while (cached_images.size() > kMaxCachedImageNum) {
    cached_images.erase(cached_images.begin());
  }
}
void SubLongFocusCallback(sensor_msgs::ImagePtr msg) {
  SubImage(CameraId::LONG_FOCUS, msg);
}

void SubShortFocusCallback(sensor_msgs::ImagePtr msg) {
  SubImage(CameraId::SHORT_FOCUS, msg);
}
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/rectify/cropbox.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

void CropBox::GetCropBox(const cv::Size &size,
                         const std::vector<LightPtr> &lights,
                         cv::Rect *cropbox) {
  int lights_num = lights.size();
  if (lights_num == 0) {
    AINFO << "No valid HD-map corrdinates info";
    ClearBox(cropbox);
    return;
  }
  int rows = size.height;
  int cols = size.width;
  float xr = 0.0;
  float yb = 0.0;
  float xl = cols - 1.0;
  float yt = rows - 1.0;
  bool initialized = false;
  // min max: for all hdmap boxes
  for (int i = 0; i < lights_num; ++i) {
    Light &light = *lights[i];
    if (!BoxIsValid(light.region.projection_roi, size)) {
      continue;
    }
    initialized = true;
    if (xl > light.region.projection_roi.x) {
      xl = light.region.projection_roi.x;
    }
    if (yt > light.region.projection_roi.y) {
      yt = light.region.projection_roi.y;
    }
    if (xr <
        light.region.projection_roi.x + light.region.projection_roi.width) {
      xr = light.region.projection_roi.x + light.region.projection_roi.width;
    }
    if (yb <
        light.region.projection_roi.y + light.region.projection_roi.height) {
      yb = light.region.projection_roi.y + light.region.projection_roi.height;
    }
  }
  if (!initialized) {
    ClearBox(cropbox);
    return;
  }
  // scale
  float center_x = (xr + xl) / 2;
  float center_y = (yb + yt) / 2;
  float resize_width = (xr - xl) * crop_scale_;
  float resize_height = (yb - yt) * crop_scale_;
  float resize = std::max(resize_width, resize_height);
  resize_width = resize_height =
      (resize < min_crop_size_) ? min_crop_size_ : resize;

  // float pad_t = (resize_height - (yb - yt)) / 2;
  // float pad_l = (resize_width - (xr - xl)) / 2;
  // float pad_b = pad_t;
  // float pad_r = pad_l;

  // clamp
  xl = center_x - resize_width / 2;
  xl = (xl < 0) ? 0 : xl;
  yt = center_y - resize_height / 2;
  yt = (yt < 0) ? 0 : yt;
  xr = center_x + resize_width / 2;
  xr = (xr >= cols) ? cols - 1 : xr;
  yb = center_y + resize_width / 2;
  yb = (yb >= rows) ? rows - 1 : yb;

  cropbox->x = static_cast<int>(xl);
  cropbox->y = static_cast<int>(yt);
  cropbox->width = static_cast<int>(xr - xl);
  cropbox->height = static_cast<int>(yb - yt);
}

void CropBox::Init(float crop_scale, float min_crop_size) {
  crop_scale_ = crop_scale;
  min_crop_size_ = min_crop_size;
}

CropBox::CropBox(float crop_scale, float min_crop_size) {
  Init(crop_scale, min_crop_size);
}

void CropBoxWholeImage::GetCropBox(const cv::Size &size,
                                   const std::vector<LightPtr> &lights,
                                   cv::Rect *cropbox) {
  for (size_t i = 0; i < lights.size(); ++i) {
    if (BoxIsValid(lights[i]->region.projection_roi, size)) {
      cropbox->x = cropbox->y = 0;
      cropbox->width = size.width;
      cropbox->height = size.height;
      return;
    }
  }
  ClearBox(cropbox);
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/rectify/detection.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::time::Timer;

void Detection::Perform(const cv::Mat &ros_image,
                        std::vector<LightPtr> *lights) {
  cv::Mat crop_image = ros_image(crop_box_);
  Timer timer;
  timer.Start();
  lights->clear();
  // resize
  cv::Mat fw_image;
  float col_shrink = static_cast<float>(resize_len_) / (crop_image.cols);
  float row_shrink = static_cast<float>(resize_len_) / (crop_image.rows);
  float crop_col_shrink_ = std::max(col_shrink, row_shrink);
  float crop_row_shrink_ = crop_col_shrink_;
  cv::resize(crop_image, fw_image,
             cv::Size(crop_image.cols * crop_col_shrink_,
                      crop_image.rows * crop_row_shrink_));
  AINFO << "resize fw image Done at " << fw_image.size();
  // detection_
  refine_input_layer_->FetchOutterImageFrame(fw_image);
  AINFO << "FetchOutterImage Done ";
  refine_net_ptr_->ForwardFrom(0);
  int forward_time_for_this_sample =
      refine_input_layer_->GetForwardTimesForCurSample();

  for (int iter = 1; iter < forward_time_for_this_sample; ++iter) {
    refine_net_ptr_->ForwardFrom(0);
  }
  AINFO << "net forward Done!";
  // dump the output

  float inflate_col = 1.0f / crop_col_shrink_;
  float inflate_row = 1.0f / crop_row_shrink_;
  SelectOutputBboxes(crop_image.size(), VERTICAL_CLASS, inflate_col,
                     inflate_row, lights);
  SelectOutputBboxes(crop_image.size(), QUADRATE_CLASS, inflate_col,
                     inflate_row, lights);

  AINFO << "Dump output Done! Get box num:" << lights->size();

  uint64_t elapsed_time = timer.End("Running detection_: ");
  AINFO << "Running detection_: " << elapsed_time << " ms";
}

void Detection::Init(const int resize_len, const std::string &refine_net,
                     const std::string &refine_model) {
  refine_net_ptr_.reset(new caffe::Net<float>(refine_net, caffe::TEST));
  refine_net_ptr_->CopyTrainedLayersFrom(refine_model);
  refine_input_layer_ =
      static_cast<caffe::PyramidImageOnlineDataLayer<float> *>(
          refine_net_ptr_->layers()[0].get());
  refine_output_layer_ = static_cast<caffe::ROIOutputSSDLayer<float> *>(
      refine_net_ptr_->layers()[refine_net_ptr_->layers().size() - 1].get());
  AINFO << refine_input_layer_->resize_scale << " "
        << refine_input_layer_->type();

  resize_len_ = resize_len;
}

Detection::Detection(int min_crop_size, const std::string &refine_net,
                     const std::string &refine_model) {
  Init(min_crop_size, refine_net, refine_model);
}

bool Detection::SelectOutputBboxes(const cv::Size &img_size, int class_id,
                                   float inflate_col, float inflate_row,
                                   std::vector<LightPtr> *lights) {
  if (class_id < 0 || class_id >= 2) {
    AERROR << "DenseBoxDetection invalid class_id, "
           << "select_output_bboxes failed.";
    return false;
  }

  vector<caffe::BBox<float>> &result_bbox =
      refine_output_layer_->GetFilteredBBox(class_id);
  for (size_t candidate_id = 0; candidate_id < result_bbox.size();
       candidate_id++) {
    LightPtr tmp(new Light);
    tmp->region.rectified_roi.x =
        static_cast<int>(result_bbox[candidate_id].x1 * inflate_col);
    tmp->region.rectified_roi.y =
        static_cast<int>(result_bbox[candidate_id].y1 * inflate_row);
    tmp->region.rectified_roi.width = static_cast<int>(
        (result_bbox[candidate_id].x2 - result_bbox[candidate_id].x1 + 1) *
        inflate_col);
    tmp->region.rectified_roi.height = static_cast<int>(
        (result_bbox[candidate_id].y2 - result_bbox[candidate_id].y1 + 1) *
        inflate_row);
    tmp->region.detect_score = result_bbox[candidate_id].score;

    if (!BoxIsValid(tmp->region.rectified_roi, img_size)) {
      AINFO << "Invalid width or height or x or y: "
            << tmp->region.rectified_roi.width << " | "
            << tmp->region.rectified_roi.height << " | "
            << tmp->region.rectified_roi.x << " | "
            << tmp->region.rectified_roi.y;
      continue;
    }

    tmp->region.rectified_roi = RefinedBox(tmp->region.rectified_roi, img_size);
    tmp->region.is_detected = true;
    tmp->region.detect_class_id = DetectionClassId(class_id);
    lights->push_back(tmp);
  }

  return true;
}

void Detection::SetCropBox(const cv::Rect &box) { crop_box_ = box; }

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/rectify/select.h"

#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

void GaussianSelect::Select(const cv::Mat &ros_image,
                            const std::vector<LightPtr> &hdmap_bboxes,
                            const std::vector<LightPtr> &refined_bboxes,
                            std::vector<LightPtr> *selected_bboxes) {
  // find bbox with max area in refined_bboxes
  auto max_area_refined_bbox =
      std::max_element(refined_bboxes.begin(), refined_bboxes.end(),
                       [](const LightPtr lhs, const LightPtr rhs) {
                         return lhs->region.rectified_roi.area() <
                                rhs->region.rectified_roi.area();
                       });

  //  cv::Mat_<int> cost_matrix(hdmap_bboxes.size(), refined_bboxes.size());
  std::vector<std::vector<double>> score_matrix(
      hdmap_bboxes.size(), std::vector<double>(refined_bboxes.size(), 0));
  for (size_t row = 0; row < hdmap_bboxes.size(); ++row) {
    cv::Point2f center_hd = GetCenter(hdmap_bboxes[row]->region.rectified_roi);
    auto width_hd = hdmap_bboxes[row]->region.rectified_roi.width;
    for (size_t col = 0; col < refined_bboxes.size(); ++col) {
      cv::Point2f center_refine =
          GetCenter(refined_bboxes[col]->region.rectified_roi);
      auto width_refine = refined_bboxes[col]->region.rectified_roi.width;

      // use gaussian score as metrics of distance and width
      // distance_score:
      //    larger distance => 0.
      //    smaller distance => 1
      double distance_score = static_cast<double>(
          Get2dGaussianScore(center_hd, center_refine, 100, 100));
      // width score:
      //   larger width diff => 0
      //   smaller width diff => 1
      double width_score =
          static_cast<double>(Get1dGaussianScore(width_hd, width_refine, 100));

      // normalized area score
      // larger area => 1
      double area_score = 1.0 *
                          refined_bboxes[col]->region.rectified_roi.area() /
                          (*max_area_refined_bbox)->region.rectified_roi.area();

      // when numerator=1， denominator is very small,
      // converting to int might reduce to same value, here uses 1000
      // + 0.05 to prevent score = 0
      // score * weight h
      score_matrix[row][col] =
          (0.05 + 0.4 * refined_bboxes[col]->region.detect_score +
           0.2 * distance_score + 0.2 * width_score + 0.2 * area_score);
    }
  }

  HungarianOptimizer munkres(score_matrix);
  std::vector<int> hd_index;
  std::vector<int> refined_index;
  munkres.maximize(&hd_index, &refined_index);
  for (size_t i = 0; i < hdmap_bboxes.size(); ++i) {
    hdmap_bboxes[i]->region.is_selected = false;
  }
  for (size_t i = 0; i < hd_index.size(); ++i) {
    if (hd_index[i] < 0 ||
        static_cast<size_t>(hd_index[i]) >= hdmap_bboxes.size() ||
        refined_index[i] < 0 ||
        static_cast<size_t>(refined_index[i]) >= refined_bboxes.size() ||
        hdmap_bboxes[hd_index[i]]->region.is_selected) {
      continue;
    }
    if (score_matrix[hd_index[i]][refined_index[i]] > 0) {
      refined_bboxes[refined_index[i]]->region.is_selected = true;
      hdmap_bboxes[hd_index[i]]->region.is_selected = true;
      selected_bboxes->push_back(refined_bboxes[refined_index[i]]);
    }
  }
  for (size_t i = 0; i < hdmap_bboxes.size(); ++i) {
    if (!hdmap_bboxes[i]->region.is_selected) {
      selected_bboxes->push_back(hdmap_bboxes[i]);
    }
  }
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/rectify/select.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class MatchTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  ~MatchTest() {}

 protected:
  GaussianSelect select;
};

TEST_F(MatchTest, nvn1) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(120, 140, 10, 30);
    detect_bboxes.push_back(light);
  }
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
}

TEST_F(MatchTest, nvn2) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    light->region.detect_score = 0.9;
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(160, 100, 10, 30);
    light->region.detect_score = 0.9;
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(220, 240, 10, 30);
    light->region.detect_score = 0.9;
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(240, 240, 10, 30);
    light->region.detect_score = 0.9;
    detect_bboxes.push_back(light);
  }

  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);
  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi,
            detect_bboxes[1]->region.rectified_roi);
}

TEST_F(MatchTest, nvm12) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(220, 240, 10, 30);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(170, 240, 10, 30);
    detect_bboxes.push_back(light);
  }

  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 1);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  std::cout << selected_bboxes[0]->region.rectified_roi << std::endl;
  std::cout << detect_bboxes[1]->region.rectified_roi << std::endl;
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[1]->region.rectified_roi);

  hdmap_bboxes[0]->region.rectified_roi.x = 220;
  selected_bboxes.clear();
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 1);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
}

TEST_F(MatchTest, nvm21) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(160, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(120, 140, 10, 30);
    detect_bboxes.push_back(light);
  }
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_FALSE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi,
            hdmap_bboxes[1]->region.rectified_roi);
}

TEST_F(MatchTest, nvm24) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(160, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }

  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(120, 140, 10, 40);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(160, 140, 10, 40);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(130, 150, 20, 20);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(170, 150, 20, 20);
    detect_bboxes.push_back(light);
  }
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi,
            detect_bboxes[1]->region.rectified_roi);
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/rectify/unity_rectify.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/rectify/cropbox.h"
#include "modules/perception/traffic_light/rectify/detection.h"
#include "modules/perception/traffic_light/rectify/select.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;

bool UnityRectify::Init() {
  if (!GetProtoFromFile(FLAGS_traffic_light_rectifier_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_rectifier_config;
    return false;
  }

  switch (config_.crop_method()) {
    default:
    case 0:
      crop_ = std::make_shared<CropBox>(config_.crop_scale(),
                                        config_.crop_min_size());
      break;
    case 1:
      crop_ = std::make_shared<CropBoxWholeImage>();
      break;
  }
  switch (config_.detect_method()) {
    default:
    case 0:
      detect_ = std::make_shared<Detection>(config_.crop_min_size(),
                                            config_.detection_net(),
                                            config_.detection_model());
      break;
    case 1:
      detect_ = std::make_shared<DummyRefine>();
      break;
  }

  select_ = std::make_shared<GaussianSelect>();

  return true;
}

bool UnityRectify::Rectify(const Image &image, const RectifyOption &option,
                           std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  std::vector<LightPtr> &lights_ref = *lights;
  std::vector<LightPtr> selected_bboxes;
  std::vector<LightPtr> detected_bboxes;

  for (auto &light : lights_ref) {
    // By default, the first debug ros is crop roi. (Reserve a position here).
    light->region.rectified_roi = light->region.projection_roi;
    light->region.debug_roi.push_back(cv::Rect(0, 0, 0, 0));
    light->region.debug_roi_detect_scores.push_back(0.0f);
  }

  cv::Rect cbox;
  crop_->GetCropBox(ros_image.size(), lights_ref, &cbox);
  AINFO << ros_image.size();
  AINFO << cbox;
  if (BoxIsValid(cbox, ros_image.size())) {
    lights_ref[0]->region.debug_roi[0] = cbox;

    detect_->SetCropBox(cbox);
    detect_->Perform(ros_image, &detected_bboxes);

    AINFO << "detect " << detected_bboxes.size() << " lights";
    for (size_t j = 0; j < detected_bboxes.size(); ++j) {
      AINFO << detected_bboxes[j]->region.rectified_roi;
      cv::Rect &region = detected_bboxes[j]->region.rectified_roi;
      float score = detected_bboxes[j]->region.detect_score;
      region.x += cbox.x;
      region.y += cbox.y;
      lights_ref[0]->region.debug_roi.push_back(region);
      lights_ref[0]->region.debug_roi_detect_scores.push_back(score);
    }

    select_->Select(ros_image, lights_ref, detected_bboxes, &selected_bboxes);
  } else {
    for (size_t h = 0; h < lights_ref.size(); ++h) {
      LightPtr light = lights_ref[h];
      light->region.is_detected = false;
      selected_bboxes.push_back(light);
    }
  }

  for (size_t i = 0; i < lights_ref.size(); ++i) {
    if (!selected_bboxes[i]->region.is_detected ||
        !selected_bboxes[i]->region.is_selected) {
      AWARN << "No detection box ,using project box";
    }
    cv::Rect region = selected_bboxes[i]->region.rectified_roi;
    lights_ref[i]->region.rectified_roi = region;
    lights_ref[i]->region.detect_class_id =
        selected_bboxes[i]->region.detect_class_id;
    lights_ref[i]->region.detect_score =
        selected_bboxes[i]->region.detect_score;
    lights_ref[i]->region.is_detected = selected_bboxes[i]->region.is_detected;
    lights_ref[i]->region.is_selected = selected_bboxes[i]->region.is_selected;
    AINFO << region;
  }
  return true;
}

std::string UnityRectify::name() const { return "UnityRectify"; }

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/base/image.h"

#include "cv_bridge/cv_bridge.h"

#include "modules/common/log.h"
#include "modules/perception/traffic_light/util/color_space.h"

DEFINE_int32(double_show_precision, 14,
             "When output a double data, the precision.");

namespace apollo {
namespace perception {
namespace traffic_light {

bool Image::Init(const double ts, const CameraId &device_id,
                 const cv::Mat &mat) {
  contain_mat_ = true;
  contain_image_ = true;
  timestamp_ = ts, camera_id_ = device_id, mat_ = mat.clone();
  ADEBUG << *this << " init.";
  return true;
}
bool Image::Init(const double ts, const CameraId &device_id,
                 boost::shared_ptr<const sensor_msgs::Image> image_data) {
  contain_mat_ = false;
  contain_image_ = true;
  timestamp_ = ts, camera_id_ = device_id, image_data_ = image_data;
  ADEBUG << *this << " init.";
  return true;
}

double Image::ts() const { return timestamp_; }

CameraId Image::camera_id() const { return camera_id_; }

std::string Image::camera_id_str() const {
  if (kCameraIdToStr.find(camera_id_) == kCameraIdToStr.end()) {
    return "unkown camera";
  }
  return kCameraIdToStr.at(camera_id_);
}
bool Image::GenerateMat() {
  if (!contain_mat_) {
    try {
      if (image_data_->encoding.compare("yuyv") == 0) {
        unsigned char *yuv = (unsigned char *)&(image_data_->data[0]);
        mat_ = cv::Mat(image_data_->height, image_data_->width, CV_8UC3);
        Yuyv2rgb(yuv, mat_.data, image_data_->height * image_data_->width);
        cv::cvtColor(mat_, mat_, CV_RGB2BGR);
      }

      contain_mat_ = true;
      AINFO << "Generate done " << mat_.size();
    } catch (const cv_bridge::Exception &e) {
      AERROR << "TLPreprocessorSubnode trans msg to cv::Mat failed."
             << e.what();
      return false;
    }
  }
  return true;
}
const cv::Mat &Image::mat() const { return mat_; }

cv::Size Image::size() const {
  if (contain_mat_) {
    return mat_.size();
  } else {
    return cv::Size(image_data_->width, image_data_->height);
  }
}

std::ostream &operator<<(std::ostream &os, const Image &image) {
  if (image.contain_mat_) {
    os << "Image device_id:" << static_cast<int>(image.camera_id_)
       << " device_id_str: " << image.camera_id_str()
       << " ts:" << std::setprecision(FLAGS_double_show_precision)
       << image.timestamp_ << " size:" << image.mat_.size();
  } else {
    os << "Image not inited.";
  }
  return os;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h"

#include "image_transport/image_transport.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/projection/projection.h"
#include "modules/perception/traffic_light/recognizer/unity_recognize.h"
#include "modules/perception/traffic_light/rectify/unity_rectify.h"
#include "modules/perception/traffic_light/reviser/color_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;
using common::adapter::AdapterManager;

bool TLPreprocessorSubnode::InitInternal() {
  RegisterFactoryBoundaryProjection();
  if (!InitSharedData()) {
    AERROR << "TLPreprocessorSubnode init failed. Shared Data init failed.";
    return false;
  }

  if (!GetProtoFromFile(FLAGS_traffic_light_subnode_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_subnode_config;
    return false;
  }

  // init preprocessor
  if (!InitPreprocessor()) {
    AERROR << "TLPreprocessorSubnode init failed.";
    return false;
  }

  // init hd_map
  if (!InitHdmap()) {
    AERROR << "TLPreprocessorSubnode Failed to init hdmap";
    return false;
  }

  CHECK(AdapterManager::GetImageLong())
      << "TLPreprocessorSubnode init failed.ImageLong is not initialized.";
  AdapterManager::AddImageLongCallback(
      &TLPreprocessorSubnode::SubLongFocusCamera, this);
  CHECK(AdapterManager::GetImageShort())
      << "TLPreprocessorSubnode init failed.ImageShort is not initialized.";
  AdapterManager::AddImageShortCallback(
      &TLPreprocessorSubnode::SubShortFocusCamera, this);
  return true;
}

bool TLPreprocessorSubnode::InitSharedData() {
  CHECK_NOTNULL(shared_data_manager_);

  const std::string preprocessing_data_name("TLPreprocessingData");
  preprocessing_data_ = dynamic_cast<TLPreprocessingData *>(
      shared_data_manager_->GetSharedData(preprocessing_data_name));
  if (preprocessing_data_ == nullptr) {
    AERROR << "TLPreprocessorSubnode failed to get shared data instance "
           << preprocessing_data_name;
    return false;
  }
  AINFO << "TLPreprocessorSubnode init shared data. name:"
        << preprocessing_data_->name();
  return true;
}

bool TLPreprocessorSubnode::InitPreprocessor() {
  if (!preprocessor_.Init()) {
    AERROR << "TLPreprocessorSubnode init preprocessor failed";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::InitHdmap() {
  hd_map_ = HDMapInput::instance();
  if (hd_map_ == nullptr) {
    AERROR << "TLPreprocessorSubnode get hdmap failed.";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::AddDataAndPublishEvent(
    const std::shared_ptr<ImageLights> &data, const CameraId &camera_id,
    double timestamp) {
  // add data down-stream
  std::string device_str = kCameraIdToStr.at(camera_id);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_str, &key)) {
    AERROR << "TLPreprocessorSubnode gen share data key failed. ts:"
           << GLOG_TIMESTAMP(timestamp);
    return false;
  }

  if (!preprocessing_data_->Add(key, data)) {
    AERROR << "TLPreprocessorSubnode push data into shared_data failed.";
    data->image.reset();
    return false;
  }

  // pub events
  for (size_t i = 0; i < this->pub_meta_events_.size(); ++i) {
    const EventMeta &event_meta = this->pub_meta_events_[i];
    Event event;
    event.event_id = event_meta.event_id;
    event.reserve = device_str;
    event.timestamp = timestamp;
    this->event_manager_->Publish(event);
  }
  return true;
}

void TLPreprocessorSubnode::SubLongFocusCamera(const sensor_msgs::Image &msg) {
  AdapterManager::Observe();
  SubCameraImage(AdapterManager::GetImageLong()->GetLatestObservedPtr(),
                 LONG_FOCUS);
  PERF_FUNCTION("SubLongFocusCamera");
}

void TLPreprocessorSubnode::SubShortFocusCamera(const sensor_msgs::Image &msg) {
  AdapterManager::Observe();
  SubCameraImage(AdapterManager::GetImageShort()->GetLatestObservedPtr(),
                 SHORT_FOCUS);
  PERF_FUNCTION("SubShortFocusCamera");
}

void TLPreprocessorSubnode::SubCameraImage(
    boost::shared_ptr<const sensor_msgs::Image> msg, CameraId camera_id) {
  // Only one image could be used in a while.
  // Ohters will be discarded
  // The pipeline turn to a single thread
  MutexLock lock(&mutex_);
  const double sub_camera_image_start_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<Image> image(new Image);
  cv::Mat cv_mat;
  double timestamp = msg->header.stamp.toSec();
  image->Init(timestamp, camera_id, msg);
  if (FLAGS_output_raw_img) {
    // user should create folders
    image->GenerateMat();
    char filename[100];
    snprintf(filename, sizeof(filename), "%s/%lf.jpg",
             image->camera_id_str().c_str(), timestamp);
    cv::imwrite(filename, image->mat());
  }
  AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << kCameraIdToStr.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec());

  // which camera should be used?  called in low frequence
  CameraSelection(timestamp);

  AINFO << "sub_camera_image_start_ts: "
        << GLOG_TIMESTAMP(sub_camera_image_start_ts)
        << " , last_proc_image_ts_: " << GLOG_TIMESTAMP(last_proc_image_ts_)
        << " , diff: "
        << GLOG_TIMESTAMP(sub_camera_image_start_ts - last_proc_image_ts_);

  const float proc_interval_seconds_ =
      1.0f / config_.tl_preprocessor_subnode_config().max_process_image_fps();

  if (last_proc_image_ts_ > 0.0 &&
      sub_camera_image_start_ts - last_proc_image_ts_ <
          proc_interval_seconds_) {
    AINFO << "skip current image, img_ts: " << GLOG_TIMESTAMP(timestamp)
          << " ,because proc_interval_seconds_: "
          << GLOG_TIMESTAMP(proc_interval_seconds_);
    return;
  }

  // sync image and publish data
  const double before_sync_image_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  bool should_pub = false;
  if (!preprocessor_.SyncImage(image, &image_lights, &should_pub)) {
    AINFO << "sync image failed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  } else {
    AINFO << "sync image succeed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  }
  const double sync_image_latency =
      TimeUtil::GetCurrentTime() - before_sync_image_ts;

  // Monitor image time and system time difference
  int max_cached_lights_size = preprocessor_.max_cached_lights_size();
  // tf frequency is 100Hz, 0.01 sec per frame，
  // cache frame num: max_cached_image_lights_array_size * 0.005 tf info
  const float tf_interval = 0.01;
  double image_sys_ts_diff_threshold = max_cached_lights_size * tf_interval;
  if (fabs(image_lights->diff_image_sys_ts) > image_sys_ts_diff_threshold) {
    std::string debug_string = "";
    debug_string += ("diff_image_sys_ts:" +
                     std::to_string(image_lights->diff_image_sys_ts));
    debug_string += (",camera_id:" + kCameraIdToStr.at(camera_id));
    debug_string += (",camera_ts:" + std::to_string(timestamp));

    AWARN << "image_ts - system_ts(in seconds): "
          << std::to_string(image_lights->diff_image_sys_ts)
          << ". Check if image timestamp drifts."
          << ", camera_id: " + kCameraIdToStr.at(camera_id)
          << ", debug_string: " << debug_string;
  }

  if (!should_pub) {
    AINFO << "TLPreprocessorSubnode not publish image, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
  }

  // verify lights projection based on image time
  if (!VerifyLightsProjection(image_lights)) {
    AINFO << "verify_lights_projection on image failed, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
  }

  // record current frame timestamp
  last_proc_image_ts_ = sub_camera_image_start_ts;

  image_lights->preprocess_receive_timestamp = sub_camera_image_start_ts;
  image_lights->preprocess_send_timestamp = TimeUtil::GetCurrentTime();
  if (AddDataAndPublishEvent(image_lights, camera_id, image->ts())) {
    preprocessor_.set_last_pub_camera_id(camera_id);
    AINFO << "TLPreprocessorSubnode::sub_camera_image msg_time: "
          << GLOG_TIMESTAMP(image->ts())
          << " sync_image_latency: " << sync_image_latency * 1000 << " ms."
          << " sub_camera_image_latency: "
          << (TimeUtil::GetCurrentTime() - sub_camera_image_start_ts) * 1000
          << " ms."
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    AINFO << " number of lights: " << image_lights->lights->size();
  }
}

bool TLPreprocessorSubnode::GetSignals(double ts, CarPose *pose,
                                       std::vector<Signal> *signals) {
  // get pose
  if (!GetCarPose(ts, pose)) {
    AERROR << "camera_selection failed to get car pose, ts:"
           << GLOG_TIMESTAMP(ts);
    return false;
  }
  AINFO << "camera_selection get position\n " << std::setprecision(12)
        << pose->pose();

  // get signals
  if (!hd_map_->GetSignals(pose->pose(), signals)) {
    if (ts - last_signals_ts_ < valid_hdmap_interval_) {
      *signals = last_signals_;
      AWARN << "camera_selection failed to get signals info. "
            << "Now use last info. ts:" << GLOG_TIMESTAMP(ts)
            << " pose:" << *pose;
    } else {
      AERROR << "camera_selection failed to get signals info. "
             << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << *pose;
      return false;
    }
  } else {
    last_signals_ = *signals;
    last_signals_ts_ = ts;
  }
  return true;
}
bool TLPreprocessorSubnode::GetCarPose(const double ts, CarPose *pose) {
  Eigen::Matrix4d pose_matrix;

  if (!GetVelodyneTrans(ts, &pose_matrix)) {
    AERROR << "TLPreprocessorSubnode failed to query pose ts:"
           << GLOG_TIMESTAMP(ts);
    return false;
  }
  pose->set_pose(pose_matrix);
  return true;
}
bool TLPreprocessorSubnode::VerifyLightsProjection(
    ImageLightsPtr image_lights) {
  std::vector<Signal> signals;
  CarPose pose;
  if (!GetSignals(image_lights->timestamp, &pose, &signals)) {
    return false;
  }

  // TODO(ghdawn): no need to init lights before this line
  image_lights->num_signals = signals.size();
  image_lights->lights.reset(new LightPtrs);
  image_lights->lights_outside_image.reset(new LightPtrs);
  if (!preprocessor_.ProjectLights(pose, signals, image_lights->camera_id,
                                   image_lights->lights.get(),
                                   image_lights->lights_outside_image.get())) {
    AINFO << "preprocessor_.select_camera_by_lights_projection failed";
    return false;
  }

  return true;
}
void TLPreprocessorSubnode::CameraSelection(double ts) {
  const double current_ts = TimeUtil::GetCurrentTime();
  AINFO << "current_ts: " << GLOG_TIMESTAMP(current_ts)
        << " , last_query_tf_ts: " << GLOG_TIMESTAMP(last_query_tf_ts_)
        << " , diff: " << GLOG_TIMESTAMP(current_ts - last_query_tf_ts_);
  if (last_query_tf_ts_ > 0.0 &&
      current_ts - last_query_tf_ts_ < config_.tl_preprocessor_subnode_config()
                                           .query_tf_inverval_seconds()) {
    AINFO << "skip current tf msg, img_ts: " << GLOG_TIMESTAMP(ts);
    return;
  }

  CarPose pose;
  std::vector<Signal> signals;
  if (!GetSignals(ts, &pose, &signals)) {
    return;
  }
  if (!preprocessor_.CacheLightsProjections(pose, signals, ts)) {
    AERROR << "add_cached_lights_projections failed, ts: "
           << GLOG_TIMESTAMP(ts);
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: "
          << GLOG_TIMESTAMP(ts);
  }
  last_query_tf_ts_ = current_ts;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/onboard/tl_proc_subnode.h"

#include <algorithm>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/recognizer/unity_recognize.h"
#include "modules/perception/traffic_light/rectify/cropbox.h"
#include "modules/perception/traffic_light/rectify/unity_rectify.h"
#include "modules/perception/traffic_light/reviser/color_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::time::Timer;
using apollo::common::adapter::AdapterManager;

namespace {

void OutputDebugImg(const std::shared_ptr<ImageLights> &image_lights,
                    const TrafficLightDebug *light_debug, cv::Mat *img) {
  const auto &lights = image_lights->lights;
  for (const auto &light : *lights) {
    cv::Rect rect = light->region.rectified_roi;
    cv::Scalar color;
    switch (light->status.color) {
      case BLACK:
        color = cv::Scalar(0, 0, 0);
        break;
      case GREEN:
        color = cv::Scalar(0, 255, 0);
        break;
      case RED:
        color = cv::Scalar(0, 0, 255);
        break;
      case YELLOW:
        color = cv::Scalar(0, 255, 255);
        break;
      default:
        color = cv::Scalar(0, 76, 153);
    }

    cv::rectangle(*img, rect, color, 2);
    cv::rectangle(*img, light->region.projection_roi, cv::Scalar(255, 255, 0),
                  2);
    auto &crop_roi = lights->at(0)->region.debug_roi[0];
    cv::rectangle(*img, crop_roi, cv::Scalar(0, 255, 255), 2);
  }

  int rows = img->rows;
  int cols = img->cols;
  double pos_x = cols / 1920.0 * 30.0;
  double step_y = rows / 1080.0 * 40.0;
  double font_scale = rows / 1080.0 * 3.0;
  double thickness = rows / 1080.0 * 2.0;

  // draw camera timestamp
  int pos_y = step_y;
  std::string ts_text = cv::format("img ts=%lf", image_lights->timestamp);
  cv::putText(*img, ts_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN,
              font_scale, CV_RGB(128, 255, 0), thickness);

  // draw distance to stopline
  pos_y += step_y;
  double distance = light_debug->distance_to_stop_line();
  if (lights->size() > 0) {
    std::string dis2sl_text = cv::format("dis2sl=%lf", distance);
    cv::putText(*img, dis2sl_text, cv::Point(pos_x, pos_y),
                cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(128, 255, 0),
                thickness);
  }

  // draw "Signals Num"
  pos_y += step_y;
  if (light_debug->valid_pos()) {
    std::string signal_txt = "Signals Num: " + std::to_string(lights->size());
    cv::putText(*img, signal_txt, cv::Point(pos_x, pos_y),
                cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 0, 0),
                thickness);
  }

  // draw "No Pose info."
  pos_y += step_y;
  if (!light_debug->valid_pos()) {
    cv::putText(*img, "No Valid Pose.", cv::Point(pos_x, pos_y),
                cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 0, 0),
                thickness);
  }

  // if image's timestamp is too early or too old
  // draw timestamp difference between image and pose
  pos_y += step_y;
  std::string diff_img_pose_ts_str =
      "ts diff: " + std::to_string(light_debug->ts_diff_pos());
  cv::putText(*img, diff_img_pose_ts_str, cv::Point(pos_x, pos_y),
              cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 0, 0), thickness);

  pos_y += step_y;
  std::string diff_img_sys_ts_str =
      "ts diff sys: " + std::to_string(light_debug->ts_diff_sys());
  cv::putText(*img, diff_img_sys_ts_str, cv::Point(pos_x, pos_y),
              cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 0, 0), thickness);

  pos_y += step_y;
  std::string signal_txt = "camera id: " + image_lights->image->camera_id_str();
  cv::putText(*img, signal_txt, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN,
              font_scale, CV_RGB(255, 0, 0), thickness);

  // draw image border size (offset between hdmap-box and detection-box)
  //    if (light_debug->project_error() > 100) {
  std::string img_border_txt =
      "Offset size: " + std::to_string(light_debug->project_error());
  int kPosYOffset = rows - (step_y * 2);
  cv::putText(*img, img_border_txt, cv::Point(pos_x, kPosYOffset),
              cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 0, 0), thickness);
  //    }

  cv::resize(*img, *img, cv::Size(960, 540));

  char filename[200];
  snprintf(filename, sizeof(filename), "img/%lf_%s.jpg",
           image_lights->image->ts(),
           image_lights->image->camera_id_str().c_str());
  cv::imwrite(filename, *img);
  cv::imshow("debug", *img);
  cv::waitKey(10);
}

}  // namespace

TLProcSubnode::~TLProcSubnode() { preprocessing_data_ = nullptr; }

bool TLProcSubnode::InitInternal() {
  RegisterFactoryUnityRectify();
  RegisterFactoryUnityRecognize();
  RegisterFactoryColorReviser();

  if (!InitSharedData()) {
    AERROR << "TLProcSubnode init shared data failed.";
    return false;
  }
  if (!InitRectifier()) {
    AERROR << "TLProcSubnode init rectifier failed.";
    return false;
  }
  if (!InitRecognizer()) {
    AERROR << "TLProcSubnode init recognizer failed.";
    return false;
  }
  if (!InitReviser()) {
    AERROR << "TLProcSubnode init reviser failed.";
    return false;
  }

  // init image_border
  if (!common::util::GetProtoFromFile(FLAGS_traffic_light_subnode_config,
                                      &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_subnode_config;
    return false;
  }
  return true;
}

bool TLProcSubnode::ProcEvent(const Event &event) {
  const double proc_subnode_handle_event_start_ts = TimeUtil::GetCurrentTime();
  PERF_FUNCTION("TLProcSubnode");
  // get up-stream data
  const double timestamp = event.timestamp;
  const std::string device_id = event.reserve;

  AINFO << "Detect Start ts:" << GLOG_TIMESTAMP(timestamp);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id, &key)) {
    AERROR << "TLProcSubnode produce_shared_data_key failed."
           << " ts:" << timestamp << " device_id:" << device_id;
    return false;
  }

  SharedDataPtr<ImageLights> image_lights;
  if (!preprocessing_data_->Get(key, &image_lights)) {
    AERROR << "TLProcSubnode failed to get shared data,"
           << " name:" << preprocessing_data_->name()
           << ", time: " << GLOG_TIMESTAMP(timestamp);
    return false;
  }
  AINFO << "TLProcSubnode get shared data ok,ts: " << GLOG_TIMESTAMP(timestamp);

  // preprocess send a msg -> proc receive a msg
  double enter_proc_latency = (proc_subnode_handle_event_start_ts -
                               image_lights->preprocess_send_timestamp);

  if (TimeUtil::GetCurrentTime() - event.local_timestamp >
      config_.tl_proc_subnode_config().valid_ts_interval()) {
    AERROR << "TLProcSubnode failed to process image"
           << "Because images are too old"
           << ",current time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime())
           << ", event time: " << GLOG_TIMESTAMP(event.local_timestamp);
    return false;
  }

  // verify image_lights from cameras
  RectifyOption rectify_option;
  if (!VerifyImageLights(*image_lights, &rectify_option.camera_id)) {
    AERROR << "TLProcSubnode invalid image_lights ";
    return false;
  }

  if (!image_lights->image->GenerateMat()) {
    AERROR << "TLProcSubnode failed to generate mat";
    return false;
  }
  // using rectifier to rectify the region.
  const double before_rectify_ts = TimeUtil::GetCurrentTime();
  if (!rectifier_->Rectify(*(image_lights->image), rectify_option,
                           (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to rectify the regions "
           << "ts:" << GLOG_TIMESTAMP(timestamp)
           << " Image:" << *(image_lights->image);
    return false;
  }
  const double detection_latency =
      TimeUtil::GetCurrentTime() - before_rectify_ts;

  // update image_border
  MutexLock lock(&mutex_);
  // int cam_id = static_cast<int>(image_lights->camera_id);
  ComputeImageBorder(*image_lights,
                     &image_border_size[image_lights->camera_id]);
  AINFO << "TLProcSubnode update image_border size: "
        << image_border_size[image_lights->camera_id]
        << " ts: " << GLOG_TIMESTAMP(timestamp)
        << " CameraId: " << image_lights->camera_id;
  image_lights->offset = image_border_size[image_lights->camera_id];

  // recognize_status
  const double before_recognization_ts = TimeUtil::GetCurrentTime();
  if (!recognizer_->RecognizeStatus(*(image_lights->image), RecognizeOption(),
                                    (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to recognize lights,"
           << " ts:" << GLOG_TIMESTAMP(timestamp)
           << " image:" << image_lights->image;
    return false;
  }
  const double recognization_latency =
      TimeUtil::GetCurrentTime() - before_recognization_ts;

  // revise status
  const double before_revise_ts = TimeUtil::GetCurrentTime();
  if (!reviser_->Revise(ReviseOption(event.timestamp),
                        image_lights->lights.get())) {
    AERROR << "TLReviserSubnode revise data failed. "
           << "sub_event:" << event.to_string();
    return false;
  }
  const double revise_latency = TimeUtil::GetCurrentTime() - before_revise_ts;
  PublishMessage(image_lights);
  AINFO << "TLProcSubnode process traffic_light, "
        << " msg_ts: " << GLOG_TIMESTAMP(timestamp)
        << " from device_id: " << device_id << " get "
        << image_lights->lights->size() << " lights."
        << " detection_latency: " << detection_latency * 1000 << " ms."
        << " recognization_latency: " << recognization_latency * 1000 << " ms."
        << " revise_latency: " << revise_latency * 1000 << " ms."
        << " TLProcSubnode::handle_event latency: "
        << (TimeUtil::GetCurrentTime() - proc_subnode_handle_event_start_ts) *
               1000
        << " ms."
        << " enter_proc_latency: " << enter_proc_latency * 1000 << " ms."
        << " preprocess_latency: "
        << (image_lights->preprocess_send_timestamp -
            image_lights->preprocess_receive_timestamp) *
               1000
        << " ms.";

  return true;
}

bool TLProcSubnode::InitSharedData() {
  CHECK_NOTNULL(shared_data_manager_);

  const std::string preprocessing_data_name("TLPreprocessingData");
  preprocessing_data_ = dynamic_cast<TLPreprocessingData *>(
      shared_data_manager_->GetSharedData(preprocessing_data_name));
  if (preprocessing_data_ == nullptr) {
    AERROR << "TLProcSubnode failed to get shared data instance: "
           << preprocessing_data_name;
    return false;
  }
  AINFO << "Init shared data successfully, "
        << "preprocessing_data: " << preprocessing_data_->name();
  return true;
}

bool TLProcSubnode::InitRectifier() {
  rectifier_.reset(BaseRectifierRegisterer::GetInstanceByName(
      FLAGS_traffic_light_rectifier));
  if (!rectifier_) {
    AERROR << "TLProcSubnode new rectifier failed. rectifier name:"
           << FLAGS_traffic_light_rectifier << " failed.";
    return false;
  }
  if (!rectifier_->Init()) {
    AERROR << "TLProcSubnode init rectifier failed. rectifier name:"
           << FLAGS_traffic_light_rectifier << " failed.";
    return false;
  }
  return true;
}

bool TLProcSubnode::InitRecognizer() {
  recognizer_.reset(BaseRecognizerRegisterer::GetInstanceByName(
      FLAGS_traffic_light_recognizer));
  if (!recognizer_) {
    AERROR << "TLProcSubnode new recognizer failed. name:"
           << FLAGS_traffic_light_recognizer;
    return false;
  }
  if (!recognizer_->Init()) {
    AERROR << "TLProcSubnode init recognizer failed.";
    return false;
  }
  return true;
}

bool TLProcSubnode::InitReviser() {
  reviser_.reset(
      BaseReviserRegisterer::GetInstanceByName(FLAGS_traffic_light_reviser));
  if (reviser_ == nullptr) {
    AERROR << "TLProcSubnode new reviser failed. name:"
           << FLAGS_traffic_light_reviser;
    return false;
  }
  if (!reviser_->Init()) {
    AERROR << "TLProcSubnode init reviser failed. name:"
           << FLAGS_traffic_light_reviser;
    return false;
  }
  return true;
}

double TLProcSubnode::GetMeanDistance(const double ts,
                                      const Eigen::Matrix4d &car_pose,
                                      const LightPtrs &lights) const {
  if (lights.empty()) {
    AWARN << "get_mean_distance failed. lights is empty, "
          << "while it should not be. ts:" << GLOG_TIMESTAMP(ts);
    return DBL_MAX;
  }

  double distance = 0.0;
  for (LightPtr light : lights) {
    auto light_distance = Distance2Stopline(car_pose, light->info.stop_line());
    if (light_distance < 0) {
      AWARN << "get_mean_distance failed. lights stop line data is illegal, "
            << "ts:" << GLOG_TIMESTAMP(ts);
      return DBL_MAX;
    }
    distance += light_distance;
  }
  return distance / lights.size();
}

bool TLProcSubnode::VerifyImageLights(const ImageLights &image_lights,
                                      CameraId *selection) const {
  if (!image_lights.image || !image_lights.image->contain_image()) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "verify_image_lights failed.";
    return false;
  }

  const int cam_id = static_cast<int>(image_lights.camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "TLProcSubnode image_lights unknown camera id: " << cam_id
           << " verify_image_lights failed.";
    return false;
  }
  for (LightPtr light : *(image_lights.lights)) {
    if (!BoxIsValid(light->region.projection_roi, image_lights.image->size())) {
      ClearBox(&(light->region.projection_roi));
      continue;
    }
  }
  *selection = image_lights.camera_id;

  return true;
}

bool TLProcSubnode::ComputeImageBorder(const ImageLights &image_lights,
                                       int *image_border) {
  if (!image_lights.image) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "compute_image_border failed.";
    return false;
  }

  auto camera_id = static_cast<int>(image_lights.camera_id);
  if (camera_id < 0 || camera_id >= kCountCameraId) {
    AERROR << "TLProcSubnode image_lights unknown camera selection, "
           << "compute_image_border failed, "
           << "camera_id: " << kCameraIdToStr.at(image_lights.camera_id);
    return false;
  }

  // check lights info
  if (image_lights.lights->empty()) {
    AINFO << "TLProcSubnode image_lights no lights info, "
          << "no need to update image border, reset image border size to 100";
    *image_border = 100;
    return true;
  }

  LightPtrs &lights_ref = *(image_lights.lights.get());
  int max_offset = -1;
  for (const auto &ref : lights_ref) {
    if (ref->region.is_detected) {
      cv::Rect rectified_roi = ref->region.rectified_roi;
      cv::Rect projection_roi = ref->region.projection_roi;
      // pick up traffic light with biggest offset
      int offset = 0;
      ComputeRectsOffset(projection_roi, rectified_roi, &offset);
      max_offset = std::max(max_offset, offset);
    }
  }
  if (max_offset != -1) {
    *image_border = max_offset;
  }

  return true;
}

void TLProcSubnode::ComputeRectsOffset(const cv::Rect &rect1,
                                       const cv::Rect &rect2, int *offset) {
  cv::Point center1(rect1.x + rect1.width / 2, rect1.y + rect1.height / 2);
  cv::Point center2(rect2.x + rect2.width / 2, rect2.y + rect2.height / 2);

  cv::Point pt1;
  cv::Point pt2;
  // record the max lateral and longitudinal offset
  if (center2.y <= center1.y) {
    if (center2.x >= center1.x) {
      pt1 = cv::Point(rect1.x + rect1.width, rect1.y);
      pt2 = cv::Point(rect2.x + rect2.width, rect2.y);
    } else {
      pt1 = cv::Point(rect1.x, rect1.y);
      pt2 = cv::Point(rect2.x, rect2.y);
    }
  } else {
    if (center2.x >= center1.x) {
      pt1 = cv::Point(rect1.x + rect1.width, rect1.y + rect1.height);
      pt2 = cv::Point(rect2.x + rect2.width, rect2.y + rect2.height);
    } else {
      pt1 = cv::Point(rect1.x, rect1.y + rect1.height);
      pt2 = cv::Point(rect2.x, rect2.y + rect2.height);
    }
  }

  *offset = std::max(abs(pt1.x - pt2.x), abs(pt1.y - pt2.y));
}

bool TLProcSubnode::PublishMessage(
    const std::shared_ptr<ImageLights> &image_lights) {
  Timer timer;
  timer.Start();
  const auto &lights = image_lights->lights;
  cv::Mat img = image_lights->image->mat();
  TrafficLightDetection result;
  AdapterManager::FillTrafficLightDetectionHeader("traffic_light", &result);
  auto *header = result.mutable_header();
  uint64_t img_timestamp =
      static_cast<uint64_t>(image_lights->image->ts() * 1e9);
  header->set_camera_timestamp(img_timestamp);

  // add traffic light result
  for (const auto &light : *lights) {
    TrafficLight *light_result = result.add_traffic_light();
    light_result->set_id(light->info.id().id());
    light_result->set_confidence(light->status.confidence);
    light_result->set_color(light->status.color);
  }

  // set contain_lights
  result.set_contain_lights(image_lights->num_signals > 0);

  // add traffic light debug info
  TrafficLightDebug *light_debug = result.mutable_traffic_light_debug();

  // set signal number
  AINFO << "TLOutputSubnode num_signals: " << image_lights->num_signals
        << ", camera_id: " << kCameraIdToStr.at(image_lights->camera_id)
        << ", is_pose_valid: " << image_lights->is_pose_valid
        << ", ts: " << GLOG_TIMESTAMP(image_lights->timestamp);
  light_debug->set_signal_num(image_lights->num_signals);

  // Crop ROI
  if (lights->size() > 0 && lights->at(0)->region.debug_roi.size() > 0) {
    auto &crop_roi = lights->at(0)->region.debug_roi[0];
    auto tl_cropbox = light_debug->mutable_cropbox();
    tl_cropbox->set_x(crop_roi.x);
    tl_cropbox->set_y(crop_roi.y);
    tl_cropbox->set_width(crop_roi.width);
    tl_cropbox->set_height(crop_roi.height);
  }

  // Rectified ROI
  for (const auto &light : *lights) {
    auto &rectified_roi = light->region.rectified_roi;
    auto tl_rectified_box = light_debug->add_box();
    tl_rectified_box->set_x(rectified_roi.x);
    tl_rectified_box->set_y(rectified_roi.y);
    tl_rectified_box->set_width(rectified_roi.width);
    tl_rectified_box->set_height(rectified_roi.height);
    tl_rectified_box->set_color(light->status.color);
    tl_rectified_box->set_selected(true);
  }

  // Projection ROI
  for (const auto &light : *lights) {
    auto &projection_roi = light->region.projection_roi;
    auto tl_projection_box = light_debug->add_box();
    tl_projection_box->set_x(projection_roi.x);
    tl_projection_box->set_y(projection_roi.y);
    tl_projection_box->set_width(projection_roi.width);
    tl_projection_box->set_height(projection_roi.height);
  }

  // debug ROI (candidate detection boxes)
  if (lights->size() > 0 && lights->at(0)->region.debug_roi.size() > 0) {
    for (size_t i = 1; i < lights->at(0)->region.debug_roi.size(); ++i) {
      auto &debug_roi = lights->at(0)->region.debug_roi[i];
      auto tl_debug_box = light_debug->add_box();
      tl_debug_box->set_x(debug_roi.x);
      tl_debug_box->set_y(debug_roi.y);
      tl_debug_box->set_width(debug_roi.width);
      tl_debug_box->set_height(debug_roi.height);
    }
  }

  light_debug->set_ts_diff_pos(image_lights->diff_image_pose_ts);
  light_debug->set_ts_diff_sys(image_lights->diff_image_sys_ts);
  light_debug->set_valid_pos(image_lights->is_pose_valid);
  light_debug->set_project_error(image_lights->offset);
  light_debug->set_camera_id(image_lights->camera_id);

  if (lights->size() > 0) {
    double distance = Distance2Stopline(image_lights->pose.pose(),
                                        lights->at(0)->info.stop_line());
    light_debug->set_distance_to_stop_line(distance);
  }
  if (FLAGS_output_debug_img) {
    OutputDebugImg(image_lights, light_debug, &img);
  }

  AdapterManager::PublishTrafficLightDetection(result);
  auto process_time =
      TimeUtil::GetCurrentTime() - image_lights->preprocess_receive_timestamp;
  AINFO << "Publish message "
        << " ts:" << GLOG_TIMESTAMP(image_lights->timestamp)
        << " device:" << image_lights->image->camera_id_str() << " consuming "
        << process_time * 1000 << " ms."
        << " number of lights:" << lights->size()
        << " lights:" << result.ShortDebugString();

  timer.End("TLProcSubnode::Publish message");
  return true;
}

Status TLProcSubnode::ProcEvents() {
  Event event;
  const EventMeta &event_meta = sub_meta_events_[0];
  if (!event_manager_->Subscribe(event_meta.event_id, &event)) {
    AERROR << "Failed to subscribe event: " << event_meta.event_id;
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to subscribe event.");
  }
  if (!ProcEvent(event)) {
    AERROR << "TLProcSubnode failed to handle event. "
           << "event:" << event.to_string();
    return Status(ErrorCode::PERCEPTION_ERROR,
                  "TLProcSubnode failed to handle event.");
  }
  return Status::OK();
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo