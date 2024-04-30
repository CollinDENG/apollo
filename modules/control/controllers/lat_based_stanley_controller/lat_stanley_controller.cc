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

#include "modules/control/controllers/lat_based_stanley_controller/lat_stanley_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "current_curvature,"
              << "steer_angle,"
              << "steering_position,"
              << "v" << std::endl;
}
}  // namespace

LatStanleyController::LatStanleyController()
    : name_("LQR-based Stanley Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

LatStanleyController::~LatStanleyController() { CloseLogFile(); }

bool LatStanleyController::LoadControlConf() {
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = lat_based_stanley_controller_conf_.ts();
  if (ts_ <= 0.0) {
    AERROR << "[LatController] Invalid control update interval.";
    return false;
  }
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = lat_based_stanley_controller_conf_.max_lateral_acceleration();
  minimum_speed_protection_ = FLAGS_minimum_speed_protection;

  return true;
}

void LatStanleyController::ProcessLogs(const SimpleLateralDebug *debug,
                                       const canbus::Chassis *chassis) {
  const std::string log_str = absl::StrCat(
      debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(),
      ",", debug->heading_error(), ",", debug->curvature(), ",",
      debug->steer_angle(), ",", chassis->steering_percentage(), ",",
      injector_->vehicle_state()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatStanleyController::InitializeFilters() {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(ts_, lat_based_stanley_controller_conf_.cutoff_freq(),
                          &den, &num);
  digital_filter_.set_coefficients(den, num);
}

Status LatStanleyController::Init(
    std::shared_ptr<DependencyInjector> injector) {
  if (!ControlTask::LoadConfig<LatBaseStanleyControllerConf>(
          &lat_based_stanley_controller_conf_)) {
    AERROR << "failed to load stanley control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "failed to load lat stanley control_conf");
  }
  injector_ = injector;
  if (!LoadControlConf()) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  InitializeFilters();
  return Status::OK();
}

void LatStanleyController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LatStanleyController::Stop() { CloseLogFile(); }

std::string LatStanleyController::Name() const { return name_; }

Status LatStanleyController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  auto vehicle_state = injector_->vehicle_state();
  auto previous_lon_debug = injector_->Get_previous_lon_debug_info();
  auto target_tracking_trajectory = *planning_published_trajectory;
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

  double front_axis_center_x =
      vehicle_state->x() + wheelbase_ * cos(vehicle_state->heading());
  double front_axis_center_y =
      vehicle_state->y() + wheelbase_ * sin(vehicle_state->heading());
  match_point_ = trajectory_analyzer_.QueryNearestPointByPosition(
      front_axis_center_x, front_axis_center_y);
  double lat_err =
      sqrt(pow(front_axis_center_x - match_point_.path_point().x(), 2) +
           pow(front_axis_center_y - match_point_.path_point().y(), 2));
  double k = 1.0;  // 速度调节系数
  double theta_e = match_point_.path_point().theta() - vehicle_state->heading();
  double delta_e = atan2(k * lat_err, vehicle_state->linear_velocity());
  double steer_angle = common::math::NormalizeAngle(delta_e + theta_e);

  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();
  // Compute the steering command limit with the given maximum lateral
  // acceleration
  const double steer_limit =
      FLAGS_set_steer_limit ? std::atan(max_lat_acc_ * wheelbase_ /
                                        (vehicle_state->linear_velocity() *
                                         vehicle_state->linear_velocity())) *
                                  steer_ratio_ * 180 / M_PI /
                                  steer_single_direction_max_degree_ * 100
                            : 100.0;

  const double steer_diff_with_max_rate =
      lat_based_stanley_controller_conf_.enable_maximum_steer_rate_limit()
          ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI /
                steer_single_direction_max_degree_ * 100
          : 100.0;

  const double steering_position = chassis->steering_percentage();
  pre_steering_position_ = steering_position;

  // Clamp the steer angle with steer limitations at current speed
  double steer_angle_limited =
      common::math::Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle_limited;
  debug->set_steer_angle_limited(steer_angle_limited);

  // Limit the steering command with the designed digital filter
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

  // Check if the steer is locked and hence the previous steer angle should be
  // executed
  if (injector_->vehicle_state()->gear() != canbus::Chassis::GEAR_REVERSE) {
    if ((std::abs(vehicle_state->linear_velocity()) <
             lat_based_stanley_controller_conf_.lock_steer_speed() ||
         previous_lon_debug->path_remain() <= 0) &&
        vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE &&
        chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE) {
      ADEBUG << "Into lock steer, path_remain is "
             << previous_lon_debug->path_remain() << "linear_velocity is "
             << vehicle_state->linear_velocity();
      steer_angle = pre_steer_angle_;
    }
  }

  // Set the steer commands
  cmd->set_steering_target(common::math::Clamp(
      steer_angle, pre_steer_angle_ - steer_diff_with_max_rate,
      pre_steer_angle_ + steer_diff_with_max_rate));
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  pre_steer_angle_ = cmd->steering_target();
  debug->set_lateral_error(lat_err);
  debug->set_ref_heading(match_point_.path_point().theta());
  debug->set_heading(vehicle_state->heading());
  debug->set_heading_error(theta_e);
  debug->set_curvature(match_point_.path_point().kappa());
  debug->set_steer_angle(steer_angle);
  debug->set_steering_position(steering_position);
  debug->set_ref_speed(vehicle_state->linear_velocity());

  AINFO << "current_lateral_error:" << lat_err << ", "
        << "current_ref_heading:" << match_point_.path_point().theta() << ", "
        << "current_heading:" << vehicle_state->heading() << ", "
        << "current_heading_error:" << theta_e << ", "
        << "current_curvature:" << match_point_.path_point().kappa() << ", "
        << "steer_angle:" << steer_angle << ", "
        << "steering_position:" << steering_position << ", "
        << "v:" << vehicle_state->linear_velocity() << ", " << std::endl;

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status LatStanleyController::Reset() { return Status::OK(); }

}  // namespace control
}  // namespace apollo
