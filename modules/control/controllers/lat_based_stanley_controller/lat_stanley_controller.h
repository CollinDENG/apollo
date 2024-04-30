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

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/control/controllers/lat_based_stanley_controller/proto/lat_based_stanley_controller_conf.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/common/filters/mean_filter.h"
#include "modules/control/control_component/controller_task_base/common/trajectory_analyzer.h"
#include "modules/control/control_component/controller_task_base/control_task.h"

namespace apollo {
namespace control {

using apollo::common::TrajectoryPoint;

class LatStanleyController : public ControlTask {
 public:
  LatStanleyController();

  virtual ~LatStanleyController();

  common::Status Init(std::shared_ptr<DependencyInjector> injector) override;

  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
      ControlCommand *cmd) override;

  common::Status Reset() override;

  void Stop() override;

  std::string Name() const override;

 protected:
  void UpdateDrivingOrientation();

  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug,
                            const canbus::Chassis *chassis);
  bool LoadControlConf();
  void InitializeFilters();
  void ProcessLogs(const SimpleLateralDebug *debug,
                   const canbus::Chassis *chassis);

  void CloseLogFile();
  LatBaseStanleyControllerConf lat_based_stanley_controller_conf_;
  common::VehicleParam vehicle_param_;
  TrajectoryAnalyzer trajectory_analyzer_;
  TrajectoryPoint match_point_;
  double ts_ = 0.0;
  // double cf_ = 0.0;
  // double cr_ = 0.0;
  double wheelbase_ = 0.0;
  // double mass_ = 0.0;
  // double lf_ = 0.0;
  // double lr_ = 0.0;
  // double iz_ = 0.0;
  double steer_ratio_ = 0.0;
  double steer_single_direction_max_degree_ = 0.0;
  double max_lat_acc_ = 0.0;
  common::DigitalFilter digital_filter_;
  // common::MeanFilter lateral_error_filter_;
  // common::MeanFilter heading_error_filter_;
  // bool enable_look_ahead_back_control_ = false;
  // double previous_lateral_acceleration_ = 0.0;
  // double previous_heading_rate_ = 0.0;
  // double previous_ref_heading_rate_ = 0.0;
  // double previous_heading_acceleration_ = 0.0;
  // double previous_ref_heading_acceleration_ = 0.0;
  std::ofstream steer_log_file_;
  const std::string name_;
  // double query_relative_time_;
  double pre_steer_angle_ = 0.0;
  double pre_steering_position_ = 0.0;
  double minimum_speed_protection_ = 0.1;
  // double current_trajectory_timestamp_ = -1.0;
  // double init_vehicle_x_ = 0.0;
  // double init_vehicle_y_ = 0.0;
  // double init_vehicle_heading_ = 0.0;
  // double low_speed_bound_ = 0.0;
  // double low_speed_window_ = 0.0;
  // double driving_orientation_ = 0.0;
  std::shared_ptr<DependencyInjector> injector_;
};

// 1.2 当前类声明为插件
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::control::LatStanleyController,
                                     ControlTask)

}  // namespace control
}  // namespace apollo
