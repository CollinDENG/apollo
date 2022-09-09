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

#pragma once

#include <string>

#include "modules/perception/lidar/lib/interface/base_ground_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

class DummyGroundDetector : public BaseGroundDetector {
 public:
  DummyGroundDetector() { name_ = "DummyGroundDetector"; }

  virtual ~DummyGroundDetector() = default;

  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;

  // @brief: detect ground points from point cloud.
  // @param [in]: options
  // @param [in/out]: frame
  // non_ground_indices should be filled, required,
  // label field of point cloud can be filled, optional,
  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() override { return enable_; }

  std::string Name() const override { return name_; }
};  // class DummyGroundDetector

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
