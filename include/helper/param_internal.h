/******************************************************************************
 * Copyright 2017-2019 Baidu Robotic Vision Authors. All Rights Reserved.
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
#ifndef XP_INCLUDE_XP_HELPER_PARAM_INTERNAL_H_
#define XP_INCLUDE_XP_HELPER_PARAM_INTERNAL_H_

#include <helper/param.h>
#include <string>
#include <vector>
#include <limits>
namespace XP {

class RelocMsgFilterParam : public ParamBase {
 public:
  RelocMsgFilterParam() {}
  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;

  struct Transform_t {
    Eigen::Matrix4f Cam_to_Base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ViSlamW_to_AmclW = Eigen::Matrix4f::Identity();
  } Transform;
};

}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_PARAM_INTERNAL_H_
