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
#include <helper/param.h>
#include <helper/param_internal.h>
#include <glog/logging.h>
#include <algorithm>
#include <fstream>
#include <list>
#include <vector>

using std::vector;
using Eigen::Matrix4f;
namespace YAML {

// Define templated helper functions for YAML to encode/decode from custom data type.
// We need special care for emitters of Vector and Matrix!
template<>
struct convert<Eigen::Vector3f> {
  static Node encode(const Eigen::Vector3f& rhs) {
    Node node;
    node.push_back(rhs(0));
    node.push_back(rhs(1));
    node.push_back(rhs(2));
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector3f& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }
    rhs(0) = node[0].as<float>();
    rhs(1) = node[1].as<float>();
    rhs(2) = node[2].as<float>();
    return true;
  }
};

template<int M, int N>
struct convert<Eigen::Matrix<float, M, N>> {
  // Encode Matrix3f as three sequences of 3-element sequence (row-major)
  static Node encode(const Eigen::Matrix<float, M, N>& rhs) {
    Node node;
    for (int r = 0; r < M; r++) {
      Node row;
      for (int c = 0; c < N; c++) {
        row.push_back(rhs(r, c));
      }
      node.push_back(row);
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<float, M, N>& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != M) {
      return false;
    }
    for (int r = 0; r < M; r++) {
      if (!node[r].IsSequence() || node[r].size() != N) {
        return false;
      }
      for (int c = 0 ; c < N; c++) {
        rhs(r, c) = node[r][c].as<float>();
      }
    }
    return true;
  }
};
template<int M, int N>
Emitter& operator << (Emitter& out, const Eigen::Matrix<float, M, N>& v) {
  out << YAML::BeginSeq;
  for (int r = 0; r < M; r++) {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int c = 0; c < N; c++) {
      out << v(r, c);
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndSeq;
  return out;
}

template<>
struct convert<XP::RelocMsgFilterParam::Transform_t> {
  static Node encode(const XP::RelocMsgFilterParam::Transform_t& rhs) {
    Node node;
    node["use_homography"] = rhs.use_homography;
    node["Cam_to_Base"] = rhs.Cam_to_Base;
    node["ViSlamW_to_AmclW"] = rhs.ViSlamW_to_AmclW;
    node["ViSlam_to_Amcl_Homography2D"] = rhs.ViSlam_to_Amcl_Homography2D;
    return node;
  }

  static bool decode(const Node& node, XP::RelocMsgFilterParam::Transform_t& rhs) {  // NOLINT
    if (!node.IsMap()) {
      return false;
    }
    rhs.use_homography = node["use_homography"].as<bool>();
    rhs.Cam_to_Base = node["Cam_to_Base"].as<Eigen::Matrix4f>();
    rhs.ViSlamW_to_AmclW = node["ViSlamW_to_AmclW"].as<Eigen::Matrix4f>();
    rhs.ViSlam_to_Amcl_Homography2D = node["ViSlam_to_Amcl_Homography2D"].as<Eigen::Matrix3f>();
    return true;
  }
};

Emitter& operator << (Emitter& out, const XP::RelocMsgFilterParam::Transform_t& v) {
  // TODO(hangmeng): maybe need special care because of Eigen::Matrix4f
  Node node;
  node = v;
  out << node;
  return out;
}

}  // namespace YAML

namespace XP {
bool RelocMsgFilterParam::LoadFromYaml(const std::string& filename) {
  YAML::Node node = deserialize(filename);
  Transform = node["Transform"].as<Transform_t>();
  return true;
}

bool RelocMsgFilterParam::LoadFromCvYaml(const std::string& filename) {
  LOG(FATAL) << "Not implemented";
  return false;
}

bool RelocMsgFilterParam::WriteToYaml(const std::string& filename) {
  YAML::Emitter emitter;
  emitter << YAML::Key << "Transform"
          << YAML::Value << Transform;
  serialize(filename, emitter);
  return true;
}

bool RelocMsgFilterParam::WriteToCvYaml(const std::string& filename) {
  LOG(FATAL) << "Not implemented";
  return false;
}

}  // namespace XP
