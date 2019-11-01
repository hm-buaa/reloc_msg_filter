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
#include <glog/logging.h>
#include <Eigen/Dense>
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

Emitter& operator << (Emitter& out, const Eigen::Vector3f& v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v(0) << v(1) << v(2) << YAML::EndSeq;
  return out;
}
template<>
struct convert<cv::Size> {
  static Node encode(const cv::Size& rhs) {
    Node node;
    node.push_back(rhs.width);
    node.push_back(rhs.height);
    return node;
  }

  static bool decode(const Node& node, cv::Size& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.width = node[0].as<int>();
    rhs.height = node[1].as<int>();
    return true;
  }
};
Emitter& operator << (Emitter& out, const cv::Size& v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.width << v.height << YAML::EndSeq;
  return out;
}
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
template<int M, int N>
struct convert<cv::Matx<float, M, N>> {
  static Node encode(const cv::Matx<float, M, N>& rhs) {
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
  static bool decode(const Node& node, cv::Matx<float, M, N>& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != M) {
      return false;
    }
    for (int r = 0; r < M; r++) {
      if (!node[r].IsSequence()) {
        return false;
      }
      if (node[r].size() != N) {
        return false;
      }
      for (int c = 0 ; c < N; c++) {
        rhs(r, c) = node[r][c].as<float>();
      }
    }
    return true;
  }
};
template<>
struct convert<cv::Mat_<float>> {
  static Node encode(const cv::Mat_<float>& rhs) {
    Node node;
    for (int r = 0; r < rhs.rows; r++) {
      Node row;
      for (int c = 0; c < rhs.cols; c++) {
        row.push_back(rhs(r, c));
      }
      node.push_back(row);
    }
    return node;
  }
  static bool decode(const Node& node, cv::Mat_<float>& rhs) {  // NOLINT
    if (!node.IsSequence()) {
      return false;
    }
    const int M = node.size();
    if (!node[0].IsSequence()) {
      return false;
    }
    const int N = node[0].size();
    rhs.create(M, N);
    for (int r = 0; r < M; r++) {
      if (!node[r].IsSequence()) {
        return false;
      }
      if (node[r].size() != N) {
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
Emitter& operator << (Emitter& out, const cv::Matx<float, M, N>& v) {
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

}  // namespace XP
