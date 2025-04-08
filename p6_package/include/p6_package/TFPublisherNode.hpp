// Copyright 2024 Mario Delicado Díaz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef P6_PACKAGE__TFPUBLISHERNODE_HPP_
#define P6_PACKAGE__TFPUBLISHERNODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/string.hpp"

namespace p6_package
{

class TFPublisherNode : public rclcpp::Node
{
public:
  TFPublisherNode();
  void control_callback(const std_msgs::msg::String::SharedPtr msg);

  bool is_active_;

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr entity_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscription_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;
  std::string current_entity_;

  void entity_callback(const std_msgs::msg::String::SharedPtr msg);
  void tf_callback(const vision_msgs::msg::Detection3DArray::SharedPtr detection_msg);
};

}  // namespace p6_package

#endif  // P6_PACKAGE__TFPUBLISHERNODE_HPP_
