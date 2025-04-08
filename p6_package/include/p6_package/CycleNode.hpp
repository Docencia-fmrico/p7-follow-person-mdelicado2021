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
// limitations under the License.#ifndef P6_PACKAGE__CYCLENODE_HPP_

#ifndef P6_PACKAGE__CYCLENODE_HPP_
#define P6_PACKAGE__CYCLENODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/string.hpp"

#include "p6_package/PIDController.hpp"

namespace p6_package
{

class CycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CycleNode)

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CycleNode();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

  void control_callback(const std_msgs::msg::String::SharedPtr msg);
  void entity_callback(const std_msgs::msg::String::SharedPtr msg);  // <-- AHORA SÍ ESTÁ

  void timer_callback();  // <-- AHORA SIN ARGUMENTOS

  double distance;
  double angle;
  double target_distance;
  double target_angle;
  double distance_error;
  double angle_error;
  double linear_velocity;
  double angular_velocity;

  bool is_active_;

private:
  std::vector<std::string> entities_;
  size_t current_entity_index_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscription_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  std::unique_ptr<PIDController> linear_pid_;
  std::unique_ptr<PIDController> angular_pid_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace p6_package

#endif  // P6_PACKAGE__CYCLENODE_HPP_
