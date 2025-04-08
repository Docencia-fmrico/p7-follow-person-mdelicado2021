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

#include "p6_package/CycleNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace p6_package
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CycleNode::CycleNode()
: LifecycleNode("cycle_node"),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_)),
  current_entity_index_(0),
  is_active_(false)
{
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "entities_topic", 10, std::bind(&CycleNode::entity_callback, this, _1));

  control_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "start_topic", 10, std::bind(&CycleNode::control_callback, this, _1));

  double min_ref = -1.0;
  double max_ref = 1.0;
  double min_output = -0.5;
  double max_output = 0.5;

  linear_pid_ = std::make_unique<PIDController>(min_ref, max_ref, min_output, max_output);
  angular_pid_ = std::make_unique<PIDController>(min_ref, max_ref, min_output, max_output);
}

CallbackReturn CycleNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CycleNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  timer_ = create_wall_timer(100ms, std::bind(&CycleNode::timer_callback, this));  // <-- OK: no argumentos
  publisher_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  timer_ = nullptr;
  publisher_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CycleNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting Down...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CycleNode::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Error State");
  return CallbackReturn::SUCCESS;
}

void CycleNode::control_callback(const std_msgs::msg::String::SharedPtr msg)
{
  is_active_ = (msg->data == "start");
}

void CycleNode::entity_callback(const std_msgs::msg::String::SharedPtr msg)
{
  entities_.push_back(msg->data);
}

void CycleNode::timer_callback()
{
  if (!is_active_ || get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    return;

  if (entities_.empty() || current_entity_index_ >= entities_.size()) {
    RCLCPP_WARN(this->get_logger(), "No hay más entidades para buscar o índice fuera de rango.");
    return;
  }

  std::string target_frame = entities_[current_entity_index_];
  std::string robot_frame = "base_link";

  try {
    auto transformStamped = tf_buffer_->lookupTransform(
      robot_frame, target_frame, tf2::TimePointZero);

    distance = std::hypot(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
    angle = std::atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);

    target_distance = 1.0;
    target_angle = 0.0;

    distance_error = target_distance - distance;
    angle_error = target_angle - angle;

    linear_velocity = linear_pid_->get_output(distance_error);
    angular_velocity = angular_pid_->get_output(angle_error);

    if (distance_error == 0) {
      linear_velocity = 0.0;
      angular_velocity = 0.0;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    velocity_publisher_->publish(cmd_vel);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
      robot_frame.c_str(), target_frame.c_str(), ex.what());
  }
}

}  // namespace p6_package
