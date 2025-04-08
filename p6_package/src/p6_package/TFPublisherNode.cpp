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

#include "p6_package/TFPublisherNode.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

namespace p6_package
{

TFPublisherNode::TFPublisherNode()
: Node("tf_publisher_node"),
  is_active_(false)
{
  detection_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detection_3d", 10, std::bind(&TFPublisherNode::tf_callback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  entity_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "entities_topic", 10, std::bind(&TFPublisherNode::entity_callback, this, _1));

  control_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "start_topic", 10, std::bind(&TFPublisherNode::control_callback, this, _1));
}

void TFPublisherNode::control_callback(const std_msgs::msg::String::SharedPtr msg)
{
  is_active_ = (msg->data == "start");
}

void TFPublisherNode::entity_callback(const std_msgs::msg::String::SharedPtr msg)
{
  current_entity_ = msg->data;
}

void TFPublisherNode::tf_callback(const vision_msgs::msg::Detection3DArray::SharedPtr detection_msg)
{
  if (!is_active_ || detection_msg->detections.empty()) {
    return;
  }

  for (const auto & detection : detection_msg->detections) {
    double dist_x = detection.bbox.center.position.x;
    double dist_y = detection.bbox.center.position.y;
    double dist_z = detection.bbox.center.position.z;

    tf2::Transform camera2person;
    camera2person.setOrigin(tf2::Vector3(dist_x, dist_y, dist_z));
    camera2person.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    geometry_msgs::msg::TransformStamped odom2camera_msg;
    tf2::Stamped<tf2::Transform> odom2camera;

    try {
      odom2camera_msg = tf_buffer_.lookupTransform(
        "odom", "camera_depth_optical_frame", tf2::TimePointZero);
      tf2::fromMsg(odom2camera_msg, odom2camera);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Person transform not found: %s", ex.what());
      return;
    }

    tf2::Transform odom2person = odom2camera * camera2person;

    geometry_msgs::msg::TransformStamped odom2person_msg;
    odom2person_msg.transform = tf2::toMsg(odom2person);
    odom2person_msg.header.stamp = detection_msg->header.stamp;
    odom2person_msg.header.frame_id = "odom";
    odom2person_msg.child_frame_id = current_entity_;

    tf_broadcaster_->sendTransform(odom2person_msg);
  }
}

}  // namespace p6_package
