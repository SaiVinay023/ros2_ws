// Copyright 2024 Intelligent Robotics Lab
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

#include <string>
#include <iostream>

#include "bt_bumpgo/Back.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{

using namespace std::chrono_literals;

Back::Back(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
bool found = config().blackboard->get("node", node_);
  if (!found) {
    throw std::runtime_error("Node not found in blackboard");
  }
  // Initialize vel_pub_ to publish to /output_vel
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 10);
}

void
Back::halt()
{
  // Optionally, you can stop the robot when the node is halted
  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.linear.x = 0.0;
  vel_msgs.angular.z = 0.0;
  vel_pub_->publish(vel_msgs);
}

BT::NodeStatus
Back::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  // Fill and publish velocities to move backward
  vel_msgs.linear.x = -0.1;  // Move backward with a small speed
  vel_msgs.angular.z = 0.0;
  vel_pub_->publish(vel_msgs);

  // Return SUCCESS after moving back for three seconds
  if ((node_->now() - start_time_) > rclcpp::Duration(3s))
  {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_bumpgo::Back>("Back");
}
