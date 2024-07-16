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

#ifndef ISOBSTACLE_HPP_
#define ISOBSTACLE_HPP_

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{

class IsObstacle : public BT::ConditionNode
{
public:
  explicit IsObstacle(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() {
    return BT::PortsList(
      {BT::InputPort<double>("distance")});
  }

  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg); 
 
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

}  // namespace bt_bumpgo

#endif  // ISOBSTACLE_HPP_
