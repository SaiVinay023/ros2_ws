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
#include <utility>

#include "bt_bumpgo/IsObstacle.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"


namespace bt_bumpgo
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsObstacle::IsObstacle(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
bool found = config().blackboard->get("node", node_);
if (!found) {
  throw std::runtime_error("Node not found in blackboard");
}
  // Initialize laser_sub_ subscribing to /input_scan
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/input_scan", 10, std::bind(&IsObstacle::laser_callback, this, _1));
}

void
IsObstacle::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  // Store last_scan_
  last_scan_ = std::move(msg);
}

BT::NodeStatus
IsObstacle::tick()
{
  if (last_scan_ == nullptr)
  {
    return BT::NodeStatus::FAILURE;
  }

  double distance = 1.0;
  getInput("distance", distance);

  // Check for obstacles within the specified distance
  for (const auto & range : last_scan_->ranges)
  {
    if (range < distance)
    {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_bumpgo::IsObstacle>("IsObstacle");
}
