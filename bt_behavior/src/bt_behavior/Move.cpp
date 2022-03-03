// Copyright 2019 Intelligent Robotics Lab
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
#include <memory>

#include "bt_behavior/Move.hpp"

namespace bt_behavior
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: bt_behavior::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{
  config().blackboard->get("node", node_);
  sound_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("/commands/sound", 10);
}

void
Move::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;

  getInput("goal", goal);

  std::cerr << "GOING TO: " << goal.pose.position.x << " , " << goal.pose.position.y << std::endl;

  goal_.pose = goal;
}

BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Suceeded");

  kobuki_ros_interfaces::msg::Sound success_alert;
  success_alert.value = 6;
  sound_pub_->publish(success_alert);

  return BT::NodeStatus::SUCCESS;
}


}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bt_behavior::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<bt_behavior::Move>(
    "Move", builder);
}
