// Copyright 2021 Intelligent Robotics Lab
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

#include "bt_behavior/GetWayPoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_behavior
{

using namespace std::chrono_literals;

GetWayPoint::GetWayPoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  //config().blackboard->set("wp", node_);
  this->declare_parameter("wp1", 1.0);
}

void
GetWayPoint::halt()
{
  std::cout << "GetWayPoint halt" << std::endl;
}

BT::NodeStatus
GetWayPoint::tick()
{
  /*
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }
  */

  rclcpp::Parameter<float> number = this->get_parameter("wp1").get_value<int>();
  std::cout << number << std::endl;
  //config().blackboard->set("wp", next);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetWayPoint>("GetWayPoint");
}
