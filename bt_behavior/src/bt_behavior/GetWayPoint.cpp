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
#include <vector>

#include "bt_behavior/GetWayPoint.hpp"

namespace bt_behavior
{

using namespace std::chrono_literals;

GetWayPoint::GetWayPoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // Read waypoint parameters and store them in a two-dimensional vector
  node_->declare_parameter("waypoints");
  rclcpp::Parameter wps_param("waypoints", std::vector<std::string>({}));
  node_->get_parameter("waypoints", wps_param);
  std::vector<std::string> wps = wps_param.as_string_array();

  for (int i = 0; i < wps.size(); i++) {
    node_->declare_parameter(wps[i]);
    rclcpp::Parameter wp_param(wps[i], std::vector<double>({}));
    node_->get_parameter(wps[i], wp_param);
    waypoints_.push_back(wp_param.as_double_array());
  }
}

void
GetWayPoint::halt()
{
  std::cerr << "GetWayPoint halt" << std::endl;
}

BT::NodeStatus
GetWayPoint::tick()
{
  geometry_msgs::msg::PoseStamped next_goal;


  if(waypoints_.size() == 0){
    next_goal.pose.position.x = (double)1000.0;
    next_goal.pose.position.y = (double)1000.0;
    setOutput("goal", next_goal);
    

    return BT::NodeStatus::FAILURE;

  }

  std::vector<double> first = waypoints_[0];
  
  next_goal.pose.position.x = first[0];
  next_goal.pose.position.y = first[1];
  setOutput("goal", next_goal);
  waypoints_.pop_front();

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetWayPoint>("GetWayPoint");
}
