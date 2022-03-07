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
#include <iostream>

#include "bt_behavior/GetWayPoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_behavior
{

using namespace std::chrono_literals;
using std::placeholders::_1;

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

  sub_map_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 1, std::bind(&GetWayPoint::map_callback, this, _1));
}

void
GetWayPoint::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  costmap_ = navigation_cavros::Costmap2D_map(*msg);
  std::cout << "LAAALAA" << std::endl;
}

bool
GetWayPoint::is_occupied(std::vector<double> coordinate)
{
  std::vector<unsigned int> map_coordinate;
  unsigned int cost = 0;

  //costmap_.worldToMap(coordinate[0], coordinate[1], map_coordinate[0], map_coordinate[1]);
  //cost = costmap_.getCost(map_coordinate[0], map_coordinate[1]);
  if (cost > 0) {
    return true;
  }
  return false;
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
  if (waypoints_.size() == 0){
    return BT::NodeStatus::FAILURE;
  }
  std::vector<double> first = waypoints_[0];

  while (is_occupied(first)) {
    waypoints_.pop_front();
    std::cout << "Waypoint: (" << first[0] << ", " << first[1] << ")" << std::endl;
    first = waypoints_[0];
  }

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
