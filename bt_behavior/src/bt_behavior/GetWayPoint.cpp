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
  std::cerr << "DDDDDDDDDDDDD" << std::endl;
  config().blackboard->get("node", node_);

  //read waypoint parameters and store them in a two-dimensional vector
  node_->declare_parameter("waypoints");
  rclcpp::Parameter wps_param("waypoints",std::vector<std::string>({}));
  node_->get_parameter("waypoints",wps_param);
  std::vector<std::string> wps = wps_param.as_string_array();

  std::cerr << "ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ" << std::endl;
  std::cerr << wps.size() << std::endl;



  //waypoints_(wps.size(), std::vector<double> (2,0));

  for (int i = 0; i < wps.size() ; i++) {
    node_->declare_parameter(wps[i]);
    rclcpp::Parameter wp_param(wps[i],std::vector<double>({}));
    std::cerr << "AAAAAAAAAAAAAA" << wps[i]<< std::endl;

    node_->get_parameter(wps[i],wp_param);
    waypoints_.push_back(wp_param.as_double_array());

    

    //debug
    std::cerr << wps[i] << " : " << waypoints_[i][0] << " " <<  waypoints_[i][1] << std::endl;
    ////////
  }

  std::cerr << "QQQQQQQQQQQQQ" << std::endl;


}

void
GetWayPoint::halt()
{
  std::cerr << "GetWayPoint halt" << std::endl;
}

BT::NodeStatus
GetWayPoint::tick()
{

  std::cerr << "TIIIIIIIIIIICK" << std::endl;

  std::vector<double> first = waypoints_[0];
  waypoints_.pop_front();

  //debug//
  for (int i = 0; i < waypoints_.size() ; i++) {
    std::cerr <<"wps ["<< i << "]: " << waypoints_[i][0] << " " <<  waypoints_[i][1] << std::endl;
  }
  std::cerr <<"goal:" << first[0] << " " <<  first[1] << std::endl;
  /////////

  config().blackboard->set("goal", first);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetWayPoint>("GetWayPoint");
}
