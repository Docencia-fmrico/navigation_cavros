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
  
}

void
GetWayPoint::halt()
{
  std::cout << "GetWayPoint halt" << std::endl;
}

BT::NodeStatus
GetWayPoint::tick()
{
  /*std::deque< std::vector<double> > wps(10, std::vector<double> (2,0));
  getInput("waypoints", wps);

  // como NO HAY BLACKBOARD AQUI ,DEBEREMOS COGER WAYPOINTS POR EL INPUT Y DEVOLVER GOAL POR EL OUTPUT ( no funciona aunn)
  //std::deque< std::vector<double> > wps(10, std::vector<double> (2,0));
  //blackboard->get("waypoints",wps);
  
  ///debug///
  for (int i = 0; i < wps.size() ; i++) {
    std::cout <<"wps ["<< i << "]: " << wps[i][0] << " " <<  wps[i][1] << std::endl;
  }
  std::cout <<"goal:" << first[0] << " " <<  first[1] << std::endl;
  ///////////

  //blackboard->set("goal", first);
  
  geometry_msgs::msg::PoseStamped goal;
  goal.pose = first;
  setOutput(goal);
  return BT::NodeStatus::SUCCESS;*/
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetWayPoint>("GetWayPoint");
}
