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
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("patrolling_main");

  //read waypoint parameters and store them in a two-dimensional vector
  node->declare_parameter("waypoints");
  rclcpp::Parameter wps_param("waypoints",std::vector<std::string>({}));
  node->get_parameter("waypoints",wps_param);
  std::vector<std::string> wps = wps_param.as_string_array();

  std::deque< std::vector<double> > waypoints(wps.size(), std::vector<double> (2,0));

  for (int i = 0; i < wps.size() ; i++) {
    node->declare_parameter(wps[i]);
    rclcpp::Parameter wp_param(wps[i],std::vector<double>({}));
    node->get_parameter(wps[i],wp_param);
    waypoints[i] = wp_param.as_double_array();

    std::cout << wps[i] << " : " << waypoints[i][0] << " " <<  waypoints[i][1] << std::endl;
  }

  //behavior tree
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_GetWaypoint_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_behavior");
  std::string xml_file = pkgpath + "/behavior_tree_xml/behavior.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  blackboard->set("waypoints", waypoints);

  /////////// esto estará en GetWayPoints.cpp ////////// con otro nombre que no sea bla , claro
  std::deque< std::vector<double> > bla(wps.size(), std::vector<double> (2,0));
  blackboard->get("waypoints",bla);
  std::vector<double> first = bla[0];
  bla.pop_front();
  for (int i = 0; i < bla.size() ; i++) {
    std::cout <<"wps ["<< i << "]: " << bla[i][0] << " " <<  bla[i][1] << std::endl;
  }
  std::cout <<"goal:" << first[0] << " " <<  first[1] << std::endl;
  blackboard->set("goal", first);
  ///////////////////////////////////////////////////////

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
