cmake_minimum_required(VERSION 3.5)
project(bt_behavior)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    behaviortree_cpp_v3
    action_msgs
    lifecycle_msgs
    geometry_msgs
    nav2_msgs
    nav2_map_server
    ament_index_cpp
    nav_msgs
    nav2_costmap_2d
    kobuki_ros_interfaces
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(br2_move_bt_node SHARED src/bt_behavior/Move.cpp)
add_library(br2_GetWaypoint_bt_node SHARED src/bt_behavior/GetWayPoint.cpp)
list(APPEND plugin_libs
  br2_move_bt_node
  br2_GetWaypoint_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(patrolling_main src/patrolling_main.cpp)
ament_target_dependencies(patrolling_main ${dependencies})
target_link_libraries(patrolling_main ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  patrolling_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_tree_xml launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(tests)
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
