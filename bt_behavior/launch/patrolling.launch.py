# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    patrolling_cmd = Node(
        package='bt_behavior',
        executable='patrolling_main',
        parameters=[{
          'use_sim_time': True
        }],
        remappings=[
          ('input_scan', '/scan_raw'),
          ('output_vel', '/nav_vel')
        ],
        output='screen'
    )

    pkg_dir = get_package_share_directory('bt_behavior')
    config_dir = os.path.join(pkg_dir, 'config')
    config_file = os.path.join(config_dir, 'waypoints.yaml')

    get_waypoint_cmd = Node(
        package='bt_behavior',
        node_executable='GetWayPoint',
        output='screen',
        parameters=[config_file]
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(patrolling_cmd)
    ld.add_action(get_waypoint_cmd)

    return ld
