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

#ifndef BT_BEHAVIOR__COSTMAP_2D_MAP_HPP_
#define BT_BEHAVIOR__COSTMAP_2D_MAP_HPP_

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include "geometry_msgs/msg/point.hpp"
#include "/opt/ros/foxy/include/nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Import constants from source files in github
// nav2_costmap_2d/cost_values.hpp
#define NO_INFORMATION 255
#define LETHAL_OBSTACLE 254
#define INSCRIBED_INFLATED_OBSTACLE 253
#define FREE_SPACE 0

// nav2_util/occ_grid_values.hpp
#define OCC_GRID_UNKNOWN -1
#define OCC_GRID_FREE 0
#define OCC_GRID_OCCUPIED 100

namespace navigation_cavros
{

class Costmap2D_map : public nav2_costmap_2d::Costmap2D
{
public:
  Costmap2D_map()
  // : size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
  {
    size_x_ = 0;
    size_y_ = 0;
    resolution_ = (0,0);
    origin_x_ = (0,0);
    origin_y_ = (0,0);
    costmap_ = NULL;
  }

  explicit Costmap2D_map(const nav_msgs::msg::OccupancyGrid & map)
  {
    // fill local variables
    size_x_ = map.info.width;
    size_y_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;

    // create the costmap
    costmap_ = new unsigned char[size_x_ * size_y_];

    // fill the costmap with a data
    int8_t data;
    for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
      data = map.data[it];
      if (data == OCC_GRID_UNKNOWN) {
        costmap_[it] = NO_INFORMATION;
      } else {
        // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
        // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
        costmap_[it] = std::round(
          static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
          (OCC_GRID_OCCUPIED - OCC_GRID_FREE));
      }
    }
  }

private:
};

}  // namespace navigation_cavros

#endif  // BT_BEHAVIOR__COSTMAP_2D_MAP_HPP_
