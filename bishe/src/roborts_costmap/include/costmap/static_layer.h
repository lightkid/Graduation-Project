/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef ROBORTS_COSTMAP_STATIC_LAYER_H
#define ROBORTS_COSTMAP_STATIC_LAYER_H

#include <nav_msgs/OccupancyGrid.h>
#include "io/io.h"
#include "map_common.h"
#include "costmap_layer.h"
#include <ros/ros.h>
//new
#include "roborts_msgs/GameZone.h"
#include "roborts_msgs/GameZoneArray.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "roborts_msgs/CarObsInfo.h"
#include <visualization_msgs/Marker.h>

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Path.h"

#include <queue>

namespace roborts_costmap {
#define CONFIG_FILE_PATH "/home/hitcsc/RMAI/cfg/robot.txt"
class StaticLayer : public CostmapLayer {

 public:
  StaticLayer() {}
  virtual ~StaticLayer() {}
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void MatchSize();
 private:
  typedef struct carpoint_{
    int map_x;
    int map_y;
  }carpoint;
  void InComingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  //new
  void BonusZoneCallback(const roborts_msgs::GameZoneArray::ConstPtr & bonus);
  void UpdateBonus();
  void UpdateCar();
  void UpdateCarAlly();
  void UpdateCarEnemy();
  void UpdateMapStatic();
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
  void AllyCallback(const nav_msgs::Odometry::ConstPtr & pose);
  void EnemyCallback(const roborts_msgs::CarObsInfo::ConstPtr & pose);
  bool PoseInsideCar(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4,int xp,int yp);
  bool PoseInsideCircle(int x1,int y1,int x2,int y2,int radius);
  std::vector<int> OriginMap;
  std::vector<int> buff_state;//buff区的6个位置的状态buff1/debuff0
  ros::Subscriber bonus_sub;//接裁判系统
  ros::Subscriber pose_sub;//接位姿
  ros::Subscriber ally_sub;//接队友位置
  ros::Subscriber enemy_sub;//接敌人位置
  ros::Publisher roborts_footprint;//发给rviz机器人四个角
  std::vector<std::pair<int,int> > bonus_fields_record;//记录buff区位置
  int bonus_radius;
  int yaw_mode;
  double car_radius_x;
  double car_radius_y;
  double car_radius;
  unsigned char car_obs_val;
  unsigned char buff_obs_val;
  geometry_msgs::PoseStamped current_pose;//机器人当前位置map
  geometry_msgs::PoseStamped ally_pose;//队友当前位置
  std::vector<geometry_msgs::PoseStamped> enemy_pose;//敌人当前位置
  std::chrono::steady_clock::time_point ally_time;
  std::chrono::steady_clock::time_point ally_time_last;
  bool has_ally_;
  std::chrono::steady_clock::time_point enemy_time;
  std::chrono::steady_clock::time_point enemy_time_last;
  bool has_enemy_;
  std::vector<nav_msgs::Odometry> car_obs_;//车障碍物中心点
  std::vector<std::vector<carpoint_>> car_obs_points;//存放车障碍物点集
  std::vector<int> car_obs_state_;//车障碍物是否存在的标志
  std::vector<std::chrono::steady_clock::time_point> car_obs_time;//车障碍物更新时间
  int CarColor = -1;
  std::vector<carpoint_> car_obs_ally;
  std::vector<std::vector<carpoint_>> car_obs_enemy;//存放各个车障碍物的点集
  
  std::vector<geometry_msgs::PoseStamped> pose_history_;
  ros::Publisher originpoint_pub_;
  sensor_msgs::PointCloud originpoint_;
  ros::Publisher smoothpath_pub_;
  nav_msgs::Path smoothpath_;
  ros::Publisher mypoint_pub_;
  nav_msgs::Odometry mypoint_;

  // std::vector<std::vector<carpoint>> car_obs_last;//存放各个车障碍物的点集
  // std::chrono::milliseconds time_thre;
  // bool bonus_get_flag;
  // bool bonus_avoid_flag;
  // nav_msgs::OccupancyGrid buff_display;
  // ros::Publisher buff_display_pub;
  // std::chrono::steady_clock::time_point penalty_start_time;
  // std::chrono::steady_clock::time_point buff_start_time;
  // bool buff_active;
//  void IncomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  unsigned char InterpretValue(unsigned char value);
  std::string global_frame_;
  std::string map_frame_;
  std::string map_topic_;
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int staic_layer_x_, staic_layer_y_, width_, height_;
  unsigned char lethal_threshold_, unknown_cost_value_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;
};


} // namespace roborts_costmap
#endif
