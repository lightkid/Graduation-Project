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
#include "static_layer.h"

#include "static_layer_setting.pb.h"

namespace roborts_costmap {

void StaticLayer::OnInitialize() {
  ros::NodeHandle nh;
  is_current_ = true;
  ParaStaticLayer para_static_layer;
  // printf("\n\n__staticmapinit__\n\n");
  std::string static_map = ros::package::getPath("roborts_costmap") +
                           "/config/static_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(static_map.c_str(), &para_static_layer);
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  first_map_only_ = para_static_layer.first_map_only();
  subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
  track_unknown_space_ = para_static_layer.track_unknown_space();
  use_maximum_ = para_static_layer.use_maximum();
  int temp_threshold = para_static_layer.lethal_threshold();
  lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
  trinary_costmap_ = para_static_layer.trinary_map();
  unknown_cost_value_ = para_static_layer.unknown_cost_value();
  map_received_ = false;
  bool is_debug_ = para_static_layer.is_debug();
  map_topic_ = para_static_layer.topic_name();
  // new
  // ROS_INFO("StaticLayer map_topic_:%s", map_topic_.c_str());
  // printf("\n\n__gettingmap111__\n\n");
  bonus_radius = para_static_layer.bonus_radius();  // 0.24/0.05=5
  yaw_mode = para_static_layer.yaw_mode();
  car_radius_x = para_static_layer.car_radius_x();
  car_radius_y = para_static_layer.car_radius_y();
  car_radius = para_static_layer.car_radius();
  car_obs_val = para_static_layer.car_obs_val();
  buff_obs_val = para_static_layer.buff_obs_val();  // 251
  bonus_fields_record.push_back(
      std::make_pair(12, 58));  // F1(0.5m,2.79m)/0.05(10,55.8)+0.1/0.05
  bonus_fields_record.push_back(
      std::make_pair(40, 35));  // F2(1.9m,1.65m)(38,33)
  bonus_fields_record.push_back(
      std::make_pair(83, 83));  // F3(4.04m,4.035m)(80.8,80.7)
  bonus_fields_record.push_back(
      std::make_pair(83, 11));  // F4(4.04m,0.445m)(80.8,8.9)
  bonus_fields_record.push_back(
      std::make_pair(126, 59));  // F5(6.18m,2.83m)(123.6,56.6)
  bonus_fields_record.push_back(
      std::make_pair(154, 36));  // F6(7.58m,1.69m)(151.6,33.8)
  // buff_display.header.frame_id = "map";
  // buff_display.header.stamp = ros::Time::now();
  // buff_display.info.resolution = 0.05;
  // buff_display.info.width = 166;
  // buff_display.info.height = 94;
  // buff_display.info.origin.position.x = -0.1;
  // buff_display.info.origin.position.y = -0.1;
  // buff_display.info.origin.position.z = 0;
  // buff_display.info.origin.orientation.w = 1.0;
  // buff_display.data.resize(166 * 94,0);

  // FILE* fp = fopen(CONFIG_FILE_PATH,"rb");
  // char fIn[1024];
  // int ret;
  // for(int i = 0; i < 4; i++) ret = fscanf(fp, "%s\n", fIn);
  // //读取车辆颜色
  // ret = fscanf(fp, "%s\n", fIn);
  // if(fIn[0] == 'r' && fIn[1] == 'e' && fIn[2] == 'd') CarColor =
  // 2;//敌对方为红车 if(fIn[0] == 'b' && fIn[1] == 'l' && fIn[2] == 'u' &&
  // fIn[3] == 'e') CarColor = 1;//敌对方为蓝车
  CarColor = 1;
  buff_state.resize(6, 0);  // 记录六个buff区的状态
  car_obs_.resize(4);       // ally,enemy1,enemy2,enemy3(ally死后)
  car_obs_state_.resize(4, 0);
  car_obs_points.resize(4);
  car_obs_time.resize(4, std::chrono::steady_clock::now());
  // enemy_pose.resize(2);//记录两个敌人的位置
  // has_ally_=false;
  // has_enemy_=false;
  // car_obs.resize(3);
  // time_thre.count = 2000;//2s
  // bonus_get_flag = false;
  // bonus_avoid_flag = false;
  // buff_active = false;
  // printf("\n\n__gettingmap222__\n\n");
  bonus_sub = nh.subscribe<roborts_msgs::GameZoneArray>(
      "/game_zone_array_status", 2, &StaticLayer::BonusZoneCallback,
      this);  // 裁判系统
  ally_sub = nh.subscribe<nav_msgs::Odometry>(
      "ally_pose", 1, &StaticLayer::AllyCallback, this);  // 内部astarplanner
  enemy_sub = nh.subscribe<roborts_msgs::CarObsInfo>(
      "enemy_pose", 1, &StaticLayer::EnemyCallback, this);  // 内部astarplanner
  pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "amcl_pose", 2, &StaticLayer::PoseCallback,
      this);  // 仿真/robot_pose//好像有点卡/定位得到
  roborts_footprint = nh.advertise<visualization_msgs::Marker>(
      "roborts_footprint", 1);  // 发给rviz
  // buff_display_pub = nh.advertise<nav_msgs::OccupancyGrid>(name_ +
  // "/bonus_costmap", 10);//发给rviz
  // new end
  map_sub_ =
      nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);

  // originpoint_pub_=nh.advertise<sensor_msgs::PointCloud>("/originpoint",2);
  // smoothpath_pub_=nh.advertise<nav_msgs::Path>("smoothpath",2);
  // mypoint_pub_=nh.advertise<nav_msgs::Odometry>("mypoint",2);

  ros::Rate temp_rate(10);
  while (!map_received_) {  // 等InComingMap接到消息
    ros::spinOnce();
    temp_rate.sleep();
    // printf("\n\n__gettingmap__\n\n");
  }
  // printf("\n\n__getmap__\n\n");
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;
}
void StaticLayer::AllyCallback(const nav_msgs::Odometry::ConstPtr &pose) {
  car_obs_[0] = *pose;
  car_obs_state_[0] = 1;
  // nav_msgs::Odometry pos=*pose;
  // car_obs_time[0] = std::chrono::steady_clock::now();
  // std::chrono::milliseconds dt =
  // std::chrono::duration_cast<std::chrono::milliseconds>(ally_time-ally_time_last);
  // if(dt.count() > 100){//数据间隔时间过长，不做平均
  //     ally_pose = pos;
  // }else{//做平均
  //     ally_pose.pose.position.x = (pos.pose.position.x +
  //     ally_pose.pose.position.x)/2; ally_pose.pose.position.y =
  //     (pos.pose.position.y + ally_pose.pose.position.y)/2;
  //     ally_pose.pose.position.z = pos.pose.position.z;
  // }
  // ally_time_last=ally_time;
  // has_ally_=true;
}
void StaticLayer::EnemyCallback(const roborts_msgs::CarObsInfo::ConstPtr
                                    &pose) {  // 来的数据都是可信的/最多三个
  // car_obs_state_[0]=0;
  // car_obs_state_[1]=0;
  // printf("\nget_enemy2\n");
  for (int i = 0; i < car_obs_state_.size(); ++i) {
    car_obs_state_[i + 1] = 0;
  }
  // printf("\ninfosize:%d\n",(*pose).obs_car.size());
  for (int i = 0; i < (*pose).obs_car.size(); i++) {
    car_obs_[i + 1] = (*pose).obs_car[i];  // 从第二位开始放敌方车
    car_obs_state_[i + 1] = 1;
    car_obs_time[i + 1] = std::chrono::steady_clock::now();
  }
  // for(int i=0;i<2;i++){
  //     if((*pose).obs_car[i].pose.pose.position.x==0&&(*pose).obs_car[i].pose.pose.position.y==0){
  //         //astar时间长了，不更新值了
  //         car_obs_state_[i]=0;
  //     }else{
  //         car_obs_[i]=(*pose).obs_car[i];
  //         car_obs_state_[i]=1;
  //         car_obs_time[i]=std::chrono::steady_clock::now();
  //     }
  // }
  // 接到后要判断
  // geometry_msgs::PoseStamped pos=*pose;
  // enemy_time = std::chrono::steady_clock::now();
  // std::chrono::milliseconds dt =
  // std::chrono::duration_cast<std::chrono::milliseconds>(enemy_time-enemy_time_last);
  // if(dt.count() > 100){//数据间隔时间过长，不做平均
  //     //要判断这个是哪个敌人的更新
  // }else{//做平均
  //     ally_pose.pose.position.x = (pos.pose.position.x +
  //     ally_pose.pose.position.x)/2; ally_pose.pose.position.y =
  //     (pos.pose.position.y + ally_pose.pose.position.y)/2;
  //     ally_pose.pose.position.z = pos.pose.position.z;
  // }
}
void StaticLayer::PoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &pose) {
  current_pose = *pose;  // 更新当前位置
  ros::NodeHandle nh;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "pose";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.6;
  marker.scale.y = 0.45;
  marker.scale.z = 0.23;
  if (nh.getNamespace().c_str() != "/" &&
      nh.getNamespace().size() >
          5)  // 如果是多机器人，默认的命名空间就是/robot_x不是/
  {
    if (nh.getNamespace().substr(1) == "robot_0") {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;
    }
    if (nh.getNamespace().substr(1) == "robot_1") {
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.5;
    }
  }
  marker.lifetime = ros::Duration();
  marker.pose.position.x = current_pose.pose.position.x;
  marker.pose.position.y = current_pose.pose.position.y;
  marker.pose.position.z = 0;
  marker.pose.orientation = current_pose.pose.orientation;
  roborts_footprint.publish(marker);
  // 发给rviz
  //  geometry_msgs::PolygonStamped profile;
  //  double yaw = tf::getYaw(current_pose.pose.orientation);
  //  double x_ = current_pose.pose.position.x;
  //  double y_ = current_pose.pose.position.y;
  //  double a=0.31,b=0.25;
  //  geometry_msgs::Point32 point;
  //  point.z = 0;
  //  point.x = x_+ a*cos(yaw) + b*sin(yaw);
  //  point.y = y_+ a*sin(yaw) - b*cos(yaw);
  //  profile.polygon.points.push_back(point);
  //  point.x = x_+ a*cos(yaw) - b*sin(yaw);
  //  point.y = y_+ a*sin(yaw) + b*cos(yaw);
  //  profile.polygon.points.push_back(point);
  //  point.x = x_- a*cos(yaw) - b*sin(yaw);
  //  point.y = y_- a*sin(yaw) + b*cos(yaw);
  //  profile.polygon.points.push_back(point);
  //  point.x = x_- a*cos(yaw) + b*sin(yaw);
  //  point.y = y_- a*sin(yaw) - b*cos(yaw);
  //  profile.polygon.points.push_back(point);
  //  profile.header.frame_id = "map";
  //  profile.header.stamp = ros::Time::now();
  //  roborts_footprint.publish(profile);
  // 这里做轨迹拟合测试//20Hz/50ms
  //  std::chrono::steady_clock::time_point starttime =
  //  std::chrono::steady_clock::now();

  // 最小二乘法/3次曲线拟合结果较直
  // double h=0.05;
  // int N=10;
  // if(pose_history_.size()> N - 1)
  //   pose_history_.erase(pose_history_.begin());//删除第一个
  // pose_history_.push_back(current_pose);
  // double xa0=0,xa1=0,xa2=0,xa3=0;
  // double ya0=0,ya1=0,ya2=0,ya3=0;
  // double alpha0 = 0, alpha1 = 0, alpha2 = 0;
  // double beta1 = 0, beta2 = 0;
  // double A0 = 0, A1 = 0, A2 = 0, A3 = 0;
  // double f20=0;// = alpha0*alpha1 - beta1;
  // double f21=0;// = - alpha0 - alpha1;
  // double f30=0;// = alpha0*beta2 + alpha2*beta1 - alpha0*alpha1*alpha2;
  // double f31=0;// = alpha0*alpha1 + alpha0*alpha2 + alpha1*alpha2 - beta1 -
  // beta2; double f32=0;// = - alpha0 - alpha1 - alpha2;

  // int size = pose_history_.size();
  // // printf("\nsize=%d\n",size);
  // alpha0=(size-1)*h/2;
  // A0=size;
  // for(int i=0;i<size;++i){
  //   double ti=i*h;
  //   double f1=ti - alpha0;
  //   A1+=f1*f1;
  //   alpha1+=ti*f1*f1;
  // }
  // alpha1/=A1;
  // beta1=A1/A0;
  // f20 = alpha0*alpha1 - beta1;
  // f21 = - alpha0 - alpha1;
  // for(int i=0;i<size;++i){
  //   double ti=i*h;
  //   double f2=ti*ti+f21*ti+f20;
  //   A2+=f2*f2;
  //   alpha2+=ti*f2*f2;
  // }
  // alpha2/=A2;
  // beta2=A2/A1;
  // f30 = alpha0*beta2 + alpha2*beta1 - alpha0*alpha1*alpha2;
  // f31 = alpha0*alpha1 + alpha0*alpha2 + alpha1*alpha2 - beta1 - beta2;
  // f32 = - alpha0 - alpha1 - alpha2;
  // for(int i=0;i<size;++i){
  //   double ti=i*h;
  //   double f3=ti*ti*ti+f32*ti*ti+f31*ti+f30;
  //   A3+=f3*f3;
  // }
  // for(int i = 0; i < size; i++){
  //   double ti = i*h;
  //   double x=pose_history_[i].pose.position.x;
  //   double y=pose_history_[i].pose.position.y;
  //   xa0 += x;
  //   xa1 += x*(ti-alpha0);
  //   xa2 += x*(ti*ti+f21*ti+f20);
  //   xa3 += x*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //   ya0 += y;
  //   ya1 += y*(ti-alpha0);
  //   ya2 += y*(ti*ti+f21*ti+f20);
  //   ya3 += y*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  // }
  // double alpha=0.05;//光滑因子
  // double t=(size-1)*h;
  // A2+=alpha*(t+3*t*t+f32*t);
  // A3+=alpha*(12*t*t*t+(3+12*f32)*t*t+(1+4*f32)*f32*t);
  // xa0/=A0;  xa1/=A1;  xa2/=A2;  xa3/=A3;
  // ya0/=A0;  ya1/=A1;  ya2/=A2;  ya3/=A3;
  // //把轨迹显示出来/原始10个点和插值之后的
  // originpoint_.points.clear();
  // for(int i=0;i<pose_history_.size();++i){
  //   geometry_msgs::Point32 point;
  //   point.x=pose_history_[i].pose.position.x;
  //   point.y=pose_history_[i].pose.position.y;
  //   originpoint_.points.push_back(point);
  // }
  // originpoint_.header.frame_id="map";
  // smoothpath_.poses.clear();
  // for(int i=0;i<(size-1)*10;++i){
  //   double ti = i*h/10;
  //   geometry_msgs::PoseStamped point;
  //   point.header.frame_id="map";
  //   point.pose.position.x=xa0+xa1*(ti-alpha0)+xa2*(ti*ti+f21*ti+f20)+xa3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //   point.pose.position.y=ya0+ya1*(ti-alpha0)+ya2*(ti*ti+f21*ti+f20)+ya3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //   smoothpath_.poses.push_back(point);
  // }
  // smoothpath_.header.frame_id="map";
  // originpoint_pub_.publish(originpoint_);
  // smoothpath_pub_.publish(smoothpath_);
  // mypoint_.header.frame_id="map";
  // mypoint_.header.stamp=ros::Time::now();
  // double ti=9*h;
  // mypoint_.pose.pose.position.x=xa0+xa1*(ti-alpha0)+xa2*(ti*ti+f21*ti+f20)+xa3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  // mypoint_.pose.pose.position.y=ya0+ya1*(ti-alpha0)+ya2*(ti*ti+f21*ti+f20)+ya3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  // mypoint_.twist.twist.linear.x=xa1+2*xa2*ti+xa2*f21+3*xa3*ti*ti+2*xa3*f32*t+xa3*f31;
  // mypoint_.twist.twist.linear.y=ya1+2*ya2*ti+ya2*f21+3*ya3*ti*ti+2*ya3*f32*t+ya3*f31;
  // mypoint_pub_.publish(mypoint_);//速度不太准阿
  // printf("\n%ft3+%ft2+%ft1+%f\t%ft3+%ft2+%ft1+%f\n",xa3,xa2,xa1,xa0,ya3,ya2,ya1,ya0);
  // std::chrono::milliseconds usedtime =
  // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-starttime);
  // printf("\nusedtime:%d\n",usedtime.count());
  //   printf("\ng0:%f\tg1:%f\n",f20,f21);
  // printf("\nf0:%f\tf1:%f\tf2:%f\n",f30,f31,f32);
  // printf("\na0:%f\ta1:%f\tb1:%f\ta2:%f\tb2:%f\tA1:%f\tA2:%f\tA3:%f\n",
  // alpha0,alpha1,beta1,alpha2,beta2,A1,A2,A3);

  // 贝塞尔曲线
}
void StaticLayer::BonusZoneCallback(
    const roborts_msgs::GameZoneArray::ConstPtr &bonus) {
  roborts_msgs::GameZoneArray state = *bonus;
  // bool has_debuff=false;
  // 以red为例
  if (CarColor == 1) {
    for (int i = 0; i < state.zone.size(); i++) {  // 6
      if (state.zone[i].active) {                  // 激活状态
        if (state.zone[i].type == 1 || state.zone[i].type == 2) {  // buff
          buff_state[i] = 0;
          // bonus_get_flag = true;//有buff
        } else {  // debuff
          buff_state[i] = 1;
          // 看robotpose在不在buff区，如果就不加costmap

          // printf("\n\n__%dhasdebuff__\n\n",i+1);
          // printf("\n_%disdebuff_\n",i+1);
          // has_debuff = true;
        }
      } else {
        buff_state[i] = 0;  // 代表没激活，应该能及时清理flag
      }
    }
  } else {
    for (int i = 0; i < state.zone.size(); i++) {  // 6
      if (state.zone[i].active) {                  // 激活状态
        if (state.zone[i].type == 3 || state.zone[i].type == 4) {  // buff
          buff_state[i] = 0;
          // bonus_get_flag = true;//有buff
        } else {  // debuff
          buff_state[i] = 1;
          // 看robotpose在不在buff区，如果就不加costmap

          // printf("\n\n__%dhasdebuff__\n\n",i+1);
          // printf("\n_%disdebuff_\n",i+1);
          // has_debuff = true;
        }
      } else {
        buff_state[i] = 0;  // 代表没激活，应该能及时清理flag
      }
    }
  }

  // printf("\n\n__get_buff__\n\n");//success
  // if(has_debuff){
  //     bonus_avoid_flag = true;//有debuff
  // }
  // rviz
  // 新开一个map专门展示这个/不然全在一起很乱
}
void StaticLayer::
    UpdateBonus() {  // 把costmap_中buff区的块改一下值，根据6个区域的状态，便于失效后去除
  // 最开始初始化的时候应该得到了正常的地图，此处只需修改buff区域即可
  for (int k = 0; k < buff_state.size();
       k++) {  // 6/根据六个zone的编号对应更改costmap_
    int x = bonus_fields_record[k].first;
    int y = bonus_fields_record[k].second;
    int start_x = x - (bonus_radius + 1);  // 6/0.27
    int start_y = y - bonus_radius;        // 5/0.24
    int end_x = x + (bonus_radius + 1);
    int end_y = y + bonus_radius;
    int start_idx = start_y * 166 + start_x;
    unsigned int temp_idx = 0;
    if (start_idx > 0) {
      // printf("\n\n_good_\n\n");
      for (int i = start_y; i <= end_y; i++) {
        for (int j = start_x; j <= end_x; j++) {
          temp_idx = i * 166 + j;
          if (buff_state[k] == 1) {  // debuff,加入
            costmap_[temp_idx] = buff_obs_val;
            // buff_display.data[temp_idx]=buff_obs_val;
          } else {  // 正常，去掉/重刷地图就不用这样了/车在这块，不清
            if (costmap_[temp_idx] != car_obs_val) {  // 车不在，清
              costmap_[temp_idx] = FREE_SPACE;
              // buff_display.data[temp_idx]=buff_obs_val;
            }
          }
        }
      }
    }
  }
}
void StaticLayer::UpdateCar() {  // 根据车的数量看
  // 依次加入新的点
  //  printf("\nupdatecar\t%d\n",car_obs_state_.size());
  //  UpdateCarAlly();
  //  UpdateCarEnemy();
  for (int k = 0; k < car_obs_points.size(); ++k) {
    if (!car_obs_points[k].empty()) {  // 全清
      unsigned int temp_idx = 0, value = 0;
      for (auto &temp : car_obs_points[k]) {
        temp_idx = temp.map_y * 166 + temp.map_x;
        value = OriginMap.at(temp_idx);
        costmap_[temp_idx] = InterpretValue(value);
      }
      car_obs_points[k].clear();
    }
  }

  for (int k = 0; k < car_obs_state_.size(); k++) {
    if (car_obs_state_[k] == 1) {  // 存在
      // printf("\nget enemy\n");
      // 添加新的/现在没有车角度信息
      // std::chrono::steady_clock::time_point
      // now_time=std::chrono::steady_clock::now(); std::chrono::milliseconds dt
      // = std::chrono::duration_cast<std::chrono::milliseconds>(now_time -
      // car_obs_time[k]);
      // if(dt.count()<1000){//太久没更新了点就不能要了dt.count()<2000
      double radius_x = car_radius_x / 0.05;  // 转换到栅格地图
      double radius_y = car_radius_y / 0.05;
      int circle_radius = std::ceil(car_radius / 0.05);
      unsigned int x = 0;
      unsigned int y = 0;
      int length_1 = 0;
      int length_2 = 0;
      layered_costmap_->GetCostMap()->World2Map(
          car_obs_[k].pose.pose.position.x, car_obs_[k].pose.pose.position.y, x,
          y);
      if (1) {  // 没角度
        int bx, by, dx, dy;
        int x_ = x;
        int y_ = y;
        bx = std::min(x_ - 10, 0);
        by = std::min(y_ - 10, 0);
        dx = std::max(x_ + 10, 165);
        dy = std::max(y_ + 10, 165);
        unsigned int temp_idx = 0;
        for (int i = bx; i < dx; i++) {  // 在以车中心为中心的正方形中搜索
          for (int j = by; j < dy; j++) {
            temp_idx = i * 166 + j;
            if (temp_idx < 166 * 94) {
              if (PoseInsideCircle(x, y, j, i,
                                   circle_radius)) {  // 加入圆形的障碍
                carpoint temp_ = {j, i};              // x,y
                car_obs_points[k].push_back(temp_);
                costmap_[temp_idx] = car_obs_val;
                // printf("\nshowenemy\n");
              }
            }
          }
        }
      } else {  // 有角度
        double yaw =
            tf::getYaw(car_obs_[k].pose.pose.orientation);  // 没角度会报错
        int x1, x2, x3, x4, y1, y2, y3, y4;  // 机器人四个角
        x1 = x +
             std::ceil((radius_x + length_1) * cos(yaw) + radius_y * sin(yaw));
        y1 = y +
             std::ceil((radius_x + length_1) * sin(yaw) - radius_y * cos(yaw));
        x1 = std::max(0, x1);
        x1 = std::min(165, x1);
        y1 = std::max(0, y1);
        y1 = std::min(93, y1);
        x2 = x +
             std::ceil((radius_x + length_1) * cos(yaw) - radius_y * sin(yaw));
        y2 = y +
             std::ceil((radius_x + length_1) * sin(yaw) + radius_y * cos(yaw));
        x2 = std::max(0, x2);
        x2 = std::min(165, x2);
        y2 = std::max(0, y2);
        y2 = std::min(93, y2);
        x3 = x -
             std::ceil((radius_x + length_2) * cos(yaw) - radius_y * sin(yaw));
        y3 = y -
             std::ceil((radius_x + length_2) * sin(yaw) + radius_y * cos(yaw));
        x3 = std::max(0, x3);
        x3 = std::min(165, x3);
        y3 = std::max(0, y3);
        y3 = std::min(93, y3);
        x4 = x -
             std::ceil((radius_x + length_2) * cos(yaw) + radius_y * sin(yaw));
        y4 = y -
             std::ceil((radius_x + length_2) * sin(yaw) - radius_y * cos(yaw));
        x4 = std::max(0, x4);
        x4 = std::min(165, x4);
        y4 = std::max(0, y4);
        y4 = std::min(93, y4);
        int bx, by, dx, dy;
        int x_ = x;
        int y_ = y;
        bx = std::min(x_ - 10, 0);
        by = std::min(y_ - 10, 0);
        dx = std::max(x_ + 10, 165);
        dy = std::max(y_ + 10, 165);
        unsigned int temp_idx = 0;
        for (int i = bx; i < dx; i++) {  // 在以车中心为中心的正方形中搜索
          for (int j = by; j < dy; j++) {
            temp_idx = i * 166 + j;
            if (temp_idx < 166 * 94) {
              if (PoseInsideCar(x1, y1, x2, y2, x3, y3, x4, y4, j,
                                i)) {     // 加入方形的障碍
                carpoint temp_ = {j, i};  // x,y
                car_obs_points[k].push_back(temp_);
                costmap_[temp_idx] = car_obs_val;
              }
            }
          }
        }
      }
      // }else{//太久没更新
      //     car_obs_state_[k]=0;
      // }
    }
  }
}
void StaticLayer::UpdateCarAlly() {
  if (!has_ally_) return;
  // 重刷ally的旧点地图原数据
  if (!car_obs_ally.empty()) {
    unsigned int temp_idx = 0, value = 0;
    for (auto &temp : car_obs_ally) {
      temp_idx = temp.map_y * 166 + temp.map_x;
      value = OriginMap.at(temp_idx);
      costmap_[temp_idx] = InterpretValue(value);
    }
    car_obs_ally.clear();
  }
  std::chrono::steady_clock::time_point now_time =
      std::chrono::steady_clock::now();
  std::chrono::milliseconds allydt =
      std::chrono::duration_cast<std::chrono::milliseconds>(now_time -
                                                            ally_time);
  if (allydt.count() < 2000) {  // 太久没更新了点就不能要了
    double radius_x = car_radius_x / 0.05;  // 转换到栅格地图
    double radius_y = car_radius_y / 0.05;
    int circle_radius = std::ceil(car_radius / 0.05);
    unsigned int x = 0;
    unsigned int y = 0;
    int length_1 = 0;
    int length_2 = 0;
    layered_costmap_->GetCostMap()->World2Map(ally_pose.pose.position.x,
                                              ally_pose.pose.position.y, x, y);
    double yaw = tf::getYaw(ally_pose.pose.orientation);
    int x1, x2, x3, x4, y1, y2, y3, y4;  // 机器人四个角
    x1 = x + std::ceil((radius_x + length_1) * cos(yaw) + radius_y * sin(yaw));
    y1 = y + std::ceil((radius_x + length_1) * sin(yaw) - radius_y * cos(yaw));
    x1 = std::max(0, x1);
    x1 = std::min(165, x1);
    y1 = std::max(0, y1);
    y1 = std::min(93, y1);
    x2 = x + std::ceil((radius_x + length_1) * cos(yaw) - radius_y * sin(yaw));
    y2 = y + std::ceil((radius_x + length_1) * sin(yaw) + radius_y * cos(yaw));
    x2 = std::max(0, x2);
    x2 = std::min(165, x2);
    y2 = std::max(0, y2);
    y2 = std::min(93, y2);
    x3 = x - std::ceil((radius_x + length_2) * cos(yaw) - radius_y * sin(yaw));
    y3 = y - std::ceil((radius_x + length_2) * sin(yaw) + radius_y * cos(yaw));
    x3 = std::max(0, x3);
    x3 = std::min(165, x3);
    y3 = std::max(0, y3);
    y3 = std::min(93, y3);
    x4 = x - std::ceil((radius_x + length_2) * cos(yaw) + radius_y * sin(yaw));
    y4 = y - std::ceil((radius_x + length_2) * sin(yaw) - radius_y * cos(yaw));
    x4 = std::max(0, x4);
    x4 = std::min(165, x4);
    y4 = std::max(0, y4);
    y4 = std::min(93, y4);
    int bx, by, dx, dy;
    int x_ = x;
    int y_ = y;
    bx = std::min(x_ - 10, 0);
    by = std::min(y_ - 10, 0);
    dx = std::max(x_ + 10, 165);
    dy = std::max(y_ + 10, 165);
    unsigned int temp_idx = 0;
    for (int i = bx; i < dx; i++) {  // 在以车中心为中心的正方形中搜索
      for (int j = by; j < dy; j++) {
        temp_idx = i * 166 + j;
        if (temp_idx < 166 * 94) {
          if (PoseInsideCar(x1, y1, x2, y2, x3, y3, x4, y4, j,
                            i)) {     // 加入方形的障碍
            carpoint temp_ = {j, i};  // x,y
            car_obs_ally.push_back(temp_);
            costmap_[temp_idx] = car_obs_val;
          }
        }
      }
    }
  } else {
    has_ally_ = false;
  }
}
void StaticLayer::UpdateCarEnemy() {
  if (!has_enemy_) return;
  for (auto &it : car_obs_enemy) {  // 重刷enemy几辆车的旧点地图原数据
    unsigned int temp_idx = 0, value = 0;
    for (auto &temp : it) {
      temp_idx = temp.map_y * 166 + temp.map_x;
      value = OriginMap.at(temp_idx);
      costmap_[temp_idx] = InterpretValue(value);
    }
    // 清空点集
    it.clear();
  }
  double radius_x = car_radius_x / 0.05;  // 转换到栅格地图
  double radius_y = car_radius_y / 0.05;
  int circle_radius = std::ceil(car_radius / 0.05);
}
void StaticLayer::UpdateMapStatic() {
  // printf("\nupdatemap\n");

  UpdateCar();  // youwenti

  // if(has_ally_)
  // UpdateCarAlly();
  // if(has_enemy_)
  // UpdateCarEnemy();
  UpdateBonus();  // 要看车的位置，重叠，激活就更新，没激活就要保持车的obsval

  // buff_display.header.stamp = ros::Time::now();
  // buff_display_pub.publish(buff_display);
}
bool StaticLayer::PoseInsideCar(int x1, int y1, int x2, int y2, int x3, int y3,
                                int x4, int y4, int xp, int yp) {
  std::pair<int, int> A = std::make_pair(x1, y1);
  std::pair<int, int> B = std::make_pair(x2, y2);
  std::pair<int, int> C = std::make_pair(x4, y4);
  std::pair<int, int> D = std::make_pair(x3, y3);
  int a = (B.first - A.first) * (yp - A.second) -
          (B.second - A.second) * (xp - A.first);
  int b = (C.first - B.first) * (yp - B.second) -
          (C.second - B.second) * (xp - B.first);
  int c = (D.first - C.first) * (yp - C.second) -
          (D.second - C.second) * (xp - C.first);
  int d = (A.first - D.first) * (yp - D.second) -
          (A.second - D.second) * (xp - D.first);
  if ((a > 0 && b > 0 && c > 0 && d > 0) ||
      (a < 0 && b < 0 && c < 0 && d < 0)) {
    return true;
  }
  return false;
}
bool StaticLayer::PoseInsideCircle(int x1, int y1, int x2, int y2, int radius) {
  int d1 = (x1 - x2) * (x1 - x2);
  int d2 = (y1 - y2) * (y1 - y2);
  if (d1 + d2 > radius * radius) return false;
  return true;
}
void StaticLayer::MatchSize() {
  if (!layered_costmap_->IsRolling()) {
    Costmap2D *master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(),
              master->GetResolution(), master->GetOriginX(),
              master->GetOriginY());
  }
}

void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  printf("\n__map(%d,%d)__\n", size_x, size_y);
  auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if (!layered_costmap_->IsRolling() &&
      (master_map->GetSizeXCell() != size_x ||
       master_map->GetSizeYCell() != size_y ||
       master_map->GetResolution() != resolution ||
       master_map->GetOriginX() != origin_x ||
       master_map->GetOriginY() != origin_y ||
       !layered_costmap_->IsSizeLocked())) {
    layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y,
                                true);
  } else if (size_x_ != size_x || size_y_ != size_y ||
             resolution_ != resolution || origin_x_ != origin_x ||
             origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      value = new_map->data[temp_index];
      OriginMap.push_back(value);
      costmap_[temp_index] = InterpretValue(value);
      ++temp_index;
    }
  }
  map_received_ = true;
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

unsigned char StaticLayer::InterpretValue(unsigned char value) {
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double)value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::Activate() { OnInitialize(); }

void StaticLayer::Deactivate() {
  //    delete cost_map_;
  // shut down the map topic message subscriber
  map_sub_.shutdown();
}

void StaticLayer::Reset() {
  if (first_map_only_) {
    has_updated_data_ = true;
  } else {
    OnInitialize();
  }
}

void StaticLayer::UpdateBounds(double robot_x,  // 让地图大小正确
                               double robot_y, double robot_yaw, double *min_x,
                               double *min_y, double *max_x, double *max_y) {
  double wx, wy;
  if (!layered_costmap_->IsRollingWindow()) {
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
  // just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_ + width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  has_updated_data_ = false;
}

void StaticLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j,
                              int max_i, int max_j) {
  if (!map_received_) {
    return;
  }
  if (!layered_costmap_->IsRollingWindow()) {
    if (!use_maximum_) {
      UpdateMapStatic();
      UpdateOverwriteByAll(
          master_grid, min_i, min_j, max_i,
          max_j);  // 把costmap的某一块(真实的全部)用costmap_赋值
    } else {
      UpdateMapStatic();
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    try {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0),
                           temp_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for (auto i = min_i; i < max_i; ++i) {
      for (auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        p = temp_transform(p);
        if (World2Map(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.SetCost(i, j, GetCost(mx, my));
          } else {
            master_grid.SetCost(
                i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
          }
        }
      }
    }
  }
}

}  // namespace roborts_costmap
