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

#include "a_star_planner.h"

namespace roborts_global_planner {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

AStarPlanner::AStarPlanner(CostmapPtr costmap_ptr)
    : GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
      gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
      gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
      cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {
  AStarPlannerConfig a_star_planner_config;
  std::string full_path = ros::package::getPath("roborts_planning") +
                          "/global_planner/a_star_planner/"
                          "config/a_star_planner_config.prototxt";

  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                             &a_star_planner_config)) {
    ROS_ERROR("Cannot load a star planner protobuf configuration file.");
  }
  //  AStarPlanner param config
  heuristic_factor_ = a_star_planner_config.heuristic_factor();
  inaccessible_cost_ = a_star_planner_config.inaccessible_cost();
  goal_search_tolerance_ = a_star_planner_config.goal_search_tolerance() /
                           costmap_ptr->GetCostMap()->GetResolution();
  block_fields_record.push_back(std::make_pair(12, 70));   // B1
  block_fields_record.push_back(std::make_pair(40, 47));   // B2
  block_fields_record.push_back(std::make_pair(34, 12));   // B3
  block_fields_record.push_back(std::make_pair(83, 71));   // B4
  block_fields_record.push_back(std::make_pair(83, 47));   // B5
  block_fields_record.push_back(std::make_pair(83, 23));   // B6
  block_fields_record.push_back(std::make_pair(132, 82));  // B7
  block_fields_record.push_back(std::make_pair(126, 47));  // B8
  block_fields_record.push_back(std::make_pair(154, 24));  // B9
  block_state.resize(9, 0);
  // block24568是场地上独立于边界的障碍物。
  special_block.push_back(std::make_pair(40, 47));
  special_block.push_back(std::make_pair(83, 71));
  special_block.push_back(std::make_pair(83, 47));
  special_block.push_back(std::make_pair(83, 23));
  special_block.push_back(std::make_pair(126, 47));
  double inflation_radius = 0.0;
  Blocks_ << 0.5, 3.38, 0.5 + inflation_radius, 0.1 + inflation_radius,  // B1
      1.9, 2.24, 0.4 + inflation_radius, 0.1 + inflation_radius,         // B2
      1.6, 0.5, 0.1 + inflation_radius, 0.5 + inflation_radius,          // B3
      4.04, 3.445, 0.5 + inflation_radius, 0.1 + inflation_radius,       // B4
      4.04, 2.24, 0.177 + inflation_radius, 0.177 + inflation_radius,    // B5
      4.04, 1.035, 0.5 + inflation_radius, 0.1 + inflation_radius,       // B6
      6.48, 3.98, 0.1 + inflation_radius, 0.5 + inflation_radius,        // B7
      6.18, 2.24, 0.4 + inflation_radius, 0.1 + inflation_radius,        // B8
      7.58, 1.1, 0.5 + inflation_radius, 0.1 + inflation_radius,         // B9
      4.19, -0.15, 4.19 + inflation_radius, 0.15 + inflation_radius,     // B10
      8.23, 2.39, 0.15 + inflation_radius, 2.39 + inflation_radius,      // B11
      3.89, 4.63, 4.19 + inflation_radius, 0.15 + inflation_radius,      // B12
      -0.15, 2.09, 0.15 + inflation_radius, 2.39 + inflation_radius;     // B13
  for (int i = 0; i < 10; ++i) {
    if (i == 4) {                                            // B5斜着
      Obstacles_(i * 4, 0) = Blocks_(i, 0) - Blocks_(i, 2);  //(cx-halfx,cy)
      Obstacles_(i * 4, 1) = Blocks_(i, 1);
      Obstacles_(i * 4 + 1, 0) = Blocks_(i, 0) + Blocks_(i, 2);  //(cx+halfx,cy)
      Obstacles_(i * 4 + 1, 1) = Blocks_(i, 1);
      Obstacles_(i * 4 + 2, 0) = Blocks_(i, 0);  //(cx,cy+hlafy)
      Obstacles_(i * 4 + 2, 1) = Blocks_(i, 1) + Blocks_(i, 3);
      Obstacles_(i * 4 + 3, 0) = Blocks_(i, 0);  //(cx,cy-hlafy)
      Obstacles_(i * 4 + 3, 1) = Blocks_(i, 1) - Blocks_(i, 3);
    } else if (i == 9) {         // 场地大圈
      Obstacles_(i * 4, 0) = 0;  //(cx-halfx,cy-hlafy)
      Obstacles_(i * 4, 1) = 0;
      Obstacles_(i * 4 + 1, 0) = map_length_;  //(cx+halfx,cy-hlafy)
      Obstacles_(i * 4 + 1, 1) = 0;
      Obstacles_(i * 4 + 2, 0) = map_length_;  //(cx+halfx,cy+hlafy)
      Obstacles_(i * 4 + 2, 1) = map_width_;
      Obstacles_(i * 4 + 3, 0) = 0;  //(cx-halfx,cy+hlafy)
      Obstacles_(i * 4 + 3, 1) = map_width_;
    } else {
      Obstacles_(i * 4, 0) =
          Blocks_(i, 0) - Blocks_(i, 2);  //(cx-halfx,cy-hlafy)
      Obstacles_(i * 4, 1) = Blocks_(i, 1) - Blocks_(i, 3);
      Obstacles_(i * 4 + 1, 0) =
          Blocks_(i, 0) + Blocks_(i, 2);  //(cx+halfx,cy-hlafy)
      Obstacles_(i * 4 + 1, 1) = Blocks_(i, 1) - Blocks_(i, 3);
      Obstacles_(i * 4 + 2, 0) =
          Blocks_(i, 0) + Blocks_(i, 2);  //(cx+halfx,cy+hlafy)
      Obstacles_(i * 4 + 2, 1) = Blocks_(i, 1) + Blocks_(i, 3);
      Obstacles_(i * 4 + 3, 0) =
          Blocks_(i, 0) - Blocks_(i, 2);  //(cx-halfx,cy+hlafy)
      Obstacles_(i * 4 + 3, 1) = Blocks_(i, 1) + Blocks_(i, 3);
    }
  }
  // obs_car_state_.resize(4,0);
  // carKalContainer.resize(2);
  // enemy_time_.resize(3,std::chrono::steady_clock::now());
  // 给enemy和ally初值/处值应该由哨岗第一次的数据得到
  // nav_msgs::Odometry initpose1;
  // initpose1.pose.pose.position.x=0.5;
  // initpose1.pose.pose.position.y=0.5;
  // initpose1.twist.twist.linear.x=0;
  // initpose1.twist.twist.linear.y=0;
  // nav_msgs::Odometry initpose2;
  // initpose2.pose.pose.position.x=7.58;
  // initpose2.pose.pose.position.y=3.98;
  // initpose2.twist.twist.linear.x=0;
  // initpose2.twist.twist.linear.y=0;
  // enemy1.SetX(initpose1);
  // enemy2.SetX(initpose2);
  // printf("\n__pos(%f,%f)__",enemy1.GetPose().pose.pose.position.x,enemy1.GetPose().pose.pose.position.y);
  // printf("__vel(%f,%f)__\n",enemy1.GetPose().twist.twist.linear.x,enemy1.GetPose().twist.twist.linear.y);
  ally_sub_ = nh_.subscribe<nav_msgs::Odometry>("ally/odom_fix", 1,
                                                &AStarPlanner::allyCallback,
                                                this);  // 速度和位姿//底层通信
  enemy_sub_ = nh_.subscribe<nav_msgs::Odometry>("outpost/planning", 1,
                                                 &AStarPlanner::enemyCallback,
                                                 this);  // 位置//底层通信
  ally_pub_ = nh_.advertise<nav_msgs::Odometry>("ally_pose",
                                                1);  // 内部/发给staticlayer
  enemy_pub_ = nh_.advertise<roborts_msgs::CarObsInfo>(
      "enemy_pose", 1);  // 内部/发给staticlayer
  relocalization_rqt_pub_ =
      nh_.advertise<std_msgs::Int32>("relocalization_rqt", 2);  // 重定位请求
  // lastpathpub=nh_.advertise<nav_msgs::Path>("globalpathlast",1);
  //     pathnow_pub_=nh_.advertise<nav_msgs::Path>("test/pathnow",1);
  //     pathlast_pub_=nh_.advertise<nav_msgs::Path>("test/pathlast",1);
  //     pathreplan_pub_=nh_.advertise<nav_msgs::Path>("test/pathreplan",1);
}

AStarPlanner::~AStarPlanner() { cost_ = nullptr; }
void AStarPlanner::allyCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // 将位置发给static_layer
  allypose_ = *msg;
  ally_pub_.publish(allypose_);
  // obs_car_state_[3]=1;
}
void AStarPlanner::enemyCallback(const nav_msgs::Odometry::ConstPtr
                                     &msg) {  // 微分跟踪器+卡尔曼滤波0.2/0.6ms
  // printf("\ngetenemy\n");
  nav_msgs::Odometry pos = *msg;
  std::chrono::steady_clock::time_point now_time =
      std::chrono::steady_clock::now();
  for (auto iter = carKalContainer.begin(); iter != carKalContainer.end();) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now_time -
                                                              iter->cartime)
            .count() > 2000) {  // 超时
      iter = carKalContainer.erase(
          iter);  // 当删除时erase函数自动指向下一个位置，就不需要进行++
    } else {
      iter++;  // 当没有进行删除的时候，迭代器++
    }
  }  // 去除动态障碍物队列中长时间未更新的车
  if (carKalContainer.empty()) {  // 没车
    carKalman car;
    car.SetX(pos);
    car.cartime = std::chrono::steady_clock::now();
    carKalContainer.push_back(car);
  } else {                // 有车
    double mindis = 1.5;  // 最远不能超过1.5m
    int index = carKalContainer.size() + 5;
    for (int i = 0; i < carKalContainer.size(); i++) {
      carKalContainer[i].predict();  // 一步预测
      double dis = getDistanceinOdom(pos, carKalContainer[i].GetPose());
      if (dis < mindis) {  // 匹配成功
        index = i;
        mindis = dis;
      }  // 得到的点与队列中的点进行匹配/匹配上就更新/没匹配上就pushback
    }    // for
    if (index < carKalContainer.size()) {  // 匹配上更新
      carKalContainer[index].updateK();
      carKalContainer[index].updateZ(pos);
      carKalContainer[index].updateX();
      carKalContainer[index].cartime = std::chrono::steady_clock::now();  // new
    } else if (carKalContainer.size() <
               3) {  // 没有匹配上的/加入/车最多3辆(2x敌+1x友)
      carKalman car;
      car.SetX(pos);
      car.cartime = std::chrono::steady_clock::now();
      carKalContainer.push_back(car);
    }
  }  // 有车/这时队列里的车都是较准的
  carinfo_.obs_car
      .clear();  // 丢失目标点需要有个保护！！！！！！！！！！！！！！
  for (auto iter = carKalContainer.begin(); iter != carKalContainer.end();
       iter++) {
    nav_msgs::Odometry car = iter->GetPose();
    car.header.stamp = ros::Time::now();
    carinfo_.obs_car.push_back(car);
  }
  enemy_pub_.publish(carinfo_);

  // printf("\n__sub_r0__\n");
  // 得到时间间隔
  // enemy_time=std::chrono::steady_clock::now();
  // std::chrono::milliseconds dt =
  // std::chrono::duration_cast<std::chrono::milliseconds>(enemy_time-enemy_time_last);
  // enemy1.Setdt(double(dt.count())/1000);
  // enemy2.Setdt(double(dt.count())/1000);
  // enemy1.Setdt(0.05);

  // printf("\n__sub_r0_over__\n");
  // printf("\n__pubmsgpos(%f,%f)__",pos.pose.pose.position.x,pos.pose.pose.position.y);
  // printf("__pubmsgvel(%f,%f)__\n",pos.twist.twist.linear.x,pos.twist.twist.linear.y);
  // printf("\n__pubpos(%f,%f)__",enemy1.GetPose().pose.pose.position.x,enemy1.GetPose().pose.pose.position.y);
  // printf("__pubvel(%f,%f)__\n",enemy1.GetPose().twist.twist.linear.x,enemy1.GetPose().twist.twist.linear.y);
  // kalman_pub_test_.publish(enemy1.GetPose());
}
double AStarPlanner::getDistanceinOdom(const nav_msgs::Odometry &pose1,
                                       const nav_msgs::Odometry &pose2) {
  return hypot(pose1.pose.pose.position.x - pose2.pose.pose.position.x,
               pose1.pose.pose.position.y - pose2.pose.pose.position.y);
}
ErrorInfo AStarPlanner::Plan(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             std::vector<geometry_msgs::PoseStamped> &path,
                             std::vector<geometry_msgs::PoseStamped> &last_path,
                             bool has_new_goal) {
  unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
  unsigned int valid_goal[2];
  unsigned int shortest_dist = std::numeric_limits<unsigned int>::max();
  bool goal_valid = false;

  if (!costmap_ptr_->GetCostMap()->World2Map(
          start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    ROS_WARN("Failed to transform start pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Start pose can't be transformed to costmap frame.");
  }
  if (!costmap_ptr_->GetCostMap()->World2Map(
          goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Goal pose can't be transformed to costmap frame.");
  }
  if (costmap_ptr_->GetCostMap()->GetCost(goal_x, goal_y) <
      inaccessible_cost_) {
    valid_goal[0] = goal_x;
    valid_goal[1] = goal_y;
    goal_valid = true;
  } else {
    tmp_goal_x = goal_x;
    tmp_goal_y = goal_y - goal_search_tolerance_;

    while (tmp_goal_y <= goal_y + goal_search_tolerance_) {
      tmp_goal_x = goal_x - goal_search_tolerance_;
      while (tmp_goal_x <= goal_x + goal_search_tolerance_) {
        unsigned char cost =
            costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
        unsigned int dist = abs(static_cast<int>(goal_x - tmp_goal_x)) +
                            abs(static_cast<int>(goal_y - tmp_goal_y));
        if (cost < inaccessible_cost_ && dist < shortest_dist) {
          shortest_dist = dist;
          valid_goal[0] = tmp_goal_x;
          valid_goal[1] = tmp_goal_y;
          goal_valid = true;
        }
        tmp_goal_x += 1;
      }
      tmp_goal_y += 1;
    }
  }
  last_path.assign(path.begin(), path.end());  // new
  ErrorInfo error_info;
  if (!goal_valid) {
    error_info = ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
    path.clear();
  } else {
    // 加入对起点的判断，卡住了就要把buff区去掉，卡住了就是踩buff了，自动buff就失效了？
    unsigned int start_index, goal_index;
    start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
    goal_index =
        costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);
    // 检测
    auto cost_start = costmap_ptr_->GetCostMap()->GetCost(start_x, start_y);
    // std_msgs::Int32 recovery;
    // if(cost_start >= BUFF_OBSTACLE)
    // {//起点在障碍物里说明定位飘了或者正好在buff刷新时重合
    //     //buff怎么处理???
    //     //对5个中心障碍物进行距离检查
    //     // auto index=10;
    //     // double distMin2=90000;
    //     // for(size_t i = 0; i < special_block.size(); ++i){
    //     //     double
    //     distNow2=(special_block[i].first-(int)start_x)*(special_block[i].first-(int)start_x)
    //     //
    //     +(special_block[i].second-(int)start_y)*(special_block[i].second-(int)start_y);
    //     //     if(distNow2 < distMin2){
    //     //         distMin2 = distNow2;
    //     //         index = i;
    //     //     }
    //     // }
    //     // //连线index中心与start
    //     // //dxdy描述
    //     // int obsx=special_block[index].first;
    //     // int obsy=special_block[index].second;
    //     // int startx=start_x;
    //     // int starty=start_y;
    //     // bool nearObs=true;
    //     // // bool nearX=true;
    //     // std::pair<double,double> V =
    //     std::make_pair(startx-obsx,starty-obsy);
    //     // if(fabs(V.first)>fabs(V.second)){//大的归一 L无穷范数?
    //     //     V.second /=V.first;
    //     //     V.first=1;
    //     // }else{
    //     //     V.first /=V.second;
    //     //     V.second=1;
    //     //     // nearX=false;
    //     // }
    //     // double xf=obsx;
    //     // double yf=obsy;
    //     // while(xf>1&&xf<160&&yf>1&&yf<90&&ros::ok()){
    //     //     if(fabs(xf-startx)<2||fabs(yf-starty)<2)
    //     //         break;
    //     //     xf += V.first;
    //     //     yf += V.second;
    //     //     if(costmap_ptr_->GetCostMap()->GetCost((unsigned int)xf,
    //     (unsigned int)yf)<inaccessible_cost_){
    //     //         nearObs=false;
    //     //         //以start向障碍物中心方向清除
    //     //         break;
    //     //     }
    //     // }
    //     // if(nearObs){//以start向障碍物反向清除V方向不变
    //     // }else{//以start向障碍物中心方向清除V反向
    //     //     V.first=-V.first;
    //     //     V.second=-V.second;
    //     // }
    //     // double x=startx;
    //     // double y=starty;
    //     // while(ros::ok()&&x>1&&x<160&&y>1&&y<90){
    //     //     if(costmap_ptr_->GetCostMap()->GetCost((unsigned int)x,
    //     (unsigned int)y) > inaccessible_cost_){
    //     //         costmap_ptr_->GetCostMap()->SetCost((unsigned int)x,
    //     (unsigned int)y,roborts_costmap::FREE_SPACE);
    //     //     }else{
    //     //         break;
    //     //     }
    //     //     x +=V.first;
    //     //     y +=V.second;
    //     // }
    //     //另一种方法，找一个邻近free点
    //     int startx=start_x;
    //     int starty=start_y;
    //     int tmpx = startx;
    //     int tmpy = starty - 10>1?starty-10:1;
    //     unsigned  int distMin = std::numeric_limits<unsigned int>::max();
    //     bool getFreeTmp=false;
    //     while(tmpy <= starty + 10){
    //         tmpx = startx - 10?startx-10:1;
    //         while(tmpx <= startx + 10){
    //             unsigned char cost =
    //             costmap_ptr_->GetCostMap()->GetCost((unsigned int)tmpx,
    //             (unsigned int)tmpy); unsigned int dist =
    //             abs(static_cast<int>(startx - tmpx)) +
    //             abs(static_cast<int>(starty - tmpy)); if (cost <
    //             inaccessible_cost_ && dist < distMin ) {
    //                 distMin = dist;
    //                 // valid_goal[0] = tmpy;
    //                 // valid_goal[1] = tmpy;
    //                 getFreeTmp = true;
    //             }
    //             tmpx += 1;
    //         }
    //         tmpy += 1;
    //     }
    //     if(getFreeTmp){
    //         std::pair<double,double> V = std::make_pair(tmpx-startx,
    //         tmpx-starty); if(fabs(V.first)>fabs(V.second)){//大的归一
    //         L无穷范数?
    //             V.second /=V.first;
    //             V.first=1;
    //         }else{
    //             V.first /=V.second;
    //             V.second=1;
    //         }
    //         double x=startx;
    //         double y=starty;
    //         while(ros::ok()&&x>1&&x<160&&y>1&&y<90){
    //             if(costmap_ptr_->GetCostMap()->GetCost((unsigned int)x,
    //             (unsigned int)y) > inaccessible_cost_){
    //                 costmap_ptr_->GetCostMap()->SetCost((unsigned int)x,
    //                 (unsigned int)y,roborts_costmap::FREE_SPACE);
    //             }else{
    //                 break;
    //             }
    //             x +=V.first;
    //             y +=V.second;
    //         }
    //     }
    //     std_msgs::Int32 val;
    //     val.data=1;
    //     relocalization_rqt_pub_.publish(val);
    // }
    costmap_ptr_->GetCostMap()->SetCost(start_x, start_y,
                                        roborts_costmap::FREE_SPACE);

    if (start_index == goal_index) {
      error_info = ErrorInfo::OK();
      path.clear();
      path.push_back(start);
      path.push_back(goal);
    } else {
      error_info =
          SearchPath(start_index, goal_index, path, last_path, has_new_goal);
      if (error_info.IsOK()) {
        path.back().pose.orientation =
            goal.pose.orientation;  // 将目标点的角度传入TEB
        path.back().pose.position.z = goal.pose.position.z;
      }
    }
  }
  //         //bishetest
  // static int count=0;
  //         geometry_msgs::PoseStamped start1;
  //         start1.pose.position.x=2.08;
  //         start1.pose.position.y=3.66;
  //         geometry_msgs::PoseStamped start2;
  //         start2.pose.position.x=2.11;
  //         start2.pose.position.y=3.39;
  //         geometry_msgs::PoseStamped end1;
  //         end1.pose.position.x=5.69;
  //         end1.pose.position.y=1.21;
  //         // std::vector<geometry_msgs::PoseStamped> path_last;
  //         // std::vector<geometry_msgs::PoseStamped> path_now;
  //         // std::vector<geometry_msgs::PoseStamped> path_replan;
  //         // std::vector<geometry_msgs::PoseStamped> nouse;
  //         int index_start=0,index_goal=0;
  //         unsigned int start1x,start1y,end1x,end1y,start2x,start2y;
  //         costmap_ptr_->GetCostMap()->World2Map(start1.pose.position.x,start1.pose.position.y,start1x,start1y);
  //         costmap_ptr_->GetCostMap()->World2Map(start2.pose.position.x,start2.pose.position.y,start2x,start2y);
  //         costmap_ptr_->GetCostMap()->World2Map(end1.pose.position.x,end1.pose.position.y,end1x,end1y);
  //         index_start=costmap_ptr_->GetCostMap()->GetIndex(start1x,start1y);
  //         index_goal=costmap_ptr_->GetCostMap()->GetIndex(end1x,end1y);
  //         if(count==0){
  //         SearchPath(index_start,index_goal,path_last,nouse,true);//上一时刻
  //         index_start=costmap_ptr_->GetCostMap()->GetIndex(start2x,start2y);
  //         SearchPath(index_start,index_goal,path_now,nouse,true);//当前时刻发生跳变
  //         // count++;
  //         }
  //         nav_msgs::Path pathlast;
  //         nav_msgs::Path pathnow;
  //         nav_msgs::Path pathreplan;
  //         pathlast.header.frame_id =
  //         costmap_ptr_->GetGlobalFrameID();//global path和global costmap
  //         ID一致 pathnow.header.frame_id =
  //         costmap_ptr_->GetGlobalFrameID();//global path和global costmap
  //         ID一致 pathreplan.header.frame_id =
  //         costmap_ptr_->GetGlobalFrameID();//global path和global costmap
  //         ID一致 pathlast.poses=path_last; pathnow.poses=path_now;
  //         pathlast_pub_.publish(pathlast);
  //         pathnow_pub_.publish(pathnow);
  //         if(count>=1){
  //         nav_msgs::Odometry carobstest;
  //         // path_replan.assign(path_last.begin(), path_last.end());//new
  //         carobstest.pose.pose.position.x=3.18;
  //         carobstest.pose.pose.position.y=3.59;
  //         // while(ros::ok()&&PathObsCheck(path_last)==false){
  //         //     // PathObsCheck(path_last);
  //         // }
  //         GetOutFromACar(path_last,carobstest,0.0,0.0,0.0,0.0);
  //         printf("\nreplan%d\n",path_last.size());
  //         if(count>=3){
  //             std::vector<geometry_msgs::PoseStamped> tmppath1;
  //             SearchPath(i_start_test,i_goal_test,tmppath1,nouse,true);//上一时刻
  //             path_replan.assign(path_last.begin(), path_last.end());//new
  //             path_replan.erase(path_replan.begin()+pathreplanstart,path_replan.begin()+pathreplanend+1);//擦除这段交叉的路径
  //             path_replan.insert(path_replan.begin()+pathreplanstart,tmppath1.begin(),tmppath1.end());//插入新生成的无碰撞路径
  //             printf("\nreplanover\n");
  //             //改起点
  //             double newstartx=path_now[0].pose.position.x;
  //             double newstarty=path_now[0].pose.position.y;
  //             double distmin=1000000.0;
  //             unsigned int iselect=0;
  //             for(unsigned int i=0;i<path_replan.size();++i){
  //                 double xnow=path_replan[i].pose.position.x;
  //                 double ynow=path_replan[i].pose.position.y;
  //                 double
  //                 distnow=(xnow-newstartx)*(xnow-newstartx)+(ynow-newstarty)*(ynow-newstarty);
  //                 if(distnow<distmin){
  //                     distmin=distnow;
  //                     iselect = i;
  //                 }
  //             }
  //             unsigned int xtmp,ytmp,xnew,ynew;
  //             costmap_ptr_->GetCostMap()->World2Map(path_replan[iselect].pose.position.x,path_replan[iselect].pose.position.y,xtmp,ytmp);
  //             costmap_ptr_->GetCostMap()->World2Map(newstartx,newstarty,xnew,ynew);
  //             unsigned int indextmp =
  //             costmap_ptr_->GetCostMap()->GetIndex(xtmp,ytmp); unsigned int
  //             indexnew = costmap_ptr_->GetCostMap()->GetIndex(xnew,ynew);
  //             std::vector<geometry_msgs::PoseStamped> tmppath2;
  //             SearchPath(indexnew,indextmp,tmppath2,nouse,true);//上一时刻
  //             path_replan.erase(path_replan.begin(),path_replan.begin()+iselect+1);//擦除这段交叉的路径
  //             path_replan.insert(path_replan.begin(),tmppath2.begin(),tmppath2.end());//插入新生成的无碰撞路径
  //             pathreplan.poses=path_replan;

  //             pathreplan_pub_.publish(pathreplan);
  // }

  // }
  // count++;

  return error_info;
}

ErrorInfo AStarPlanner::SearchPath(
    const int &start_index, const int &goal_index,
    std::vector<geometry_msgs::PoseStamped> &path,
    std::vector<geometry_msgs::PoseStamped> &last_path, bool has_new_goal) {
  g_score_.clear();
  f_score_.clear();
  parent_.clear();
  state_.clear();
  gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
  gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
  // ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
  cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
  g_score_.resize(gridmap_height_ * gridmap_width_,
                  std::numeric_limits<int>::max());
  f_score_.resize(gridmap_height_ * gridmap_width_,
                  std::numeric_limits<int>::max());
  parent_.resize(gridmap_height_ * gridmap_width_,
                 std::numeric_limits<int>::max());
  state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

  // costmap_ptr_->GetCostMap()->SetCost(start_x,
  // start_y,roborts_costmap::FREE_SPACE);
  std::priority_queue<int, std::vector<int>, Compare> openlist;
  g_score_.at(start_index) = 0;
  openlist.push(start_index);

  std::vector<int> neighbors_index;
  int current_index, move_cost, h_score, count = 0;

  while (!openlist.empty()) {
    current_index =
        openlist.top();  // 先将旧点记录，然后新点pop，这样两点生成一个direction
    openlist.pop();
    state_.at(current_index) = SearchState::CLOSED;

    if (current_index == goal_index) {
      // ROS_INFO("Search takes %d cycle counts", count);
      break;
    }

    GetNineNeighbors(current_index, neighbors_index);

    for (auto neighbor_index : neighbors_index) {
      if (neighbor_index < 0 ||  // 边界
          neighbor_index >= gridmap_height_ * gridmap_width_) {
        continue;
      }

      if (cost_[neighbor_index] >= inaccessible_cost_ ||  // 障碍物
          state_.at(neighbor_index) == SearchState::CLOSED) {
        continue;
      }

      GetMoveCost(current_index, neighbor_index, move_cost);

      if (g_score_.at(neighbor_index) >
          g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {
        g_score_.at(neighbor_index) =
            g_score_.at(current_index) + move_cost + cost_[neighbor_index];
        parent_.at(neighbor_index) = current_index;
        GetL2Distance(neighbor_index, goal_index, h_score);
        auto f_score = g_score_.at(neighbor_index) + h_score;
        if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
          // GetManhattanDistance(neighbor_index, goal_index, h_score);
          // f_score_.at(neighbor_index) = g_score_.at(neighbor_index) +
          // h_score;
          f_score_.at(neighbor_index) = f_score;
          openlist.push(neighbor_index);
          state_.at(neighbor_index) = SearchState::OPEN;
        }
        // else if (state_.at(neighbor_index) == SearchState::OPEN &&
        //            f_score < f_score_.at(neighbor_index)) {
        //   // GetL2Distance(neighbor_index, goal_index, h_score);
        //   f_score_.at(neighbor_index) = f_score;
        //   parent_.at(neighbor_index) = current_index;
        //   // openlist.push(neighbor_index);
        // }
      }
    }
    count++;
  }

  if (current_index != goal_index) {
    ROS_WARN("Global planner can't search the valid path!old");
    return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,
                     "Valid global path not found.");
  }

  unsigned int iter_index = current_index, iter_x, iter_y;

  geometry_msgs::PoseStamped iter_pos;
  iter_pos.pose.orientation.w = 1;
  iter_pos.header.frame_id = "map";
  path.clear();
  costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
  costmap_ptr_->GetCostMap()->Map2World(
      iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
  path.push_back(iter_pos);

  while (iter_index != start_index) {
    iter_index = parent_.at(iter_index);
    //    if(cost_[iter_index]>= inaccessible_cost_){
    //      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned
    //      int>(cost_[iter_index]);
    //    }
    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(
        iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);
  }

  std::reverse(path.begin(), path.end());
  // std::vector<geometry_msgs::PoseStamped> temppath;
  // // bool enoughflag=false;
  // int step = path.size() / 15 + 1;
  // if (step < 3) step = 3;
  // for (int i = 0; i < path.size(); i += step) {
  //   temppath.push_back(path[i]);
  //   // if(temppath.size()>20){
  //   //   enoughflag=true;
  //   //   break;
  //   // }
  // }
  // // if(!enoughflag)
  // temppath.push_back(path.back());

  // PathSmoother(temppath, path);

  carobs.clear();
  carobs.push_back(allypose_);
  for (auto iter = carKalContainer.begin(); iter != carKalContainer.end();
       iter++) {
    carobs.push_back(iter->GetPose());
  }
  // if(obs_car_state_[0]==1)carobs.push_back(enemy1.GetPose());
  // if(obs_car_state_[1]==1)carobs.push_back(enemy2.GetPose());
  // if(obs_car_state_[2]==1)carobs.push_back(allypose_);
  // printf("\n__carobsnum=%d__\n",carobs.size());
  // lastpathpub.publish(last_path);
  //////////////////////////////////////////////////new//////////////////////////////////////////////
  // test

  // testend
  // 检查是否前后帧跳变
  // 前后跳变原因
  // 先检查有无动态障碍物逼近，无就lastnew分析，有就分析速度反向
  if (!has_new_goal && !last_path.empty() && 0) {  // 没新目标点且有上一帧路径
    // printf("\n__homocheck__\n");
    double start_x_f, start_y_f, goal_x_f, goal_y_f, circle_x_f, circle_y_f,
        circle_r2_f;
    start_x_f = (*path.begin()).pose.position.x;
    start_y_f = (*path.begin()).pose.position.y;
    goal_x_f = (*path.end()).pose.position.x;
    goal_y_f = (*path.end()).pose.position.y;
    circle_x_f = (start_x_f + goal_x_f) / 2;
    circle_y_f = (start_y_f + goal_y_f) / 2;
    circle_r2_f =
        (std::abs(start_x_f - goal_x_f) * std::abs(start_x_f - goal_x_f) +
         std::abs(start_y_f - goal_y_f) * std::abs(start_y_f - goal_y_f)) /
        4;
    // 动态障碍物homo
    int carObsnum = -1;
    carObsnum =
        GetTheCarNeedToAvoid(carobs, path, start_x_f, start_y_f, goal_x_f,
                             goal_y_f, circle_x_f, circle_y_f, circle_r2_f);
    if (carObsnum != -1) {  // 存在最需要紧急避障的/给TEB提供跨越障碍物的初解
      unsigned int tempx, tempy;
      GetBackPoint(carobs[carObsnum], start_x_f, start_y_f, goal_x_f, goal_y_f,
                   tempx, tempy);
      std::vector<geometry_msgs::PoseStamped> new_path;
      // printf("\n__replandongtai__\n");
      if (replan(start_index, goal_index, tempx, tempy, new_path)) {
        // printf("\n__replandongtai_success__\n");
        path.assign(new_path.begin(), new_path.end());
      }
    } else {
      // 静态障碍物homo
      block_state.resize(9, 0);
      for (int k = 0; k < block_state.size(); ++k) {
        double xk_f, yk_f, rk2_f;
        costmap_ptr_->GetCostMap()->Map2World(block_fields_record[k].first,
                                              block_fields_record[k].second,
                                              xk_f, yk_f);
        rk2_f = std::abs(xk_f - circle_x_f) * std::abs(xk_f - circle_x_f) +
                std::abs(yk_f - circle_y_f) * std::abs(yk_f - circle_y_f);
        // printf("\n__B%drk2=%f__\n",k+1,rk2_f);
        if (rk2_f < circle_r2_f) {  // 障碍物在start和goal生成的圆当中
          block_state[k] = 1;
          printf("\n__B%din__\n", k + 1);
          // 针对在圆中的障碍物做跳变检测(同伦检测)
          std::pair<double, double> Vx =
              getVxstatic(xk_f, yk_f, start_x_f, start_y_f, goal_x_f, goal_y_f);
          std::pair<double, double> Vy =
              std::make_pair(-Vx.second, Vx.first);  // 合力正交方向，逆时针90度
          // 找到路径中与合力方向交叉的点/new和last/二分法
          int direction = 0, direction_last = 0;  // 1Vx,-1反Vx两种
          PathHomoCheck(path, direction, Vx, Vy, xk_f, yk_f);
          PathHomoCheck(last_path, direction_last, Vx, Vy, xk_f, yk_f);
          // printf("\n__B%d,dir=%d,dirlast=%d__,%d\n",k+1,direction,direction_last,direction*direction_last);
          if (direction * direction_last > 0) {         // 同向
                                                        //  normal
          } else if (direction * direction_last < 0) {  // 反向
            // 判断lastpath是不是因为穿过了某个动态障碍物导致的
            printf("\n__different__\n");
            if (PathObsCheck(last_path)) {  // 有
              printf("\n__lastpathhasobs__\n");
              // 用新的/动态避障/得看属于哪个动态障碍物/直接用最近的动态障碍物？
              for (int i = 0; i < carobs.size(); i++) {
                // 对三个车都进行//进行过之后path没改变就是因为有静态障碍物阻挡
                GetOutFromACar(path, carobs[i], start_x_f, start_y_f, goal_x_f,
                               goal_y_f);
              }
              // unsigned int temp1x,temp1y;
              // if(GetBackPoint(enemy1.GetPose(),start_x_f,start_y_f,goal_x_f,goal_y_f,temp1x,temp1y)){
              //     std::vector<geometry_msgs::PoseStamped> new_path;
              //     if(replan(start_index,goal_index,temp1x,temp1y,new_path)){
              //         path.assign(new_path.begin(),new_path.end());
              //     }
              // }
            } else {  // 无/跳变/采用上一帧的
              // path.assign(last_path.begin(),last_path.end());
              printf("\n__choose_last__\n");
              break;
            }
          } else {
            // error
            //  printf("\n\n__check_homo_error__\n\n");
          }
        }
      }
    }
  }
  // 选旧路就要告诉/并且不在teb中setplan
  return ErrorInfo(ErrorCode::OK);
}
int AStarPlanner::PathSmoother(
    std::vector<geometry_msgs::PoseStamped> &path,
    std::vector<geometry_msgs::PoseStamped> &smooth_path) {
  // ros::Time starttime=ros::Time::now();
  // path坐标打开
  int npath = path.size();
  Eigen::VectorXd x(npath * D_);
  for (int i = 0; i < npath; ++i) {
    x(i) = path[i].pose.position.x;
    x(i + npath) = path[i].pose.position.y;
  }
  // printf("\n__change path to x__\n");
  lbfgs::lbfgs_parameter_t params;
  params.g_epsilon = 1.0e-6;  // 精度高则慢，精度底轨迹质量一般
  params.past = 3;
  params.delta = 1.0e-6;
  double finalCost = -1;
  // printf("\n__startoptimal3__");
  // 提前计算梯度需用的矩阵
  // 将vec形式的x整合成矩阵形式
  int N = x.size() / D_;
  Eigen::MatrixXd X(N, D_);
  if (X.rows() < 3) {
    return -1;
  }
  for (int i = 0; i < D_; ++i) {
    X.col(i) = x.segment(i * N, N);
  }
  // printf("\n__startoptimal4__");
  unsigned int n = X.rows() - 1;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n + 1);
  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(n, n + 1);
  A1.middleCols(1, n).setIdentity(n, n);
  A = -A + A1;
  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(n - 1, n + 1);
  Eigen::MatrixXd B1 = Eigen::MatrixXd::Zero(n - 1, n + 1);
  B1.middleCols(2, n - 1).setIdentity(n - 1, n - 1);
  B = -B + B1;
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n - 1, n - 1);
  Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(n - 1, n - 1);
  Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(n - 1, n - 1);
  Eigen::MatrixXd Mu = Eigen::MatrixXd::Zero(n, n - 1);
  Eigen::MatrixXd Md = Eigen::MatrixXd::Zero(n, n - 1);
  M1.bottomLeftCorner(n - 2, n - 2).setIdentity(n - 2, n - 2);
  M2.topRightCorner(n - 2, n - 2).setIdentity(n - 2, n - 2);
  M = 4 * M + M1 + M2;
  M = M.inverse();
  Md.bottomRows(n - 1) = M;
  Mu.topRows(n - 1) = M;
  P_ = 3 * A - 6 * Md * B - 3 * Mu * B;
  Q_ = -2 * A + 3 * Md * B + 3 * Mu * B;
  // printf("\n__startoptimal5__");
  Eigen::MatrixXd PE = P_.middleCols(1, n - 1);
  Eigen::MatrixXd QE = Q_.middleCols(1, n - 1);
  Eigen::MatrixXd b = X;
  b.middleRows(1, n - 1).setZero();
  Eigen::MatrixXd Pb = P_ * b;
  Eigen::MatrixXd Qb = Q_ * b;
  // printf("\n__startoptimal6__");
  H_ = 8 * PE.transpose() * PE +
       12 * (PE.transpose() * QE + QE.transpose() * PE) +
       24 * QE.transpose() * QE;
  // printf("\n__startoptimal7__");
  h_ = 8 * PE.transpose() * Pb +
       12 * (PE.transpose() * Qb + QE.transpose() * Pb) +
       24 * QE.transpose() * Qb;
  // std::cout<<h_<<std::endl;
  // printf("\n__startoptimal__\n");
  // int ret=1;
  int ret = lbfgs::lbfgs_optimize(x, finalCost, smoothFunction, MonitorProgress,
                                  this, params);
  // printf("\n__optimize over__\n");
  // 优化的x还原回smoothpath
  smooth_path.resize(npath);
  for (int i = 0; i < npath; ++i) {
    smooth_path[i].pose.position.x = x(i);
    smooth_path[i].pose.position.y = x(i + npath);
  }
  // double usedtime = 1000*(ros::Time::now()-starttime).toSec();
  // std::cout << std::setprecision(4)
  //           << "================================" << std::endl
  //           << "L-BFGS Optimization Returned: " << ret << std::endl
  //           << "Minimized Cost: " << finalCost << std::endl
  //           << "Used time(ms): " << std::endl
  //           << usedtime << std::endl;
  // usedtime = 1000*(ros::Time::now()-starttime).toSec();
  // std::cout<<"Used time(ms): " << std::endl
  // << usedtime << std::endl;
  return ret;
}
// /*getDistBlocks(Obstacles_,rpose,0.3,grad,dist);*/
void AStarPlanner::getDistBlocks(
    const Eigen::Matrix<double, 10 * 4, 2> &Obstacles,
    const Eigen::Vector2d &pos, const double &safe_radius,
    Eigen::Matrix<double, 2, 10> &grad, Eigen::Matrix<double, 10, 1> &dist) {
  Eigen::Matrix<double, 2, 2> Q;
  Q << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 1> c;
  c << 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> x;
  Eigen::Matrix<double, -1, 2> A(4, 2);
  Eigen::VectorXd b(4);
  b << -1.0, -1.0, -1.0, -1.0;
  for (int i = 0; i < 10; ++i) {
    A = Obstacles.middleRows(i * 4, 4);
    for (int j = 0; j < A.rows(); ++j) {
      A.row(j) = pos.transpose() - A.row(j);
    }
    double minobj = sdqp::sdqp<2>(Q, c, A, b, x);
    Eigen::Vector2d z;
    if (std::isinf(minobj)) {  // 如果为inf则当前点在该障碍物内
      // A = -A;//vertex-x
      Eigen::Vector4d b_in;
      Eigen::Matrix<double, 2, 4> a_in;
      a_in.col(0) = Eigen::Vector2d(A(1, 1) - A(0, 1), A(0, 0) - A(1, 0));
      a_in.col(1) = Eigen::Vector2d(A(2, 1) - A(1, 1), A(1, 0) - A(2, 0));
      a_in.col(2) = Eigen::Vector2d(A(3, 1) - A(2, 1), A(2, 0) - A(3, 0));
      a_in.col(3) = Eigen::Vector2d(A(0, 1) - A(3, 1), A(3, 0) - A(0, 0));
      int idx = -1;
      b_in(0) =
          (A(0, 0) * (A(1, 1) - A(0, 1)) - A(0, 1) * (A(1, 0) - A(0, 0))) /
          a_in.col(0).norm();
      b_in(1) =
          (A(1, 0) * (A(2, 1) - A(1, 1)) - A(1, 1) * (A(2, 0) - A(1, 0))) /
          a_in.col(1).norm();
      b_in(2) =
          (A(2, 0) * (A(3, 1) - A(2, 1)) - A(2, 1) * (A(3, 0) - A(2, 0))) /
          a_in.col(2).norm();
      b_in(3) =
          (A(3, 0) * (A(0, 1) - A(3, 1)) - A(3, 1) * (A(0, 0) - A(3, 0))) /
          a_in.col(3).norm();
      // std::cout<<"A: "<<A<<std::endl;
      // std::cout<<"b: "<<b_in<<std::endl;
      if (i == 9) {  // 场地大矩形，在内距离为负，在外距离为正
        dist(i) = -b_in.cwiseAbs().minCoeff(&idx);
        Eigen::Vector2d g = a_in.col(idx);
        double gnorm = g.norm();
        g = g * dist(i) / gnorm;
        grad.col(i) = g / g.norm();
        // std::cout<<"g: "<<g<<std::endl;
        // z = g + rpose;
      } else {
        dist(i) = b_in.cwiseAbs().minCoeff(&idx);
        Eigen::Vector2d g = a_in.col(idx);  // 距离内正外负
        double gnorm = g.norm();
        g = g * dist(i) / gnorm;  // change
        grad.col(i) = g / g.norm();
        // std::cout<<"g: "<<g<<std::endl;
        // z = g + rpose;
      }
    } else {
      double xx = x.dot(x);
      z = x / xx;

      if (i == 9) {
        grad.col(i) = -z / z.norm();
        dist(i) = (x / xx).norm();
        // z+=rpose;
      } else {
        grad.col(i) = z / z.norm();
        dist(i) = -(x / xx).norm();
        // z+=rpose;
      }
    }
    dist(i) += safe_radius;
    if (dist(i) < 0) {
      grad.col(i) = c;  // 置0
      dist(i) = 0;
    }
    // std::cout<<"B"<<i+1<<": "<<"point:"<<z.transpose()<<" "<<"dist:
    // "<<dist(i)<<"minobj: "<<minobj<<"g:
    // "<<grad.col(i).transpose()<<std::endl;
  }
}
int AStarPlanner::GetTheCarNeedToAvoid(
    std::vector<nav_msgs::Odometry> &carobs,
    std::vector<geometry_msgs::PoseStamped> &path, double start_x_f,
    double start_y_f, double goal_x_f, double goal_y_f, double circle_x_f,
    double circle_y_f, double circle_r2_f) {
  double time_min = std::numeric_limits<double>::max();
  int i_min = -1;
  for (int i = 0; i < carobs.size(); ++i) {
    double xo_f, yo_f, Roc2_f, Ros2_f;  // 障碍物
    xo_f = carobs[i].pose.pose.position.x;
    yo_f = carobs[i].pose.pose.position.y;
    Roc2_f = std::abs(xo_f - circle_x_f) * std::abs(xo_f - circle_x_f) +
             std::abs(yo_f - circle_y_f) * std::abs(yo_f - circle_y_f);
    Ros2_f = std::abs(xo_f - start_x_f) * std::abs(xo_f - start_x_f) +
             std::abs(yo_f - start_y_f) * std::abs(yo_f - start_y_f);
    if (Roc2_f < circle_r2_f && Ros2_f < circle_r2_f) {  // 在两个圆交集中
      // 进行时间判断
      double time = ObsThreatCheck(path, start_x_f, start_y_f, goal_x_f,
                                   goal_y_f, carobs[i]);
      if (time < time_min) {
        time_min = time;
        i_min = i;
      }
    }
  }
  return i_min;
}
bool AStarPlanner::GetBackPoint(
    nav_msgs::Odometry pose, double start_x_f, double start_y_f,
    double goal_x_f, double goal_y_f, unsigned int &tempx,
    unsigned int &tempy) {  // 不应该找太远的/远的就不要了当作失败
  int direction;
  bool slow = false;
  std::pair<double, double> Vx =
      getVx(pose, start_x_f, start_y_f, goal_x_f, goal_y_f,
            slow);  // 速度很小就不用在后边找点了
  std::pair<double, double> Vback;
  Vback.first = -Vx.first;
  Vback.second = -Vx.second;
  unsigned int obsx, obsy;
  // unsigned int tempx,tempy;
  bool gettemp = false;
  costmap_ptr_->GetCostMap()->World2Map(pose.pose.pose.position.x,
                                        pose.pose.pose.position.y, obsx, obsy);
  if (std::fabs(Vback.first) >
      std::fabs(Vback.second)) {  // 速度靠近x轴，也就是y轴速度可能等于0
    double k = Vback.second / Vback.first;
    int distance = 0;
    if (Vback.first > 0) {
      for (tempx = obsx; tempx < 160; ++tempx) {
        ++distance;
        tempy = obsy + k * static_cast<int>(tempx - obsx);
        if (costmap_ptr_->GetCostMap()->GetCost(tempx, tempy) <
            inaccessible_cost_) {
          // 找到点了
          gettemp = true;
          break;
        }
        if (distance > 20) {
          return false;
        }
      }
    } else {
      for (tempx = obsx; tempx > 0; --tempx) {
        ++distance;
        tempy = obsy - k * static_cast<int>(obsx - tempx);
        if (costmap_ptr_->GetCostMap()->GetCost(tempx, tempy) <
            inaccessible_cost_) {
          // 找到点了
          gettemp = true;
          break;
        }
        if (distance > 20) {  // 1m/距离过远失败
          return false;
        }
      }
    }
  } else {  // 速度靠近y轴，也就是x轴速度有可能等于0
    double k = Vback.first / Vback.second;
    int distance = 0;
    if (Vback.second > 0) {
      for (tempy = obsy; tempy < 90; ++tempy) {
        ++distance;
        tempx = obsx + k * static_cast<int>(tempy - obsy);
        if (costmap_ptr_->GetCostMap()->GetCost(tempx, tempy) <
            inaccessible_cost_) {
          // 找到点了
          gettemp = true;
          break;
        }
        if (distance > 20) {
          return false;
        }
      }
    } else {
      for (tempy = obsy; tempy > 0; --tempy) {
        ++distance;
        tempx = obsx - k * static_cast<int>(obsy - tempy);
        if (costmap_ptr_->GetCostMap()->GetCost(tempx, tempy) <
            inaccessible_cost_) {
          // 找到点了
          gettemp = true;
          break;
        }
        if (distance > 20) {
          return false;
        }
      }
    }
  }
  return gettemp;
}
bool AStarPlanner::GetFrontPoint(nav_msgs::Odometry pose, double start_x_f,
                                 double start_y_f, double goal_x_f,
                                 double goal_y_f, unsigned int &tempx,
                                 unsigned int &tempy) {}
bool AStarPlanner::replan(const int &start_index, const int &goal_index,
                          unsigned int &tempx, unsigned int &tempy,
                          std::vector<geometry_msgs::PoseStamped> &path) {
  // 两次Astar连接
  int start_i = start_index;
  int goal_i = goal_index;
  std::vector<geometry_msgs::PoseStamped> path1;
  std::vector<geometry_msgs::PoseStamped> path2;
  unsigned int startx, starty, goalx, goaly;
  int temp_index = costmap_ptr_->GetCostMap()->GetIndex(tempx, tempy);
  costmap_ptr_->GetCostMap()->Index2Cells(start_index, startx, starty);
  costmap_ptr_->GetCostMap()->Index2Cells(goal_index, goalx, goaly);
  printf("\n__useastar_replan__\n");
  if (Astar(start_i, temp_index, path1) &&
      Astar(temp_index, goal_i, path2)) {  // 可能astar会出错//找不到路径
    printf("\n__useastar_replan_success__\n");
    path1.pop_back();  // tempindex没找到就修改
    path1.insert(path1.end(), path2.begin(), path2.end());
    path.assign(path1.begin(), path1.end());
    return true;
  }
  return false;
}
bool AStarPlanner::GetOutFromACar(std::vector<geometry_msgs::PoseStamped> &path,
                                  nav_msgs::Odometry pose, double start_x_f,
                                  double start_y_f, double goal_x_f,
                                  double goal_y_f) {  // 其实只要Astar即可
  bool flag = false;
  unsigned int i = 0;
  unsigned int i_start = 0, i_goal = 0;
  // for(i = 1;i<path.size();++i){
  //     unsigned int x,y;
  //     // unsigned int x,y;
  //         costmap_ptr_->GetCostMap()->World2Map(path[i].pose.position.x,path[i].pose.position.y,x,y);
  //         // unsigned int index=costmap_ptr_->GetCostMap()->GetIndex(x,y);
  //     // costmap_ptr_->GetCostMap()->Index2Cells(index,x,y);
  //     if(costmap_ptr_->GetCostMap()->GetCost(x, y)>inaccessible_cost_){
  //         i_start=i-1;
  //         break;
  //     }
  // }
  printf("\npathsize%d\n", path.size());
  for (i = 0; i < path.size(); ++i) {
    if (getDistanceinGeo(path[i], pose) < 1 &&
        !flag) {  // 第一次与障碍物交叉//cost值判断
      i_start = i;
      flag = true;
    }
    if (getDistanceinGeo(path[i], pose) >= 1 && flag) {  // 交叉后又分离
      i_goal = i;
      break;
    }
  }
  for (i = i_start; i > 0; --i) {  // 确保起止点都是可通行的
    unsigned int x, y;
    // unsigned int x,y;
    costmap_ptr_->GetCostMap()->World2Map(path[i].pose.position.x,
                                          path[i].pose.position.y, x, y);
    // unsigned int index=costmap_ptr_->GetCostMap()->GetIndex(x,y);
    // costmap_ptr_->GetCostMap()->Index2Cells(index,x,y);
    if (costmap_ptr_->GetCostMap()->GetCost(x, y) < inaccessible_cost_) {
      i_start = i;
      break;
    }
  }
  for (i = i_goal; i < path.size(); ++i) {
    unsigned int x, y;
    costmap_ptr_->GetCostMap()->World2Map(path[i].pose.position.x,
                                          path[i].pose.position.y, x, y);
    // costmap_ptr_->GetCostMap()->Index2Cells(i,x,y);
    if (costmap_ptr_->GetCostMap()->GetCost(x, y) < inaccessible_cost_) {
      i_goal = i;
      break;
    }
  }
  // 得到起止点了/判断起止点合适与否？
  if (i_start != 0 && i_goal - i_start > 3) {  // 有一定交叉
    // unsigned int tempx,tempy;
    int index_start = 0, index_goal = 0;
    // if(GetBackPoint(pose,start_x_f,start_y_f,goal_x_f,goal_y_f,tempx,tempy)){//应该用GetFrontPoint
    unsigned int x, y;
    costmap_ptr_->GetCostMap()->World2Map(path[i_start].pose.position.x,
                                          path[i_start].pose.position.y, x, y);
    printf("start(%d,%d)", x, y);
    index_start = costmap_ptr_->GetCostMap()->GetIndex(x, y);
    costmap_ptr_->GetCostMap()->World2Map(path[i_goal].pose.position.x,
                                          path[i_goal].pose.position.y, x, y);
    printf("goal(%d,%d)", x, y);
    index_goal = costmap_ptr_->GetCostMap()->GetIndex(x, y);
    std::vector<geometry_msgs::PoseStamped> temp_path;
    // printf("\n__replan_getoutfromcar__\n");
    // if(replan(index_start,index_goal,tempx,tempy,temp_path)){
    // printf("\n__replan_getoutfromcar_success__\n");
    // printf("\nindex s\t%d,e\t%d\n",index_start,index_goal);
    // i_start_test=index_start;//map
    // i_goal_test=index_goal;
    // pathreplanstart=i_start;//path
    // pathreplanend=i_goal;
    // if(Astar(index_start,index_goal,temp_path)){
    //     path.erase(path.begin()+i_start,path.begin()+i_goal+1);//擦除这段交叉的路径
    //     path.insert(path.begin()+i_start,temp_path.begin(),temp_path.end());//插入新生成的无碰撞路径
    //     return true;
    // }
    // }
    // }
    return false;  // 路径重搜索失败了
  }                // 没交叉，原路径就可以
  return true;
}
bool AStarPlanner::Astar(int &start_index, int &goal_index,
                         std::vector<geometry_msgs::PoseStamped> &path) {
  // haha
  //  unsigned int goal_x,goal_y,tmp_goal_x, tmp_goal_y;
  //  unsigned  int shortest_dist = std::numeric_limits<unsigned int>::max();
  //  costmap_ptr_->GetCostMap()->Index2Cells(goal_index,goal_x,goal_y);
  //  if
  //  (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)>=inaccessible_cost_){
  //      tmp_goal_x = goal_x;
  //      tmp_goal_y = goal_y - goal_search_tolerance_;
  //      while(tmp_goal_y <= goal_y + goal_search_tolerance_){
  //          tmp_goal_x = goal_x - goal_search_tolerance_;
  //          while(tmp_goal_x <= goal_x + goal_search_tolerance_){
  //              unsigned char cost =
  //              costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
  //              unsigned int dist = abs(static_cast<int>(goal_x - tmp_goal_x))
  //              + abs(static_cast<int>(goal_y - tmp_goal_y)); if (cost <
  //              inaccessible_cost_ && dist < shortest_dist ) {
  //                  shortest_dist = dist;
  //                  goal_x = tmp_goal_x;
  //                  goal_y = tmp_goal_y;
  //              }
  //              tmp_goal_x += 1;
  //          }
  //          tmp_goal_y += 1;
  //      }
  //  }
  //  goal_index=costmap_ptr_->GetCostMap()->GetIndex(goal_x,
  //  goal_y);//更改了目标点位置
  std::vector<int> f_score;
  std::vector<int> g_score;
  std::vector<int> h_score;
  std::vector<int> parent;
  std::vector<AStarPlanner::SearchState> state;
  // unsigned int gridmap_height;
  // unsigned int gridmap_width;
  // gridmap_width = costmap_ptr_->GetCostMap()->GetSizeXCell();
  // gridmap_height = costmap_ptr_->GetCostMap()->GetSizeYCell();
  unsigned char *cost;
  cost = costmap_ptr_->GetCostMap()->GetCharMap();
  // int bx,by,dx,dy;

  //             int x_=x;int y_=y;
  //             bx=std::min(x_-10,0);
  //             by=std::min(y_-10,0);
  //             dx=std::max(x_+10,165);
  //             dy=std::max(y_+10,165);
  //             unsigned int temp_idx = 0;
  //             for(int i = bx ; i < dx; i ++){//在以车中心为中心的正方形中搜索
  //                 for(int j = by ; j < dy; j ++){
  //                     temp_idx = i*166 + j;
  //                     if(temp_idx < 166*94){
  //                         if(PoseInsideCircle(x,y,j,i,circle_radius)){//加入圆形的障碍
  //                             carpoint temp_={j,i};//x,y
  //                             car_obs_points[k].push_back(temp_);
  //                             costmap_[temp_idx] = car_obs_val;
  //                             // printf("\nshowenemy\n");
  //                         }
  //                     }
  //                 }
  //             }
  g_score.resize(gridmap_height_ * gridmap_width_,
                 std::numeric_limits<int>::max());
  f_score.resize(gridmap_height_ * gridmap_width_,
                 std::numeric_limits<int>::max());
  parent.resize(gridmap_height_ * gridmap_width_,
                std::numeric_limits<int>::max());
  state.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);
  std::priority_queue<int, std::vector<int>, Compare> openlist;
  g_score.at(start_index) = 0;
  openlist.push(start_index);
  std::vector<int> neighbors_index;
  int current_index, move_cost, hscore, count = 0;
  while (!openlist.empty()) {
    current_index =
        openlist.top();  // 先将旧点记录，然后新点pop，这样两点生成一个direction
    openlist.pop();
    state.at(current_index) = SearchState::CLOSED;

    if (current_index == goal_index) {
      // ROS_INFO("Search takes %d cycle counts", count);
      break;
    }

    GetNineNeighbors(current_index, neighbors_index);

    for (auto neighbor_index : neighbors_index) {
      if (neighbor_index < 0 ||  // 边界
          neighbor_index >= gridmap_height_ * gridmap_width_) {
        continue;
      }

      if (cost[neighbor_index] >= inaccessible_cost_ ||  // 障碍物
          state.at(neighbor_index) == SearchState::CLOSED) {
        continue;
      }

      GetMoveCost(current_index, neighbor_index, move_cost);

      if (g_score.at(neighbor_index) >
          g_score.at(current_index) + move_cost + cost[neighbor_index]) {
        g_score.at(neighbor_index) =
            g_score.at(current_index) + move_cost + cost[neighbor_index];
        parent.at(neighbor_index) = current_index;
        GetL2Distance(neighbor_index, goal_index, hscore);
        auto fscore = g_score.at(neighbor_index) + hscore;
        if (state.at(neighbor_index) == SearchState::NOT_HANDLED) {
          // GetManhattanDistance(neighbor_index, goal_index, h_score);
          // f_score_.at(neighbor_index) = g_score_.at(neighbor_index) +
          // h_score;
          f_score.at(neighbor_index) = fscore;
          openlist.push(neighbor_index);
          state.at(neighbor_index) = SearchState::OPEN;
          // parent.at(neighbor_index) = current_index;
        } else if (state.at(neighbor_index) == SearchState::OPEN &&
                   fscore < f_score.at(neighbor_index)) {
          // GetL2Distance(neighbor_index, goal_index, h_score);
          f_score.at(neighbor_index) = fscore;
          // parent.at(neighbor_index) = current_index;
          // openlist.push(neighbor_index);
        }
      }
    }
    count++;
  }
  if (current_index != goal_index) {
    ROS_WARN("Global planner can't search the valid path!");
    return false;
  }
  unsigned int iter_index = current_index, iter_x, iter_y;

  geometry_msgs::PoseStamped iter_pos;
  iter_pos.pose.orientation.w = 1;
  iter_pos.header.frame_id = "map";
  path.clear();
  costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
  costmap_ptr_->GetCostMap()->Map2World(
      iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
  path.push_back(iter_pos);

  while (iter_index != start_index) {
    iter_index = parent.at(iter_index);
    //    if(cost_[iter_index]>= inaccessible_cost_){
    //      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned
    //      int>(cost_[iter_index]);
    //    }
    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(
        iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);
  }

  std::reverse(path.begin(), path.end());
  return true;
}
std::pair<double, double> AStarPlanner::getVx(nav_msgs::Odometry pose,
                                              double start_x_f,
                                              double start_y_f, double goal_x_f,
                                              double goal_y_f, bool &slow) {
  std::pair<double, double> Vx = std::make_pair(
      pose.twist.twist.linear.x, pose.twist.twist.linear.y);  // 速度方向
  double vel = sqrt(Vx.first * Vx.first + Vx.second * Vx.second);
  if (vel < 0.3) {  // 速度较低，加个死区，也是减少速度跟踪的滞后影响
    slow = true;
    return getVxstatic(pose.pose.pose.position.x, pose.pose.pose.position.y,
                       start_x_f, start_y_f, goal_x_f, goal_y_f);
  }
  Vx.first /= vel;
  Vx.second /= vel;
  return Vx;
}
std::pair<double, double> AStarPlanner::getVxstatic(
    double obs_x_f, double obs_y_f, double start_x_f, double start_y_f,
    double goal_x_f, double goal_y_f) {
  std::pair<double, double> Vs = std::make_pair(
      start_x_f - obs_x_f, start_y_f - obs_y_f);  // 障碍物中心指向起点
  Vs.first /= sqrt(Vs.first * Vs.first + Vs.second * Vs.second);
  Vs.second /= sqrt(Vs.first * Vs.first + Vs.second * Vs.second);
  std::pair<double, double> Vg = std::make_pair(
      goal_x_f - obs_x_f, goal_y_f - obs_y_f);  // 障碍物中心指向终点
  Vg.first /= sqrt(Vg.first * Vg.first + Vg.second * Vg.second);
  Vg.second /= sqrt(Vg.first * Vg.first + Vg.second * Vg.second);
  std::pair<double, double> Vx;
  // std::pair<double,double> Vy;
  if ((Vs.first * Vg.first + Vs.second * Vg.second) + 1 <
      0.04) {  // cos+1/两个向量基本反向/175度
    Vx = std::make_pair(-Vg.second, Vg.first);  // 合力令与Vg正交
    // Vy = std::make_pair(-Vx.second,Vx.first);//合力正交方向，逆时针90度
  } else {
    Vx = std::make_pair(Vs.first + Vg.first, Vs.second + Vg.second);  // 合力
    Vx.first /= sqrt(Vx.first * Vx.first + Vx.second * Vx.second);
    Vx.second /= sqrt(Vx.first * Vx.first + Vx.second * Vx.second);
    // Vy = std::make_pair(-Vx.second,Vx.first);//合力正交方向，逆时针90度
  }
  return Vx;
}
bool AStarPlanner::PathObsCheck(std::vector<geometry_msgs::PoseStamped> &path)
    const {  // 给出哪一个动态障碍物？/动态静态都可能
  bool check = false;
  unsigned int x, y;
  unsigned char cost;
  int count = 0;
  for (auto &it : path) {
    costmap_ptr_->GetCostMap()->World2Map(it.pose.position.x,
                                          it.pose.position.y, x, y);
    cost = costmap_ptr_->GetCostMap()->GetCost(x, y);
    if (cost > inaccessible_cost_) {
      ++count;
    }
    if (count > 3) {
      check = true;
      break;
    }
  }
  return check;
}
double AStarPlanner::ObsThreatCheck(
    std::vector<geometry_msgs::PoseStamped> &path, double start_x_f,
    double start_y_f, double goal_x_f, double goal_y_f,
    nav_msgs::Odometry pose) {
  int direction;
  bool slow = false;
  double distance, vel, time = std::numeric_limits<double>::max();
  std::pair<double, double> Vx =
      getVx(pose, start_x_f, start_y_f, goal_x_f, goal_y_f, slow);
  std::pair<double, double> Vy = std::make_pair(-Vx.second, Vx.first);
  if (!slow) {
    int index =
        PathHomoCheck(path, direction, Vx, Vy, pose.pose.pose.position.x,
                      pose.pose.pose.position.y);
    if (index >= 0 && direction == 1) {
      distance = getDistanceinGeo(path[index], pose);
      vel = sqrt(pose.twist.twist.linear.x * pose.twist.twist.linear.x +
                 pose.twist.twist.linear.y * pose.twist.twist.linear.y);
      time = distance / vel;
    }
  }
  return time;
}
int AStarPlanner::PathHomoCheck(std::vector<geometry_msgs::PoseStamped> &path,
                                int &direction, std::pair<double, double> Vx,
                                std::pair<double, double> Vy, double xk_f,
                                double yk_f) {  // 障碍物坐标方向
  int Vu_index = path.size();
  int Vd_index = 0;
  int V_index = path.size() / 2;
  unsigned int V_x, V_y;
  double V_x_f, V_y_f;
  std::pair<double, double> V;
  int count = 0;
  // printf("\n__Vx=(%f,%f),Vy=(%f,%f)__\n",Vx.first,Vx.second,Vy.first,Vy.second);
  while (ros::ok()) {
    V_x_f = path[V_index].pose.position.x;
    V_y_f = path[V_index].pose.position.y;
    // costmap_ptr_->GetCostMap()->Index2Cells(V_index, V_x, V_y);
    // costmap_ptr_->GetCostMap()->Map2World(V_x, V_y, V_x_f, V_y_f);
    V = std::make_pair(V_x_f - xk_f, V_y_f - yk_f);
    V.first /= sqrt(V.first * V.first + V.second * V.second);
    V.second /= sqrt(V.first * V.first + V.second * V.second);
    double VVx = V.first * Vx.first + V.second * Vx.second;
    double VVy = V.first * Vy.first + V.second * Vy.second;
    // printf("\n__Vindex=%d,VVx=%f,VVy=%f__\n",V_index,VVx,VVy);
    if (1 - std::abs(VVx) <= 0.03 &&
        std::abs(VVy) <= 0.03) {  // 收敛到Vx直线上/上下各5～6度
      if (VVx > 0) {              // Vx
        ++direction;              // 1
        return V_index;
      } else if (VVx < 0) {  //-Vx
        --direction;         //-1
        return V_index;
      }
    }
    if (VVy > 0) {  // 1,2
      Vd_index = V_index;
    } else if (VVy < 0) {  // 3,4
      Vu_index = V_index;
    }
    // else if(VVx>0&&VVy>0){//1
    //     Vd_index = V_index;
    // }else if(VVx<0&&VVy>0){//2
    //     Vd_index = V_index;
    // }else if(VVx<0&&VVy<0){//3
    //     Vu_index = V_index;
    // }else if(VVx>0&&VVy<0){//4
    //     Vu_index = V_index;
    // }

    V_index = (Vu_index + Vd_index) / 2;
    if (++count > 10) {  // 防止死循环
      // printf("\n\n__deadloop__\n\n");
      if (VVx > 0) {  // Vx
        ++direction;  // 1
        return V_index;
      } else if (VVx < 0) {  //-Vx
        --direction;         //-1
        return V_index;
      } else {
        // printf("\n\n__check_homo_error1__\n\n");
        return -1;  // 0,error
      }
    }
  }
  return V_index;
}
ErrorInfo AStarPlanner::GetMoveCost(const int &current_index,
                                    const int &neighbor_index,
                                    int &move_cost)
    const {  // 可加上对方向的判断让搜索尽量向前，有一个来到这一点的方向进行对比，加一个direction的参数//代表速度方向不易改变
  if (abs(neighbor_index - current_index) == 1 ||
      abs(neighbor_index - current_index) == gridmap_width_) {
    move_cost = 10;
  } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
             abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
    move_cost = 14;
  } else {
    return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                     "Move cost can't be calculated cause current neighbor "
                     "index is not accessible");
  }
  return ErrorInfo(ErrorCode::OK);
}
void AStarPlanner::GetL2Distance(const int &index1, const int &index2,
                                 int &L2_distance) const {
  double a =
      abs(static_cast<int>(index1 / gridmap_width_ - index2 / gridmap_width_));
  double b =
      abs(static_cast<int>(index1 % gridmap_width_ - index2 % gridmap_width_));
  L2_distance = heuristic_factor_ * 10 * sqrt(a * a + b * b);
  // L2_distance =
  //     heuristic_factor_ * 10 * (abs(a - b) + sqrt(2) * (a > b ? a : b));
}
void AStarPlanner::GetManhattanDistance(const int &index1, const int &index2,
                                        int &manhattan_distance) const {
  manhattan_distance = heuristic_factor_ * 10 *
                       (abs(static_cast<int>(index1 / gridmap_width_ -
                                             index2 / gridmap_width_)) +
                        abs(static_cast<int>((index1 % gridmap_width_ -
                                              index2 % gridmap_width_))));
}
double AStarPlanner::getDistanceinGeo(const geometry_msgs::PoseStamped &pose1,
                                      const nav_msgs::Odometry &pose2) {
  return hypot(pose1.pose.position.x - pose2.pose.pose.position.x,
               pose1.pose.position.y - pose2.pose.pose.position.y);
}
void AStarPlanner::GetNineNeighbors(const int &current_index,
                                    std::vector<int> &neighbors_index) const {
  neighbors_index.clear();
  if (current_index - gridmap_width_ >= 0) {
    neighbors_index.push_back(current_index - gridmap_width_);  // up
  }
  if (current_index - gridmap_width_ - 1 >= 0 &&
      (current_index - gridmap_width_ - 1 + 1) % gridmap_width_ != 0) {
    neighbors_index.push_back(current_index - gridmap_width_ - 1);  // left_up
  }
  if (current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_ != 0) {
    neighbors_index.push_back(current_index - 1);  // left
  }
  if (current_index + gridmap_width_ - 1 < gridmap_width_ * gridmap_height_ &&
      (current_index + gridmap_width_ - 1 + 1) % gridmap_width_ != 0) {
    neighbors_index.push_back(current_index + gridmap_width_ - 1);  // left_down
  }
  if (current_index + gridmap_width_ < gridmap_width_ * gridmap_height_) {
    neighbors_index.push_back(current_index + gridmap_width_);  // down
  }
  if (current_index + gridmap_width_ + 1 < gridmap_width_ * gridmap_height_ &&
      (current_index + gridmap_width_ + 1) % gridmap_width_ != 0) {
    neighbors_index.push_back(current_index + gridmap_width_ +
                              1);  // right_down
  }
  if (current_index + 1 < gridmap_width_ * gridmap_height_ &&
      (current_index + 1) % gridmap_width_ != 0) {
    neighbors_index.push_back(current_index + 1);  // right
  }
  if (current_index - gridmap_width_ + 1 >= 0 &&
      (current_index - gridmap_width_ + 1) % gridmap_width_ != 0) {
    neighbors_index.push_back(current_index - gridmap_width_ + 1);  // right_up
  }
}

}  // namespace roborts_global_planner
