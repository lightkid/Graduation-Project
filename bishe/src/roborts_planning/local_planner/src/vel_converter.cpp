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

#include <geometry_msgs/PoseStamped.h>  //new
#include <geometry_msgs/PoseStamped.h>

#include <chrono>
#include <mutex>
#include <thread>

#include "geometry_msgs/Twist.h"
#include "roborts_msgs/TwistAccel.h"
#include "ros/ros.h"

class VelConverter {
 public:
  VelConverter() : new_cmd_acc_(false), begin_(false) {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.angular.z = 0;

    cmd_pub_ = cmd_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    cmd_sub_ = cmd_handle_.subscribe<roborts_msgs::TwistAccel>(
        "cmd_vel_acc", 100, boost::bind(&VelConverter::VelCallback, this, _1));
    pos_sub_ =
        cmd_handle_.subscribe<geometry_msgs::PoseStamped>(  // fake locolization
            "amcl_pose", 2, boost::bind(&VelConverter::PosCallback, this, _1));
  }
  void VelCallback(const roborts_msgs::TwistAccel::ConstPtr& msg);
  void UpdateVel();
  void PosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 private:
  roborts_msgs::TwistAccel cmd_vel_acc_;
  geometry_msgs::Twist cmd_vel_;

  bool new_cmd_acc_, begin_;

  ros::NodeHandle cmd_handle_;
  ros::Publisher cmd_pub_;
  ros::Subscriber cmd_sub_;

  ros::Subscriber pos_sub_;
  double selfAngle = 0;
  double targetAngle = 0;
  double deltaAngle = 0;
  double deltaAnglelast = 0;

  std::chrono::high_resolution_clock::time_point time_begin_;

  std::mutex cmd_mutex_;
};

void VelConverter::VelCallback(
    const roborts_msgs::TwistAccel::ConstPtr& twist_acc_msg) {
  if (!begin_) {
    begin_ = true;
  }
  std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
  new_cmd_acc_ = true;
  cmd_vel_acc_ = *twist_acc_msg;
}
void VelConverter::PosCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // pos_=*msg;

  // double x=pos_.pose.orientation.x;
  double x1 = (*msg).pose.orientation.x;
  double y1 = (*msg).pose.orientation.y;
  double z1 = (*msg).pose.orientation.z;
  double w1 = (*msg).pose.orientation.w;
  double selfAngle1 = std::atan2(2 * (w1 * z1 + x1 * y1),
                                 1 - 2 * (y1 * y1 + z1 * z1));  // 偏航角
  while (selfAngle1 > M_PI) selfAngle1 -= 2 * M_PI;
  while (selfAngle1 < -M_PI) selfAngle1 += 2 * M_PI;
  selfAngle = selfAngle1;
  // printf("\nupdatepos(%f,%f)\n",x1,y1);
  targetAngle =
      std::atan2(2.25 - (*msg).pose.position.y,
                 4.05 - (*msg).pose.position.x);  // 0～PI//一直对准场地中心
  // targetAngle=0;
  // printf("\ntargetangle:%f\n",targetAngle);
}
void VelConverter::UpdateVel() {
  if (!begin_) {
    return;
  }
  // PD控制角速度
  targetAngle = 0;
  deltaAngle = targetAngle - selfAngle;
  while (deltaAngle > M_PI) deltaAngle -= 2 * M_PI;
  while (deltaAngle < -M_PI) deltaAngle += 2 * M_PI;

  double delta_error = deltaAngle - deltaAnglelast;
  while (delta_error > M_PI) delta_error -= 2 * M_PI;
  while (delta_error < -M_PI) delta_error += 2 * M_PI;

  double omega = 1.5 * deltaAngle + 5 * delta_error;
  if (omega > 3) omega = 3;
  if (omega < -3) omega = -3;
  cmd_vel_.angular.z = omega;
  deltaAnglelast = deltaAngle;

  auto begin = std::chrono::high_resolution_clock::now();
  if (new_cmd_acc_) {
    std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
    // cmd_vel_ = cmd_vel_acc_.twist;
    auto cmdVelAccHistory = cmd_vel_acc_;
    cmdVelAccHistory.twist.linear.x =
        cmdVelAccHistory.twist.angular.z * cmdVelAccHistory.twist.linear.x;
    cmdVelAccHistory.twist.linear.y =
        cmdVelAccHistory.twist.angular.z * cmdVelAccHistory.twist.linear.y;
    cmd_vel_.linear.x = cmdVelAccHistory.twist.linear.x * cos(-selfAngle) -
                        cmdVelAccHistory.twist.linear.y * sin(-selfAngle);
    cmd_vel_.linear.y = cmdVelAccHistory.twist.linear.x * sin(-selfAngle) +
                        cmdVelAccHistory.twist.linear.y * cos(-selfAngle);
    cmd_pub_.publish(cmd_vel_);
    new_cmd_acc_ = false;
    time_begin_ = std::chrono::high_resolution_clock::now();
    return;
  }
  auto actual_time =
      std::chrono::duration<double, std::ratio<1, 1>>(
          std::chrono::high_resolution_clock::now() - time_begin_)
          .count();
  time_begin_ = std::chrono::high_resolution_clock::now();

  cmd_vel_.linear.x =
      cmd_vel_.linear.x + actual_time * cmd_vel_acc_.accel.linear.x;
  cmd_vel_.linear.y =
      cmd_vel_.linear.y + actual_time * cmd_vel_acc_.accel.linear.y;
  // cmd_vel_.angular.z = cmd_vel_.angular.z + actual_time *
  // cmd_vel_acc_.accel.angular.z;

  cmd_pub_.publish(cmd_vel_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vel_converter");

  VelConverter vel_converter;

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ros::spinOnce();
    vel_converter.UpdateVel();
  }

  return 0;
}