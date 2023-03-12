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

#include "local_planner/local_visualization.h"

namespace roborts_local_planner {
LocalVisualization::LocalVisualization() : initialized_(false){

}
LocalVisualization::LocalVisualization(ros::NodeHandle &nh, const std::string &visualize_frame) : initialized_(false){
  Initialization(nh, visualize_frame);
}
void LocalVisualization::Initialization(ros::NodeHandle &nh, const std::string &visualize_frame) {
  if (initialized_) {

  }

  visual_frame_ = visualize_frame;
  local_planner_ = nh.advertise<nav_msgs::Path>("trajectory", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("trajectory_pose", 1);
  marker_pub_=nh.advertise<visualization_msgs::Marker>("trajectory_marker",1);
  marker_array_pub_=nh.advertise<visualization_msgs::MarkerArray>("trajectory_array",1);
  initialized_ = true;
  if(nh.getNamespace().c_str()!="/"&&nh.getNamespace().size()>5)//如果是多机器人，默认的命名空间就是/robot_x不是/
  {
    if(nh.getNamespace().substr(1) == "robot_0")
    {
      color=0;
    }
    if(nh.getNamespace().substr(1) == "robot_1")
    {
      color=1;
    }
  }
}

void LocalVisualization::PublishLocalPlan(const TebVertexConsole& vertex_console) const{
  visualization_msgs::Marker marker;
  marker.header.frame_id="map";
  marker.header.stamp=ros::Time::now();
  marker.ns="trajectory";
  marker.id=0;
  marker.type=visualization_msgs::Marker::CUBE_LIST;
  marker.action=visualization_msgs::Marker::ADD;
  marker.scale.x = 0.45;
  marker.scale.y = 0.60;
  marker.scale.z = 0.23;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.7;
  marker.lifetime = ros::Duration();

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();
  visualization_msgs::Marker bbox_marker;
  bbox_marker.header.frame_id = "map";
  bbox_marker.header.stamp = ros::Time::now();
  bbox_marker.ns = "basic_shapes";
  bbox_marker.lifetime = ros::Duration();
  bbox_marker.frame_locked = true;
  bbox_marker.type = visualization_msgs::Marker::CUBE;
  bbox_marker.action = visualization_msgs::Marker::ADD;
  int marker_id = 0;
  // ros::NodeHandle nh;
  
  bbox_marker.color.r = 1.0f;
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 1.0;
  if(color==1){
    bbox_marker.color.r = 0.0f;
  bbox_marker.color.b = 1.0f;
  }
  bbox_marker.scale.x = 0.6;
  bbox_marker.scale.y = 0.45;
  bbox_marker.scale.z = 0.2;

  nav_msgs::Path local_plan;
  local_plan.header.frame_id = visual_frame_;
  local_plan.header.stamp = ros::Time::now();

  geometry_msgs::PoseArray local_plan_pose;
  local_plan_pose.header.frame_id = visual_frame_;
  local_plan_pose.header.stamp = ros::Time::now();
  double timemarker=0;
  for (int i = 0; i <vertex_console.SizePoses(); ++i) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = local_plan.header.frame_id;
    pose_stamped.header.stamp = local_plan.header.stamp;
    pose_stamped.pose.position.x = vertex_console.Pose(i).GetPosition().coeffRef(0);
    pose_stamped.pose.position.y = vertex_console.Pose(i).GetPosition().coeffRef(1);
    pose_stamped.pose.position.z = 0;
    //new for trajectory
    
    if(i<vertex_console.SizeTimeDiffs()){
      pose_stamped.pose.position.z=vertex_console.TimeDiff(i);
      
    }

    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vertex_console.Pose(i).GetTheta());
    local_plan.poses.push_back(pose_stamped);

    geometry_msgs::Pose pose;
    pose.position.x = vertex_console.Pose(i).GetPosition().coeffRef(0);
    pose.position.y = vertex_console.Pose(i).GetPosition().coeffRef(1);
    pose.position.z = 0; 
    pose.orientation = tf::createQuaternionMsgFromYaw(vertex_console.Pose(i).GetTheta());
    local_plan_pose.poses.push_back(pose);
    
    if(i>0){
      timemarker += vertex_console.TimeDiff(i-1);//第一个点t=0
      // timemarker = vertex_console.TimeDiff(i-1);//第一个点t=0
    }
    geometry_msgs::Point p;
    p.x=pose.position.x;
    p.y=pose.position.y;
    p.z=timemarker;
    marker.points.push_back(p);

    bbox_marker.id=i;
    bbox_marker.color.a = 1.0;
    bbox_marker.pose.position.x = pose.position.x;
    bbox_marker.pose.position.y = pose.position.y;
    bbox_marker.pose.position.z = timemarker;
    bbox_marker.pose.orientation = pose.orientation;
    marker_array.markers.push_back(bbox_marker);



  }
  marker_array_pub_.publish(marker_array);
  marker_pub_.publish(marker);
  local_planner_.publish(local_plan);
  pose_pub_.publish(local_plan_pose);
}

} // namespace roborts_local_planner

