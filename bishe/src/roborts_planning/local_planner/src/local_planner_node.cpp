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

#include "local_planner/local_planner_node.h"

#include <csignal>

namespace roborts_local_planner {

using roborts_common::NodeState;
LocalPlannerNode::LocalPlannerNode()
    :  // local_planner_nh_("~"),
       // as_(local_planner_nh_, "/local_planner_node_action",
       // boost::bind(&LocalPlannerNode::ExcuteCB, this, _1), false),
       // has_aim(false),decouple(false),
      new_path_local_(false),
      initialized_(false),
      node_state_(roborts_common::NodeState::IDLE),
      node_error_info_(roborts_common::ErrorCode::OK),
      max_error_(5),
      local_cost_(nullptr),
      tf_(nullptr) {
  if (Init().IsOK()) {
    ROS_INFO("local planner initialize completed.");
  } else {
    ROS_WARN("local planner initialize failed.");
    SetNodeState(NodeState::FAILURE);
  }
  // as_.start();
}

LocalPlannerNode::~LocalPlannerNode() { StopPlanning(); }

roborts_common::ErrorInfo LocalPlannerNode::Init() {
  ROS_INFO("local planner start");
  LocalAlgorithms local_algorithms;
  std::string full_path = ros::package::getPath("roborts_planning") +
                          "/local_planner/config/local_planner.prototxt";
  roborts_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms);
  if (&local_algorithms == nullptr) {
    return roborts_common::ErrorInfo(
        roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
        "Cannot load local planner protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();
  frequency_ = local_algorithms.frequency();
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  global_sub_ = local_planner_nh_.subscribe<nav_msgs::Path>(
      "globalpath", 1, &LocalPlannerNode::GlobalCallback, this);  // new
  // aim_sub_ =
  // local_planner_nh_.subscribe<geometry_msgs::PoseStamped>("aimpoint",1,&LocalPlannerNode::AimCallback,
  // this);//new
  path_sub_ = local_planner_nh_.subscribe<nav_msgs::Path>(
      "globalpath", 1, &LocalPlannerNode::PathCallback, this);  // new
  vel_pub_ =
      local_planner_nh_.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 5);
  // odom_sub_=local_planner_nh_.subscribe<geometry_msgs::PoseStamped>("amcl_pose",1,&LocalPlannerNode::OdomCallback,this);

  // traget_pub_=local_planner_nh_.advertise<nav_msgs::Odometry>("/targetAngle_planning",2);
  // amcl_sub_=
  // local_planner_nh_.subscribe<geometry_msgs::PoseStamped>("amcl_pose",1,&LocalPlannerNode::AmclCallback,this);
  // global_pub_ =
  // local_planner_nh_.advertise<nav_msgs::Path>("localpath",1);//new
  std::string map_path =
      ros::package::getPath("roborts_costmap") +
      "/config/costmap_parameter_config_for_local_plan.prototxt";
  local_cost_ = std::make_shared<roborts_costmap::CostmapInterface>(
      "local_costmap", *tf_, map_path.c_str());
  local_planner_ =
      roborts_common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(
          selected_algorithm_);

  if (local_planner_ == nullptr) {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return roborts_common::ErrorInfo(
        roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
        "local planner algorithm instance can't be loaded");
  }

  std::string name;
  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = LocalVisualizationPtr(
      new LocalVisualization(local_planner_nh_, visual_frame_));
  // vel_pub_ =
  // local_planner_nh_.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc",
  // 5);//放这就不好使 printf("\n\n__success__\n\n"); decouple=true;
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

// void LocalPlannerNode::OdomCallback(const
// geometry_msgs::PoseStamped::ConstPtr & odom){//以里程计频率发布速度指令
// // static ros::Time timeBegin=ros::Time::now();
// //   if(GetNodeState() !=
// NodeState::RUNNING){//loop不进行了即车到达目标点停下
// //     double x = backpos_.pose.orientation.x;
// //     double y = backpos_.pose.orientation.y;
// //     double z = backpos_.pose.orientation.z;
// //     double w = backpos_.pose.orientation.w;
// //     double targetAngle = std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z));//偏航角
// //     while(targetAngle > M_PI) targetAngle -= 2*M_PI;
// //     while(targetAngle < -M_PI) targetAngle += 2*M_PI;

// //     double x1 = (*odom).pose.orientation.x;
// //     double y1 = (*odom).pose.orientation.y;
// //     double z1 = (*odom).pose.orientation.z;
// //     double w1 = (*odom).pose.orientation.w;
// //     double selfAngle = std::atan2(2*(w1*z1+x1*y1),
// 1-2*(y1*y1+z1*z1));//偏航角
// //     while(selfAngle > M_PI) selfAngle -= 2*M_PI;
// //     while(selfAngle < -M_PI) selfAngle += 2*M_PI;

// //     double frequency=6/2*3.14;
// //     double theta=targetAngle+0.6 * std::sin(2*3.14*frequency *
// (ros::Time::now() - timeBegin).toSec());
// //     while(theta > M_PI) theta -= 2*M_PI;
// //     while(theta < -M_PI) theta += 2*M_PI;
// //     double deltaAngle=theta-selfAngle;
// //     while(deltaAngle > M_PI) deltaAngle -= 2*M_PI;
// //     while(deltaAngle < -M_PI) deltaAngle += 2*M_PI;
// //     cmd_vel_.twist.angular.z = deltaAngle/0.05;
// //     vel_pub_.publish(cmd_vel_);
// //   }
// }
// void LocalPlannerNode::AimCallback(const geometry_msgs::PoseStamped::ConstPtr
// & aim){//更新aim位置
//   aimpos_=*aim;
//   if(aimpos_.pose.position.x > 0.01&&aimpos_.pose.position.y>0.01){//有效
//     has_aim=true;
//     decouple=true;
//   }else{
//     has_aim=false;
//     decouple=false;
//   }
// }
void LocalPlannerNode::GlobalCallback(
    const nav_msgs::Path::ConstPtr &path) {  //改完path就结束
  // global_pub_.publish(*path);
  // printf("\ngetglobalpath1\n");
  new_path_local_ = true;  //打断GoalCallback，速度太快导致一直打断，为何
  // backpos_=(*path).poses.back();
  // roborts_common::ErrorInfo error_info = GetErrorInfo();
  // NodeState node_state = GetNodeState();
  // if (node_state == NodeState::FAILURE) {
  //   // roborts_msgs::LocalPlannerFeedback feedback;
  //   // roborts_msgs::LocalPlannerResult result;
  //   // feedback.error_code = error_info.error_code();
  //   // feedback.error_msg  = error_info.error_msg();
  //   // result.error_code   = feedback.error_code;
  //   // as_.publishFeedback(feedback);
  //   // as_.setAborted(result, feedback.error_msg);
  //   ROS_ERROR("Initialization Failed, Failed to execute action!");
  //   return;
  // }
  // if (plan_mtx_.try_lock()) {
  //   local_planner_->SetPlan(*path, local_goal_);//new
  //   plan_mtx_.unlock();
  //   plan_condition_.notify_one();
  // }
  // if (node_state == NodeState::IDLE) {
  //   // ROS_INFO("startplanning");
  //   StartPlanning();
  // }
}
// void LocalPlannerNode::ExcuteCB(const
// roborts_msgs::LocalPlannerGoal::ConstPtr &command) {
void LocalPlannerNode::PathCallback(
    const nav_msgs::Path::ConstPtr &path) {  //等local到目标点即success才退
  // printf("\ngetglobalpath2\n");
  roborts_common::ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  if (node_state == NodeState::FAILURE) {
    // roborts_msgs::LocalPlannerFeedback feedback;
    // roborts_msgs::LocalPlannerResult result;
    // feedback.error_code = error_info.error_code();
    // feedback.error_msg  = error_info.error_msg();
    // result.error_code   = feedback.error_code;
    // as_.publishFeedback(feedback);
    // as_.setAborted(result, feedback.error_msg);
    SetNodeState(NodeState::IDLE);  //失败要重启，不然一直卡着
    ROS_ERROR("Initialization Failed, Failed to execute action!");
    return;
  }
  // printf("\nlocklock\n");
  if (plan_mtx_.try_lock()) {
    // printf("\nlocklock1\n");
    local_planner_->SetPlan(*path, local_goal_);  // new
    // printf("\nlocklock2\n");
    plan_mtx_.unlock();
    // printf("\nlocklock3\n");
    plan_condition_.notify_one();
    // printf("\nlocklock4\n");
  }
  // {
  //   // std::lock_guard<std::mutex> guard(node_state_mtx_);
  //   std::unique_lock<std::mutex> plan_lock(plan_mtx_);
  //   local_planner_->SetPlan(*path, local_goal_);//new
  //   plan_condition_.notify_one();
  // }//一定要有这个大括号，能让线程一直解锁，到达目标才锁上//原因不知道

  // ROS_INFO("Send Plan!");
  if (node_state == NodeState::IDLE) {  //为什么stop后还能进来
    // printf("\n\n__startstart__\n\n");
    StartPlanning();
  }
  // printf("\n\n__startstart1__\n\n");
  while (ros::ok()) {  //一直等问题处理或者success

    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS ||
        node_state == NodeState::FAILURE) {
      if (!error_info.IsOK()) {
        SetErrorInfo(roborts_common::ErrorInfo::OK());  //等待规划过程
      }
      if (node_state ==
          NodeState::
              SUCCESS) {  //如果一直success不了就会卡死，不过不影响啥，该改的都会改
        SetNodeState(NodeState::IDLE);
        // printf("\n_local_success_\n");
        StopPlanning();  //为什么一段全局路径走完要停掉线程
        // printf("\n\n__stopstop__\n\n");
        //只有局部规划让车到达目标点才会停掉这次planning，哪怕全局路径已经变了，无法应对突然更改的目标点//打断之后就没事了
        break;
      } else if (node_state == NodeState::FAILURE) {
        SetNodeState(NodeState::IDLE);
        StopPlanning();
        break;
      }
      // break;//给local设置完path就退出，等下一种path,其他的都没啥用
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    //需要打断功能，保证使用globalplanner最新的path
    if (new_path_local_) {
      // SetNodeState(NodeState::IDLE);
      new_path_local_ = false;  //能被改回来，而且速度很快
      // printf("\n\n__overridepath__\n\n");
      // ROS_INFO("PathOverride!");
      break;
    }
  }
}

void LocalPlannerNode::Loop() {
  roborts_common::ErrorInfo error_info =
      local_planner_->Initialize(local_cost_, tf_, visual_);
  if (error_info.IsOK()) {
    // ROS_INFO("local planner algorithm initialize completed.");
  } else {
    ROS_WARN("local planner algorithm initialize failed.");
    SetNodeState(NodeState::FAILURE);
    SetErrorInfo(error_info);
  }
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  int error_count = 0;

  while (GetNodeState() == NodeState::RUNNING) {
    // printf("\nloop1\n");
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();
    // printf("\nloop2\n");
    // double targetAngle=0;
    roborts_common::ErrorInfo error_info =
        local_planner_->ComputeVelocityCommands(cmd_vel_);  //更改传入参数
    // printf("\nloop3\n");
    // double x1 = backpos_.pose.orientation.x;
    // double y1 = backpos_.pose.orientation.y;
    // double z1 = backpos_.pose.orientation.z;
    // double w1 = backpos_.pose.orientation.w;
    // double targetAngle1 = std::atan2(2*(w1*z1+x1*y1),
    // 1-2*(y1*y1+z1*z1));//偏航角 while(targetAngle1 > M_PI) targetAngle1 -=
    // 2*M_PI; while(targetAngle1 < -M_PI) targetAngle1 += 2*M_PI;
    // targetAngle=targetAngle1;
    // nav_msgs::Odometry tempangle;
    // tempangle.pose.pose.position.x=targetAngle;
    // traget_pub_.publish(tempangle);//发布目标点角度
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - begin);
    int need_time = 1000 / frequency_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;

    if (sleep_time <= std::chrono::milliseconds(0)) {
      // LOG_WARNING << "The time planning once is " << cost_time.count() << "
      // beyond the expected time "
      //         << std::chrono::milliseconds(50).count();
      sleep_time = std::chrono::milliseconds(0);
      // SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once
      // time out."));
    }

    if (error_info.IsOK()) {
      error_count = 0;
      vel_pub_.publish(cmd_vel_);
      if (local_planner_->IsGoalReached()) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (error_count > max_error_ && max_error_ > 0) {
      ROS_WARN("Can not finish plan with max retries( %d  )", max_error_);
      error_info = roborts_common::ErrorInfo(
          roborts_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::FAILURE);
    } else {
      error_count++;
      ROS_ERROR("Can not get cmd_vel for once. %s error count:  %d",
                error_info.error_msg().c_str(), error_count);
    }

    SetErrorInfo(error_info);
  }

  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.angular.z = 0;

  cmd_vel_.accel.linear.x = 0;
  cmd_vel_.accel.linear.y = 0;
  cmd_vel_.accel.angular.z = 0;
  //  for (int i = 0; i < 10; ++i) {
  vel_pub_.publish(cmd_vel_);
  //    usleep(5000);
  //  }
}

void LocalPlannerNode::SetErrorInfo(
    const roborts_common::ErrorInfo error_info) {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(
    const roborts_common::NodeState &node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

roborts_common::NodeState LocalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

roborts_common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning() {
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
  // printf("\nstartstart2\n");
  SetNodeState(roborts_common::NodeState::RUNNING);
  // printf("\nstartstart3\n");
  local_planner_thread_ = std::thread(std::bind(&LocalPlannerNode::Loop, this));
  // printf("\nstartstart4\n");
}

void LocalPlannerNode::StopPlanning() {
  SetNodeState(roborts_common::IDLE);
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(
    const roborts_common::ErrorInfo &algorithm_error_info) {
  SetErrorInfo(algorithm_error_info);
}

}  // namespace roborts_local_planner

void SignalHandler(int signal) {
  if (ros::isInitialized() && ros::isStarted() && ros::ok() &&
      !ros::isShuttingDown()) {
    ros::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  ros::init(argc, argv, "local_planner_node",
            ros::init_options::NoSigintHandler);

  roborts_local_planner::LocalPlannerNode local_planner;

  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  local_planner.StopPlanning();

  return 0;
}
