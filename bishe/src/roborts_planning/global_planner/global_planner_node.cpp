#include "global_planner_node.h"

#include <csignal>

namespace roborts_global_planner {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;
using roborts_common::NodeState;
GlobalPlannerNode::GlobalPlannerNode()
    : has_new_goal(false),
      new_goal_(false),
      new_path_(false),
      get_path_fail_(false),
      get_path_success_(false),
      node_state_(NodeState::IDLE),
      error_info_(ErrorCode::OK) {
  if (Init().IsOK()) {
    ROS_INFO("Global planner initialization completed.");
    StartPlanning();  //初始化成功就把全局规划的循环开启
  } else {
    ROS_ERROR("Initialization failed.");
    SetNodeState(NodeState::FAILURE);
  }
}
GlobalPlannerNode::~GlobalPlannerNode() { StopPlanning(); }

ErrorInfo GlobalPlannerNode::Init() {
  //加载参数
  GlobalPlannerConfig global_planner_config;
  std::string full_path =
      ros::package::getPath("roborts_planning") +
      "/global_planner/config/global_planner_config.prototxt";
  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                             &global_planner_config)) {
    ROS_ERROR("Cannot load global planner protobuf configuration file.");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "Cannot load global planner protobuf configuration file.");
  }
  selected_algorithm_ = global_planner_config.selected_algorithm();
  cycle_duration_ =
      std::chrono::microseconds((int)(1e6 / global_planner_config.frequency()));
  max_retries_ = global_planner_config.max_retries();
  goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();
  goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();

  // //给enemy和ally初值
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
  // path publisher to local_planner_node and rviz
  path_pub_ = nh_.advertise<nav_msgs::Path>("globalpath", 1);  //内部
  last_path_pub_ = nh_.advertise<nav_msgs::Path>("globallastpath", 1);
  // ros::NodeHandle viz_nh("~");
  // path_pub_rviz_ = viz_nh.advertise<nav_msgs::Path>("path",1);
  // path_pub_local_=nh_.advertise<std_msgs::Int32>("new_path", 1);
  // goal subscriber from rvizCB
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "globalgoal", 1, &GlobalPlannerNode::GoalCallback, this);  //内部
  rviz_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 1, &GlobalPlannerNode::RvizCallback,
      this);  // rviz得到
  rviz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("globalgoal", 1);
  // strategy_sub_ =
  // nh_.subscribe<roborts_msgs::strategy2planning>("strategy2planning",2,&GlobalPlannerNode::StrategyCallback,this);//决策得到
  // strategy_pub_ =
  // nh_.advertise<roborts_msgs::planning2strategy>("planning2strategy",1);//给决策
  // aim_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("aimpoint",1);//new
  // ally_sub_=nh_.subscribe<nav_msgs::Odometry>("ally_pose",1,&GlobalPlannerNode::allyCallback,
  // this);//速度和位姿
  // enemy_sub_=nh_.subscribe<nav_msgs::Odometry>("enemy_pose",1,&GlobalPlannerNode::enemyCallback,
  // this);//位置 ally_pub_=nh_.advertise<nav_msgs::Odometry>("",1);
  // kalman_pub_test_=nh_.advertise<nav_msgs::Odometry>("kalman",1);

  // Create tf listener
  //  printf("\n\n__create__tf__global__\n\n");
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  // Create global costmap
  //  printf("\n\n__createcostmapglobal__\n\n");
  std::string map_path =
      ros::package::getPath("roborts_costmap") +
      "/config/costmap_parameter_config_for_global_plan.prototxt";  //读取全局costmap配置文件
  // printf("\n\n__readover__\n\n");
  costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>(
      "global_costmap", *tf_ptr_, map_path.c_str());
  printf("\n\n__createcostmapglobal__over__\n\n");
  // Create the instance of the selected algorithm
  global_planner_ptr_ = roborts_common::AlgorithmFactory<
      GlobalPlannerBase, CostmapPtr>::CreateAlgorithm(selected_algorithm_,
                                                      costmap_ptr_);
  if (global_planner_ptr_ == nullptr) {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "global planner algorithm instance can't be loaded");
  }
  // Initialize path frame from global costmap
  path_.header.frame_id =
      costmap_ptr_->GetGlobalFrameID();  // global path和global costmap ID一致
  last_path_.header.frame_id =
      costmap_ptr_
          ->GetGlobalFrameID();  // global pathlast和global costmap ID一致
  return ErrorInfo(ErrorCode::OK);
}

// from strategy
//  void GlobalPlannerNode::StrategyCallback(const
//  roborts_msgs::strategy2planning::ConstPtr& msg){
//      // printf("\n__get_aim__\n");
//      roborts_msgs::strategy2planning strategy_msg;
//      roborts_msgs::planning2strategy planner_msg;
//      strategy_msg = *msg;
//      std::vector<double> strategy_pos;
//      strategy_pos.resize(8, 0);
//      strategy_pos[0] = strategy_msg.goal_pos_x;
//      strategy_pos[1] = strategy_msg.goal_pos_y;
//      strategy_pos[2] = strategy_msg.aim_pos_x;
//      strategy_pos[3] = strategy_msg.aim_pos_y;
//      strategy_pos[4] = strategy_msg.goal_pos_x_2;
//      strategy_pos[5] = strategy_msg.goal_pos_y_2;
//      strategy_pos[6] = strategy_msg.aim_pos_x_2;
//      strategy_pos[7] = strategy_msg.aim_pos_y_2;
//      int GoalStatus = strategy_msg.goal_status;//决定摆头与否//暂时不用/
//      int ChaseStatus =
//      strategy_msg.chase_status;//前后夹击还是就近，决定使用哪套点，优先前后夹击//暂时不用
//      //接到两个点，角度信息包含进去，作为path的最后一个点的角度进入localplanner
//      geometry_msgs::PoseStamped current_goal;
//      // geometry_msgs::PoseStamped current_aim;
//      current_goal.header.frame_id = "map";
//      current_goal.header.stamp = ros::Time::now();
//      current_goal.pose.position.x = strategy_pos[0];
//      current_goal.pose.position.y = strategy_pos[1];
//      current_goal.pose.position.z = 0;
//      current_aim.header.frame_id = "map";
//      current_aim.header.stamp = ros::Time::now();
//      current_aim.pose.position.x = strategy_pos[2];
//      current_aim.pose.position.y = strategy_pos[3];
//      current_aim.pose.position.z = 0;
//      has_aim_=true;
//      double GoalAngle = std::atan2(strategy_pos[3] - strategy_pos[1],
//      strategy_pos[2] - strategy_pos[0]);//0～PI geometry_msgs::Quaternion q_
//      = tf::createQuaternionMsgFromYaw(GoalAngle);
//      current_goal.pose.orientation = q_;
//      /////////////////////////////可视化两套点//start////////////////////////////
//      /////////////////////////////可视化两套点//end/////////////////////////////
//      //状态量处理
//      //发第一个
//      while(GetNodeState()==NodeState::RUNN
//      int count = 0;
//      while (ros::ok()) {
//          if(get_path_success_){//成功了
//              // printf("\n__success%d__\n",count);
//              get_path_success_ = false;
//              if(count == 0){//第一次就成功
//                  // get_path_success_ = false;
//                  //告诉决策选择了第一个点，前后夹击模式
//                  planner_msg.planning_goal_x = strategy_pos[0];
//                  planner_msg.planning_goal_y = strategy_pos[1];
//                  planner_msg.chase_status = 1;
//                  strategy_pub_.publish(planner_msg);
//                  // printf("\n__first success__\n");
//                  break;
//              }else if(count == 1){//第二次才成功
//                  // get_path_success_ = false;
//                  //告诉决策选择了第二个点，就近射击模式
//                  planner_msg.planning_goal_x = strategy_pos[4];
//                  planner_msg.planning_goal_y = strategy_pos[5];
//                  planner_msg.chase_status = 2;
//                  strategy_pub_.publish(planner_msg);
//                  // printf("\n__second success__\n");
//                  break;
//              }
//              // printf("\n__successbutnotbreak__\n");
//          }
//          if(get_path_fail_){//搜索路径失败，发送第二个点
//              if(count == 1){
//                  //第二个点也失败了
//                  get_path_fail_ = false;
//                  //给决策回发一个消息
//                  //gg
//                  // printf("\n__all fail__\n");
//                  break;
//              }
//              ++count;
//              // current_goal.header.frame_id = "map";
//              current_goal.header.stamp = ros::Time::now();
//              current_goal.pose.position.x = strategy_pos[4];
//              current_goal.pose.position.y = strategy_pos[5];
//              current_goal.pose.position.z = 0;
//              current_aim.header.stamp = ros::Time::now();
//              current_aim.pose.position.x = strategy_pos[6];
//              current_aim.pose.position.y = strategy_pos[7];
//              current_aim.pose.position.z = 0;
//              has_aim_=true;void GlobalPlannerNode::StrategyCallback(const
//              roborts_msgs::strategy2planning::ConstPtr& msg){
//      // printf("\n__get_aim__\n");
//      roborts_msgs::strategy2planning strategy_msg;
//      roborts_msgs::planning2strategy planner_msg;
//      strategy_msg = *msg;
//      std::vector<double> strategy_pos;
//      strategy_pos.resize(8, 0);
//      strategy_pos[0] = strategy_msg.goal_pos_x;
//      strategy_pos[1] = strategy_msg.goal_pos_y;
//      strategy_pos[2] = strategy_msg.aim_pos_x;
//      strategy_pos[3] = strategy_msg.aim_pos_y;
//      strategy_pos[4] = strategy_msg.goal_pos_x_2;
//      strategy_pos[5] = strategy_msg.goal_pos_y_2;
//      strategy_pos[6] = strategy_msg.aim_pos_x_2;
//      strategy_pos[7] = strategy_msg.aim_pos_y_2;
//      int GoalStatus = strategy_msg.goal_status;//决定摆头与否//暂时不用/
//      int ChaseStatus =
//      strategy_msg.chase_status;//前后夹击还是就近，决定使用哪套点，优先前后夹击//暂时不用
//      //接到两个点，角度信息包含进去，作为path的最后一个点的角度进入localplanner
//      geometry_msgs::PoseStamped current_goal;
//      // geometry_msgs::PoseStamped current_aim;
//      current_goal.header.frame_id = "map";
//      current_goal.header.stamp = ros::Time::now();
//      current_goal.pose.position.x = strategy_pos[0];
//      current_goal.pose.position.y = strategy_pos[1];
//      current_goal.pose.position.z = 0;
//      current_aim.header.frame_id = "map";
//      current_aim.header.stamp = ros::Time::now();
//      current_aim.pose.position.x = strategy_pos[2];
//      current_aim.pose.position.y = strategy_pos[3];
//      current_aim.pose.position.z = 0;
//      has_aim_=true;
//      double GoalAngle = std::atan2(strategy_pos[3] - strategy_pos[1],
//      strategy_pos[2] - strategy_pos[0]);//0～PI geometry_msgs::Quaternion q_
//      = tf::createQuaternionMsgFromYaw(GoalAngle);
//      current_goal.pose.orientation = q_;
//      /////////////////////////////可视化两套点//start////////////////////////////
//      /////////////////////////////可视化两套点//end/////////////////////////////
//      //状态量处理
//      //发第一个
//      while(GetNodeState()==NodeState::RUNN
//      int count = 0;
//      while (ros::ok()) {
//          if(get_path_success_){//成功了
//              // printf("\n__success%d__\n",count);
//              get_path_success_ = false;
//              if(count == 0){//第一次就成功
//                  // get_path_success_ = false;
//                  //告诉决策选择了第一个点，前后夹击模式
//                  planner_msg.planning_goal_x = strategy_pos[0];
//                  planner_msg.planning_goal_y = strategy_pos[1];
//                  planner_msg.chase_status = 1;
//                  strategy_pub_.publish(planner_msg);
//                  // printf("\n__first success__\n");
//                  break;
//              }else if(count == 1){//第二次才成功
//                  // get_path_success_ = false;
//                  //告诉决策选择了第二个点，就近射击模式
//                  planner_msg.planning_goal_x = strategy_pos[4];
//                  planner_msg.planning_goal_y = strategy_pos[5];
//                  planner_msg.chase_status = 2;
//                  strategy_pub_.publish(planner_msg);
//                  // printf("\n__second success__\n");
//                  break;
//              }
//              // printf("\n__successbutnotbreak__\n");
//          }
//          if(get_path_fail_){//搜索路径失败，发送第二个点
//              if(count == 1){
//                  //第二个点也失败了
//                  get_path_fail_ = false;
//                  //给决策回发一个消息
//                  //gg
//                  // printf("\n__all fail__\n");
//                  break;
//              }
//              ++count;
//              GoalAngle = std::atan2(strategy_pos[7] - strategy_pos[5],
//              strategy_pos[6] - strategy_pos[4]);//0～PI q_ =
//              tf::createQuaternionMsgFromYaw(GoalAngle);
//              current_goal.pose.orientation = q_;
//              while(GetNodeState()==NodeState::RUNNING&&ros::ok()){//来了新点然后节点正在运行中
//                  new_goal_ = true;//打断GoalCallback
//              }
//              while(ros::ok()&&GetNodeState()!=NodeState::RUNNING){
//                  rviz_pub_.publish(current_goal);
//              }
//              // aim_pub_.publish(current_aim);//给teb发用来做角度规划
//              get_path_fail_ = false;
//              // printf("\n__second point__\n");
//              // break;
//          }
//          // printf("\n__justnotbreak__\n");
//          std::this_thread::sleep_for(std::chrono::microseconds(5));
//      }
//      // get_path_success_=false;
//      // get_path_fail_ = false;//确保一下
//      // printf("\n__getoutaim__\n");
//  }
void GlobalPlannerNode::RvizCallback(
    const geometry_msgs::PoseStamped::ConstPtr &goal) {
  // test
  geometry_msgs::PoseStamped goal1 = *goal;
  // auto q_orign = goal1.pose.orientation;
  // double OrignAngle = atan2(2*(q_orign.w*q_orign.z+q_orign.x*q_orign.y),
  // 1-2*(q_orign.y*q_orign.y+q_orign.z*q_orign.z));//偏航角
  // // geometry_msgs::PoseStamped goal_ = goal;
  // // if(1) {
  //   // std::cout << "OrignAngle " << OrignAngle/M_PI*180 << std::endl;
  //   double base_angle=0;
  //   if(OrignAngle > M_PI/2 || OrignAngle < -M_PI/2) {
  //     base_angle = M_PI;
  //   } else {
  //     base_angle = 0;
  //   }
  //   geometry_msgs::Quaternion q_ =
  //   tf::createQuaternionMsgFromYaw(base_angle); goal1.pose.orientation = q_;
  // //   plan_.poses.back().pose.orientation = q_;
  // // }

  while (GetNodeState() == NodeState::RUNNING &&
         ros::ok()) {  //来了新点然后节点正在运行中
    new_goal_ = true;  //打断GoalCallback
                       // rviz_pub_.publish(*goal);
  }
  while (ros::ok() &&
         GetNodeState() != NodeState::RUNNING) {  //节点空闲就一直发直到节点接到
    rviz_pub_.publish(goal1);
  }
  // printf("\n\n__sendnewgoal__\n\n");
  // has_aim_=false;//new/从rviz接的说明只有目的地，没有对准
  // printf("\n\n__newgoal__\n\n");
}
// from rvizcallback or strategycallback
void GlobalPlannerNode::GoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr &goal) {
  //接到新目标点开始规划新路径
  // ROS_INFO("Received a Goal!");
  // printf("\n\n__newgoal__\n\n");
  ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();
  // Set the current goal
  SetGoal(*goal);
  has_new_goal = true;
  /////////////////////////////可视化goal//start////////////////////////////
  /////////////////////////////可视化goal//end/////////////////////////////
  // new_goal_ = true;
  // If the last state is not running, set it to running
  if (GetNodeState() != NodeState::RUNNING) {
    SetNodeState(NodeState::RUNNING);
  }
  // Notify the condition variable to stop lock waiting the fixed duration
  {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.notify_one();
  }  //一定要有这个大括号，能让线程一直解锁，到达目标才锁上//原因不知道
  while (ros::ok()) {
    //需要一个能打断当前点的处理/如果是第一个点，不能退出，保持循环
    if (new_goal_) {
      new_goal_ = false;
      SetNodeState(NodeState::IDLE);  //当前目标点被打断，节点变成空闲
      // printf("\n__getnewgoal__\n");
      // printf("\n\n__override__\n\n");
      // ROS_INFO("Override!");
      break;
    }

    // Update the current state and error info
    node_state = GetNodeState();
    error_info = GetErrorInfo();
    if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS ||
        node_state == NodeState::FAILURE) {
      if (!error_info.IsOK() || new_path_) {
        // printf("\ngetin\n");
        if (!error_info.IsOK()) {
          SetErrorInfo(ErrorInfo::OK());
        }
        if (new_path_) {
          // printf("\n__newpath__\n");
          new_path_ = false;  //这里保留标志位让strategycallback清除
          get_path_success_ = true;
          get_path_fail_ = false;
          // if(has_aim_){//如果aim更新了/在行进过程中，会一直循环，保持hasaim=true
          //     // current_aim.pose.position.z = 1;//aim有效
          //     aim_pub_.publish(current_aim);//给teb发用来做角度规划/有新路就把对应的aim发出
          //     // printf("\n__hasaim__\n");
          //     // has_aim_=false;//发出后就无aim了
          // }else{//aim没有值
          //     //aim清0发出
          //     current_aim.header.frame_id = "map";
          //     current_aim.header.stamp = ros::Time::now();
          //     current_aim.pose.position.x = 0;//aim无效
          //     current_aim.pose.position.y = 0;//aim无效
          //     aim_pub_.publish(current_aim);//通知teb，aim无效
          //     // printf("\n__no_aim__\n");
          // }
          //如果有newpath就把它发出去
          // pub，在thread里发了
          // path_pub_.publish(path_);
        }
      }
      // printf("\n\n___circle__\n\n");
      // After get the result, deal with actionlib server and jump out of the
      // loop//这是最后了
      if (node_state == NodeState::SUCCESS) {  //到目的地了
        SetNodeState(NodeState::IDLE);
        // has_aim_=false;
        // get_path_fail_ = false;
        break;
      } else if (node_state == NodeState::FAILURE) {
        SetNodeState(NodeState::IDLE);
        // has_aim_=false;
        get_path_fail_ = true;  //搜索路径失败//给strategycallback一个信息
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(5));
  }
  // printf("\nendend\n");
  // get_path_success_=false;//strategycallback先结束，没法清掉标志，goalcallback只有到终点才结束
  // get_path_fail_=false;
}
// double GlobalPlannerNode::getDistanceinOdom(const nav_msgs::Odometry&
// pose1,const nav_msgs::Odometry& pose2){
//     return hypot(pose1.pose.pose.position.x - pose2.pose.pose.position.x
//                 ,pose1.pose.pose.position.y - pose2.pose.pose.position.y);
// }
NodeState GlobalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  return node_state_;
}
void GlobalPlannerNode::SetNodeState(NodeState node_state) {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  node_state_ = node_state;
}
ErrorInfo GlobalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
  return error_info_;
}
void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info) {
  std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
  error_info_ = error_info;
}
geometry_msgs::PoseStamped GlobalPlannerNode::GetGoal() {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  return goal_;
}
void GlobalPlannerNode::SetGoal(geometry_msgs::PoseStamped goal) {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  goal_ = goal;
}
void GlobalPlannerNode::StartPlanning() {
  SetNodeState(NodeState::IDLE);
  plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this);
}
void GlobalPlannerNode::StopPlanning() {
  SetNodeState(NodeState::RUNNING);
  if (plan_thread_.joinable()) {
    plan_thread_.join();
  }
}
void GlobalPlannerNode::PlanThread() {
  // ROS_INFO("Plan thread start!");
  geometry_msgs::PoseStamped current_start;
  geometry_msgs::PoseStamped current_goal;
  std::vector<geometry_msgs::PoseStamped> current_path;
  std::vector<geometry_msgs::PoseStamped> last_path;
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  ErrorInfo error_info;
  int retries = 0;
  // int count=0;
  while (ros::ok()) {
    // ROS_INFO("Wait to plan!");
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    while (GetNodeState() != NodeState::RUNNING) {
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    // double starttime=ros::Time::now().toSec();
    // ROS_INFO("Go on planning!");
    std::chrono::steady_clock::time_point start_time =
        std::chrono::steady_clock::now();
    {
      std::unique_lock<roborts_costmap::Costmap2D::mutex_t> lock(
          *(costmap_ptr_->GetCostMap()->GetMutex()));
      bool error_set = false;
      // Get the robot current pose
      while (!costmap_ptr_->GetRobotPose(current_start)) {
        if (!error_set) {
          // ROS_ERROR("Get Robot Pose Error.");
          SetErrorInfo(
              ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error."));
          error_set = true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }
      // Get the robot current goal and transform to the global frame
      current_goal = GetGoal();
      if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID()) {
        current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
        SetGoal(current_goal);
      }
      // Plan
      //  if(count==0){

      error_info = global_planner_ptr_->Plan(
          current_start, current_goal, current_path, last_path, has_new_goal);

      has_new_goal = false;

      // printf("\n\n__planplan__\n\n");
    }

    // double usedtime = 1000*(ros::Time::now().toSec()-starttime);
    // printf("\nglobalusedtime:\t%fms\n",usedtime);
    if (error_info.IsOK()) {
      // When planner succeed, reset the retry times
      //  printf("\n\n__globalok__\n\n");//卡在角里没法到达目的地，就会一直在这个条件中，且无法到达success状态//卡死
      retries = 0;
      PathVisualization(
          current_path,
          last_path);  // visualize and set the nav_path in action servcice
      // Set the goal to avoid the same goal from getting transformed every time
      current_goal = current_path.back();
      SetGoal(current_goal);
      // Decide whether robot reaches the goal according to
      // tolerance//车到终点才停，不然一直规划
      if (GetDistance(current_start, current_goal) < goal_distance_tolerance_
          // && GetAngle(current_start, current_goal) < goal_angle_tolerance_
      ) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (max_retries_ > 0 && retries > max_retries_) {
      // When plan failed to max retries, return failure
      //  ROS_ERROR("Can not get plan with max retries( %d )", max_retries_ );
      //  error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max
      //  retries.");
      SetNodeState(NodeState::FAILURE);
      retries = 0;
    } else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR)) {
      // When goal is not reachable, return failure immediately
      //  ROS_ERROR("Current goal is not valid!");
      SetNodeState(NodeState::FAILURE);
      retries = 0;
    } else {
      // Increase retries
      retries++;
      // ROS_ERROR("Can not get plan for once. %s",
      // error_info.error_msg().c_str());
    }
    // Set and update the error info
    SetErrorInfo(error_info);
    // Deal with the duration to wait
    std::chrono::steady_clock::time_point end_time =
        std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                              start_time);
    // printf("\nglobalusedtime:\t%fms\n", execution_duration.count());
    sleep_time = cycle_duration_ - execution_duration;
    // std::cout << "sleep time" << sleep_time.count()*0.001 << "
    // cycle_duration_" << cycle_duration_.count()*0.001 << std::endl; Report
    // warning while planning timeout
    if (sleep_time <= std::chrono::microseconds(0)) {
      ROS_ERROR("  is %ld beyond the expected time %ld",
                execution_duration.count(), cycle_duration_.count());
      std::cout << "sleep time" << sleep_time.count() * 0.001
                << " cycle_duration_" << cycle_duration_.count() * 0.001
                << std::endl;
      sleep_time = std::chrono::microseconds(0);
      SetErrorInfo(
          ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }
  }
  ROS_INFO("Plan thread terminated!");
}

void GlobalPlannerNode::PathVisualization(
    const std::vector<geometry_msgs::PoseStamped> &path,
    const std::vector<geometry_msgs::PoseStamped> &last_path) {
  path_.poses = path;
  last_path_.poses = last_path;
  // path_pub_rviz_.publish(path_);//发给rviz
  // printf("\n\n__pathpub__\n\n");
  // if(count___==0){
  path_pub_.publish(path_);  //发给local
  // ++count___;
  // }

  last_path_pub_.publish(last_path_);
  new_path_ = true;
  // std_msgs::Int32 msg;
  // msg.data=1;
  // path_pub_local_.publish(msg);//发给local打断用
}
double GlobalPlannerNode::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                      const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}
double GlobalPlannerNode::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                   const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
}

}  // namespace roborts_global_planner
void SignalHandler(int signal) {
  if (ros::isInitialized() && ros::isStarted() && ros::ok() &&
      !ros::isShuttingDown()) {
    ros::shutdown();
  }
}
int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  ros::init(argc, argv, "global_planner_node",
            ros::init_options::NoSigintHandler);
  roborts_global_planner::GlobalPlannerNode global_planner;
  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}