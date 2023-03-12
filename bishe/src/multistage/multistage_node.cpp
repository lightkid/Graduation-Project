#include "multistage_node.h"

namespace multistage {  // 仿真共享网络
MultistageNode::MultistageNode() {
  goal_count = 0;
  carColor0 = 1;  // blue2red1
  carColor1 = 1;
  exclude_self_distance = 0.4;
  goal_sub = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, &MultistageNode::GoalCallback, this);
  pose_sub_0 = nh_.subscribe<geometry_msgs::PoseStamped>(
      "robot_0/amcl_pose", 2, &MultistageNode::Pose0Callback, this);  //
  pose_sub_1 = nh_.subscribe<geometry_msgs::PoseStamped>(
      "robot_1/amcl_pose", 2, &MultistageNode::Pose1Callback, this);
  // rviz目标i点
  goal_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>(
      "/robot_0/move_base_simple/goal", 1);
  goal_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>(
      "/robot_1/move_base_simple/goal", 1);
  // 发布友方odom
  allypose_pub_0 =
      nh_.advertise<nav_msgs::Odometry>("/robot_0/ally/odom_fix", 1);  // 给0发
  allypose_pub_1 =
      nh_.advertise<nav_msgs::Odometry>("/robot_1/ally/odom_fix", 1);  // 给1发
  // 发布敌方position
  enemy_pub_0 =
      nh_.advertise<nav_msgs::Odometry>("/robot_0/outpost/planning", 1);
  enemy_pub_1 =
      nh_.advertise<nav_msgs::Odometry>("/robot_1/outpost/planning", 1);
  // 发布友方trajectory
  allytrajectory_pub_0 = nh_.advertise<roborts_msgs::Allytrajectory>(
      "/robot_0/ally/trajectory", 1);
  allytrajectory_pub_1 = nh_.advertise<roborts_msgs::Allytrajectory>(
      "/robot_1/ally/trajectory", 1);
  allytrajectory_sub_0 = nh_.subscribe<nav_msgs::Path>(
      "/robot_0/trajectory", 2, &MultistageNode::Trajectory0Callback, this);
  allytrajectory_sub_1 = nh_.subscribe<nav_msgs::Path>(
      "/robot_1/trajectory", 2, &MultistageNode::Trajectory1Callback, this);
  smoothpath0_pub_ = nh_.advertise<nav_msgs::Path>("/robot_0/smoothpath", 2);
  smoothpath1_pub_ = nh_.advertise<nav_msgs::Path>("/robot_1/smoothpath", 2);
  trajectorys0_.trajectorys.resize(3);  // 0ally1enemy2enemy
  trajectorys1_.trajectorys.resize(3);
  // trajectorys0_pub_ =
  //     nh_.advertise<roborts_msgs::trajectorys>("/robot_0/trajectorys", 2);
  trajectorys0_pub_ =
      nh_.advertise<roborts_msgs::trajectorys>("/robot_0/trajectorys", 2);
  trajectorys1_pub_ =
      nh_.advertise<roborts_msgs::trajectorys>("/robot_1/trajectorys", 2);
  // 接敌方位置信息拟合更新轨迹网络(敌方信息用bag来模拟)
  enemy_sub0_ = nh_.subscribe<nav_msgs::Odometry>(
      "/robot_0/outpost/planning", 2, &MultistageNode::Enemy0Callback, this);
  enemy_sub1_ = nh_.subscribe<nav_msgs::Odometry>(
      "/robot_1/outpost/planning", 2, &MultistageNode::Enemy1Callback, this);
  enemy_geo_sub0_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/vision_pose/pose", 2, &MultistageNode::Enemygeo0Callback, this);
  enemy_geo_pub0_ =
      nh_.advertise<nav_msgs::Odometry>("/robot_0/outpost/planning", 2);
  start_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/startsimlulation", 1, &MultistageNode::StartCallback, this);

  odom201_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      "odom201", 1, &MultistageNode::odom201Callback, this);
  odom202_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      "odom202", 1, &MultistageNode::odom202Callback, this);
  point201_pub_ = nh_.advertise<sensor_msgs::PointCloud>("point201", 1);
  point202_pub_ = nh_.advertise<sensor_msgs::PointCloud>("point202", 1);
  path201_pub_ = nh_.advertise<nav_msgs::Path>("path201", 1);
  path202_pub_ = nh_.advertise<nav_msgs::Path>("path202", 1);

  // buff_state.resize(6,0);
  // bonus_sub =
  // nh_.subscribe<roborts_msgs::GameZoneArray>("/game_zone_array_status", 2,
  //                                                                         &GlobalPlannerCostmapNode::BonusZoneCallback,this);
  // bonus_pub = nh_.advertise<>("/buff_state",1);
  // odom_fix_sub_=nh_.subscribe<nav_msgs::Odometry>("/odom_fix",2,&MultistageNode::odomfixCallback,this);
  // originpoint_pub_=nh_.advertise<sensor_msgs::PointCloud>("/originpoint",2);
  // smoothpath_pub_=nh_.advertise<nav_msgs::Path>("/smoothpath",2);
  mypoint_pub_ = nh_.advertise<nav_msgs::Odometry>("/mypoint", 2);
}
MultistageNode::~MultistageNode() = default;
void MultistageNode::Enemygeo0Callback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  geometry_msgs::PoseStamped pos = *msg;
  nav_msgs::Odometry pose;
  pose.pose.pose.position.x = pos.pose.position.x;
  pose.pose.pose.position.y = pos.pose.position.y;
  enemy_geo_pub0_.publish(pose);
}
void MultistageNode::odom201Callback(const nav_msgs::Odometry::ConstPtr &msg) {
  // printf("\ngetpoint\n");
  nav_msgs::Odometry current_pose = *msg;
  current_pose.header.stamp = ros::Time::now();
  double h = 0.05;
  int N = 30;
  double predict_length = 0.2;
  if (pose_history201_.size() > N - 1)
    pose_history201_.erase(pose_history201_.begin());  // 删除第一个
  pose_history201_.push_back(current_pose);
  // originpoint201_.points.clear();
  geometry_msgs::Point32 point;
  point.x = current_pose.pose.pose.position.x;
  point.y = current_pose.pose.pose.position.y;
  originpoint201_.points.push_back(point);
  originpoint201_.header.frame_id = "map";

  if (pose_history201_.size() < N / 2) {
    kalman_point201_.SetX(current_pose);  // 第一个点不处理
    return;
  }
  // lsq
  //  unsigned int order_polynomial=5;//多项式次数
  //  double smooth = 0.01;
  //  std::vector<double> t;t.resize(pose_history201_.size());
  //  std::vector<double> x;x.resize(pose_history201_.size());
  //  std::vector<double> y;y.resize(pose_history201_.size());

  // // std::vector<unsigned int> index;index.reserve(10);
  // // originpoint201_.points.clear();
  // double t_start=pose_history201_[0].header.stamp.toSec();
  // double t_max=0;
  // for(size_t k=0;k<pose_history201_.size();++k){
  //     t[k]=pose_history201_[k].header.stamp.toSec()-t_start;//第一个点t=0
  //     if(t[k]>t_max){
  //         t_max=t[k];
  //     }
  //     // t[k]=k*h;
  //     x[k]=pose_history201_[k].pose.pose.position.x;
  //     y[k]=pose_history201_[k].pose.pose.position.y;
  //     // geometry_msgs::Point32 point;
  //     // point.x=pose_history201_[k].pose.pose.position.x;
  //     // point.y=pose_history201_[k].pose.pose.position.y;
  //     // originpoint201_.points.push_back(point);
  // }
  // std::vector<double> rho;rho.resize(t.size(),1);
  // std::vector<double> alpha;alpha.resize(order_polynomial);
  // std::vector<double> beta;beta.resize(order_polynomial-1);
  // std::vector<double> xa;xa.resize(order_polynomial+1);//多项式族系数
  // std::vector<double> ya;ya.resize(order_polynomial+1);
  // // leastSquare(order_polynomial,t,x,y,rho,xa,ya,alpha,beta);
  // leastSquareSmooth(order_polynomial,t,x,y,rho,xa,ya,alpha,beta,smooth,predict_length);
  // //把轨迹显示出来/原始10个点和插值之后的
  // // originpoint201_.header.frame_id="map";
  // smoothpath201_.poses.clear();
  // double t_end=pose_history201_.back().header.stamp.toSec() - t_start;
  // for(double ti=0;ti<t_end+predict_length;ti+=0.005){
  //     geometry_msgs::PoseStamped point;
  //     point.header.frame_id="map";
  //     point.pose.position.x=getfai(xa,alpha,beta,ti);
  //     point.pose.position.y=getfai(ya,alpha,beta,ti);
  //     //
  //     printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
  //     smoothpath201_.poses.push_back(point);
  // }
  // smoothpath201_.header.frame_id="map";
  // point201_pub_.publish(originpoint201_);
  // path201_pub_.publish(smoothpath201_);

  // kalman
  double dt = current_pose.header.stamp.toSec() -
              kalman_point201_.GetPose().header.stamp.toSec();
  kalman_point201_.Setdt(dt);
  kalman_point201_.predict();
  kalman_point201_.updateK();
  kalman_point201_.updateZ(current_pose);
  kalman_point201_.updateX();
  nav_msgs::Odometry filtered_point = kalman_point201_.GetPose();
  // 预测0.2秒的5个点0.04s间隔
  geometry_msgs::PoseStamped point1;
  point1.header.frame_id = "map";
  point1.pose.position.x = filtered_point.pose.pose.position.x;
  point1.pose.position.y = filtered_point.pose.pose.position.y;
  smoothpath201_.poses.clear();

  smoothpath201_.poses.push_back(point1);
  // for(size_t k=0;k<pose_history_.size();++k){
  //     geometry_msgs::Point32 point1;
  //     point1.x=pose_history_[k].pose.pose.position.x;
  //     point1.y=pose_history_[k].pose.pose.position.y;
  //     originpoint_.points.push_back(point1);
  // }
  // if(smoothpath_.poses.size()>30){
  //     smoothpath_.poses.erase(smoothpath_.poses.begin());//删除第一个
  // }
  for (double ti = 0.005; ti < 0.005 + predict_length; ti += 0.005) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = filtered_point.pose.pose.position.x +
                            ti * filtered_point.twist.twist.linear.x;
    point.pose.position.y = filtered_point.pose.pose.position.y +
                            ti * filtered_point.twist.twist.linear.y;
    // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
    smoothpath201_.poses.push_back(point);
  }
  smoothpath201_.header.frame_id = "map";
  // originpoint_.header.frame_id="map";
  path201_pub_.publish(smoothpath201_);
  point201_pub_.publish(originpoint201_);
}
void MultistageNode::odom202Callback(const nav_msgs::Odometry::ConstPtr &msg) {}
void MultistageNode::StartCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {  // 开始自定义的掩饰
  geometry_msgs::PoseStamped start = *msg;
  geometry_msgs::PoseStamped goal0;
  geometry_msgs::PoseStamped goal1;
  // test1
  // goal0.pose.position.x = 1.7;
  // goal0.pose.position.y = 2.9;
  // goal0.pose.orientation.w = 1;
  // goal1.pose.position.x = 2.9;
  // goal1.pose.position.y = 1.7;
  // goal1.pose.orientation.z = 0.70046159992;
  // goal1.pose.orientation.w = 0.713690091732;
  // test2
  // goal0.pose.position.x = 3.0;
  // goal0.pose.position.y = 0.6;
  // goal0.pose.orientation.w = 1;
  // goal1.pose.position.x = 5;
  // goal1.pose.position.y = 0.6;
  // goal1.pose.orientation.z = 0.999952376442;
  // goal1.pose.orientation.w = 0.00975934668695;
  // test3
  goal0.pose.position.x = 3.01;
  goal0.pose.position.y = 3.88;
  goal0.pose.orientation.w = 1;
  goal1.pose.position.x = 4.01;
  goal1.pose.position.y = 2.88;
  goal1.pose.orientation.z = 0.70046159992;
  goal1.pose.orientation.w = 0.713690091732;

  while (ros::ok()) {
    goal_pub_0.publish(goal0);
    goal_pub_1.publish(goal1);
    if (getDistanceinGeo(pose0_, goal0) < 0.15 &&
        getDistanceinGeo(pose1_, goal1) < 0.15) {  // 到达预设起点
      break;
    }
  }

  double nowtime = ros::Time::now().toSec();
  while (ros::ok()) {
    double timenow = ros::Time::now().toSec();
    if (timenow - nowtime > 1.5) break;
  }  // 延时1.5s后进行下一步
     // test1
  // goal0.pose.position.x = 3.9;
  // goal0.pose.position.y = 2.9;
  // goal1.pose.position.x = 2.9;
  // goal1.pose.position.y = 3.9;
  // test2
  // goal0.pose.position.x = 6;
  // goal0.pose.position.y = 0.6;
  // goal1.pose.position.x = 2.2;
  // goal1.pose.position.y = 0.6;
  // test3
  goal0.pose.position.x = 2.45;
  goal0.pose.position.y = 0.599;
  goal1.pose.position.x = 1.27;
  goal1.pose.position.y = 2.9;
  double start_time = ros::Time::now().toSec();
  // double end_time=0;
  double used_time = 0;
  while (ros::ok()) {
    goal_pub_0.publish(goal0);
    goal_pub_1.publish(goal1);
    nav_msgs::Odometry recorder;
    recorder.header.stamp = ros::Time::now();
    recorder.pose.pose.position.x =
        getDistanceinGeo(pose0_, pose1_);  // 双车距离
    if (getDistanceinGeo(pose0_, goal0) < 0.1 &&
        getDistanceinGeo(pose1_, goal1) < 0.1) {  // 到达预设终点
      double end_time = ros::Time::now().toSec();
      used_time = end_time - start_time;  // 计算两车完成任务所需时间
      printf("\n__mission_success_used_time:\t%f__\n", used_time);
      break;
    }
    if (ros::Time::now().toSec() - start_time > 60) {
      // 都没完成任务
      printf("\n__mission_failed__\n");
      break;
    }
  }
}
void MultistageNode::Enemy0Callback(
    const nav_msgs::Odometry::ConstPtr
        &msg) {  // robot0和robot1接到的哨岗信息一致
  // printf("\n\nget enemy\n\n");
  // geometry_msgs::PoseStamped pose = *msg;
  nav_msgs::Odometry pos = *msg;
  // pos.pose.pose.position.x = pose.pose.position.x;
  // pos.pose.pose.position.y = pose.pose.position.y;
  // allypose_pub_0.publish(pos);
  std::chrono::steady_clock::time_point now_time =
      std::chrono::steady_clock::now();
  for (auto iter = carKalContainer0.begin(); iter != carKalContainer0.end();) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now_time -
                                                              iter->cartime)
            .count() > 2000) {  // 超时
      iter = carKalContainer0.erase(
          iter);  // 当删除时erase函数自动指向下一个位置，就不需要进行++
    } else {
      iter++;  // 当没有进行删除的时候，迭代器++
    }
  }  // 去除动态障碍物队列中长时间未更新的车
  if (carKalContainer0.empty()) {  // 没车
    carKalman car;
    car.SetX(pos);
    car.cartime = std::chrono::steady_clock::now();
    carKalContainer0.push_back(car);
  } else {                // 有车
    double mindis = 1.5;  // 最远不能超过1.5m
    int index = carKalContainer0.size() + 5;
    for (int i = 0; i < carKalContainer0.size(); i++) {
      carKalContainer0[i].predict();  // 一步预测
      double dis = getDistanceinOdom(pos, carKalContainer0[i].GetPose());
      if (dis < mindis) {  // 匹配成功
        index = i;
        mindis = dis;
      }  // 得到的点与队列中的点进行匹配/匹配上就更新/没匹配上就pushback
    }    // for
    if (index < carKalContainer0.size()) {  // 匹配上更新
      carKalContainer0[index].updateK();
      carKalContainer0[index].updateZ(pos);
      carKalContainer0[index].updateX();
      carKalContainer0[index].cartime =
          std::chrono::steady_clock::now();  // new
    } else if (carKalContainer0.size() <
               3) {  // 没有匹配上的/加入/车最多3辆(2x敌+1x友)
      carKalman car;
      car.SetX(pos);
      car.cartime = std::chrono::steady_clock::now();
      carKalContainer0.push_back(car);
    }
  }  // 有车/这时队列里的车都是较准的
  // printf("\n\npredict\n\n");
  // carinfo_.obs_car.clear();//丢失目标点需要有个保护！！！！！！！！！！！！！！
  // trajectorys1_.trajectorys.clear();
  trajectorys0_.trajectorys.clear();
  for (unsigned int i = 0; i < carKalContainer0.size(); ++i) {
    nav_msgs::Odometry car = carKalContainer0[i].GetPose();
    car.header.stamp = ros::Time::now();
    carKalContainer0[i].addPose(car);
    std::vector<nav_msgs::Odometry> poslist = carKalContainer0[i].getPoseList();
    unsigned int order_polynomial = 5;  // 多项式次数
    double predict_length = 0.4;
    std::vector<double> t;
    t.resize(poslist.size(), 0);
    std::vector<double> x;
    x.resize(poslist.size(), 0);
    std::vector<double> y;
    y.resize(poslist.size(), 0);
    double t_now = 0;
    double t_start = poslist[0].header.stamp.toSec();
    for (unsigned int k = 0; k < poslist.size(); ++k) {
      t[k] = t_now;
      x[k] = poslist[k].pose.pose.position.x;
      y[k] = poslist[k].pose.pose.position.y;
      t_now += poslist[k].header.stamp.toSec() - t_start;  // 加上时间间隔
    }
    // printf("\n\npredict1\n\n");
    std::vector<double> rho;
    rho.resize(t.size(), 1);
    std::vector<double> alpha;
    alpha.resize(order_polynomial);
    std::vector<double> beta;
    beta.resize(order_polynomial - 1);
    std::vector<double> xa;
    xa.resize(order_polynomial + 1);  // 多项式族系数
    std::vector<double> ya;
    ya.resize(order_polynomial + 1);
    // printf("\n\npredict2\n\n");
    leastSquareSmooth(order_polynomial, t, x, y, rho, xa, ya, alpha, beta,
                      0.001,
                      predict_length);  // 预测0.4s
    // printf("\n\npredict2.5\n\n");
    std::vector<double> paramx = paramfai(alpha, beta, xa, order_polynomial);
    std::vector<double> paramy = paramfai(alpha, beta, ya, order_polynomial);
    roborts_msgs::Allytrajectory msg;
    // printf("\n\npredict3\n\n");
    msg.timestart.data = t.back();
    msg.timestamp.data = t.back() + predict_length;
    for (unsigned int i = 0; i < paramx.size(); ++i) {
      std_msgs::Float64 xat;
      xat.data = paramx[i];
      std_msgs::Float64 yat;
      yat.data = paramy[i];
      msg.xa.push_back(xat);
      msg.ya.push_back(yat);
    }
    // printf("\n\npredic4\n\n");
    // trajectorys1_.trajectorys.push_back(msg);
    trajectorys0_.trajectorys.push_back(msg);
    // carinfo_.obs_car.push_back(car);
  }
  // printf("\ntjy0cnt:%d\n", trajectorys0_.trajectorys.size());
  // trajectorys1_pub_.publish(trajectorys1_);
  trajectorys0_pub_.publish(trajectorys0_);
  // geometry_msgs::PoseStamped goal0;
  // goal0.pose.position.x = 2.5;
  // goal0.pose.position.y = 0.717;
  // goal0.header.frame_id = "map";
  // geometry_msgs::PoseStamped startflag;
  // startflag.pose.position.x = 2.1;
  // startflag.pose.position.y = 1.61;
  // startflag.header.frame_id = "map";

  // if (getDistanceinGeo(pose, startflag) < 0.03) CCCstart = true;
  // if (CCCstart) goal_pub_0.publish(goal0);
  // enemy_pub_.publish(carinfo_);
}
double MultistageNode::getDistanceinOdom(const nav_msgs::Odometry &pose1,
                                         const nav_msgs::Odometry &pose2) {
  return hypot(pose1.pose.pose.position.x - pose2.pose.pose.position.x,
               pose1.pose.pose.position.y - pose2.pose.pose.position.y);
}
void MultistageNode::Enemy1Callback(const nav_msgs::Odometry::ConstPtr &msg) {
  nav_msgs::Odometry pos = *msg;
  std::chrono::steady_clock::time_point now_time =
      std::chrono::steady_clock::now();
  for (auto iter = carKalContainer1.begin(); iter != carKalContainer1.end();) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now_time -
                                                              iter->cartime)
            .count() > 2000) {  // 超时
      iter = carKalContainer1.erase(
          iter);  // 当删除时erase函数自动指向下一个位置，就不需要进行++
    } else {
      iter++;  // 当没有进行删除的时候，迭代器++
    }
  }  // 去除动态障碍物队列中长时间未更新的车
  if (carKalContainer1.empty()) {  // 没车
    carKalman car;
    car.SetX(pos);
    car.cartime = std::chrono::steady_clock::now();
    carKalContainer1.push_back(car);
  } else {                // 有车
    double mindis = 1.5;  // 最远不能超过1.5m
    int index = carKalContainer1.size() + 5;
    for (int i = 0; i < carKalContainer1.size(); i++) {
      carKalContainer1[i].predict();  // 一步预测
      double dis = getDistanceinOdom(pos, carKalContainer1[i].GetPose());
      if (dis < mindis) {  // 匹配成功
        index = i;
        mindis = dis;
      }  // 得到的点与队列中的点进行匹配/匹配上就更新/没匹配上就pushback
    }    // for
    if (index < carKalContainer1.size()) {  // 匹配上更新
      carKalContainer1[index].updateK();
      carKalContainer1[index].updateZ(pos);
      carKalContainer1[index].updateX();
      carKalContainer1[index].cartime =
          std::chrono::steady_clock::now();  // new
    } else if (carKalContainer1.size() <
               3) {  // 没有匹配上的/加入/车最多3辆(2x敌+1x友)
      carKalman car;
      car.SetX(pos);
      car.cartime = std::chrono::steady_clock::now();
      carKalContainer1.push_back(car);
    }
  }  // 有车/这时队列里的车都是较准的
  // carinfo_.obs_car.clear();//丢失目标点需要有个保护！！！！！！！！！！！！！！
  trajectorys1_.trajectorys.clear();
  // trajectorys0_.trajectorys.clear();
  for (unsigned int i = 0; i < carKalContainer1.size(); ++i) {
    nav_msgs::Odometry car = carKalContainer1[i].GetPose();
    car.header.stamp = ros::Time::now();
    carKalContainer1[i].addPose(car);
    std::vector<nav_msgs::Odometry> poslist = carKalContainer1[i].getPoseList();
    unsigned int order_polynomial = 5;  // 多项式次数
    double predict_length = 0.4;
    std::vector<double> t;
    t.resize(poslist.size(), 0);
    std::vector<double> x;
    x.resize(poslist.size(), 0);
    std::vector<double> y;
    y.resize(poslist.size(), 0);
    double t_now = 0;
    double t_start = poslist[0].header.stamp.toSec();
    for (unsigned int k = 0; k < poslist.size(); ++k) {
      t[k] = t_now;
      x[k] = poslist[k].pose.pose.position.x;
      y[k] = poslist[k].pose.pose.position.y;
      t_now += poslist[k].header.stamp.toSec() - t_start;  // 加上时间间隔
    }
    std::vector<double> rho;
    rho.resize(t.size(), 1);
    std::vector<double> alpha;
    alpha.resize(order_polynomial);
    std::vector<double> beta;
    beta.resize(order_polynomial - 1);
    std::vector<double> xa;
    xa.resize(order_polynomial + 1);  // 多项式族系数
    std::vector<double> ya;
    ya.resize(order_polynomial + 1);
    leastSquareSmooth(order_polynomial, t, x, y, rho, xa, ya, alpha, beta, 0.01,
                      predict_length);  // 预测0.4s
    std::vector<double> paramx = paramfai(alpha, beta, xa, order_polynomial);
    std::vector<double> paramy = paramfai(alpha, beta, ya, order_polynomial);
    roborts_msgs::Allytrajectory msg;
    msg.timestart.data = t.back();
    msg.timestamp.data = t.back() + predict_length;
    for (unsigned int i = 0; i < paramx.size(); ++i) {
      std_msgs::Float64 xat;
      xat.data = paramx[i];
      std_msgs::Float64 yat;
      yat.data = paramy[i];
      msg.xa.push_back(xat);
      msg.ya.push_back(yat);
    }
    trajectorys1_.trajectorys.push_back(msg);
    // trajectorys0_.trajectorys.push_back(msg);
    // carinfo_.obs_car.push_back(car);
  }
  printf("\ntjy1cnt:%d\n", trajectorys1_.trajectorys.size());
  trajectorys1_pub_.publish(trajectorys1_);
  // trajectorys0_pub_.publish(trajectorys0_);
  // enemy_pub_.publish(carinfo_);
}
void MultistageNode::Pose0Callback(const geometry_msgs::PoseStamped::ConstPtr
                                       &msg) {  // 接robot0位置发送至网络
  geometry_msgs::PoseStamped pos = *msg;
  // pose0_.pose.position.x = pos.pose.pose.position.x;
  // pose0_.pose.position.y = pos.pose.pose.position.y;
  pose0_ = *msg;
  nav_msgs::Odometry pose;
  pose.pose.pose.position.x = pos.pose.position.x;
  pose.pose.pose.position.y = pos.pose.position.y;
  if (carColor0 == carColor1) {    // 两辆车是同伴
    allypose_pub_1.publish(pose);  // 给1车发队友
  } else {                         // 两辆车是敌人
    // bool include=false;
    // for(auto &it:enemy_pose_1.obs_car){
    //     if(getDistanceinGeo(it,pos)<exclude_self_distance){
    //         include = true;
    //         it.pose.position.x = pos.pose.position.x; //+
    //         it.pose.position.x)/2; it.pose.position.y =
    //         pos.pose.position.y;// + it.pose.position.y)/2;
    //         it.pose.position.z = pos.pose.position.z;
    //     }
    // }
    // if(!include){
    //     enemy_pose_1.obs_car.push_back(*msg);
    // }
    // enemy_pose_1.obs_car.push_back(*msg);
    enemy_pub_1.publish(pose);  // 给1车发敌人
    //  printf("\n_pub_r0_to_r1_\n");
  }
}
void MultistageNode::Pose1Callback(const geometry_msgs::PoseStamped::ConstPtr
                                       &msg) {  // 接robot1位置发送至网络
  geometry_msgs::PoseStamped pos = *msg;
  // pose1_.pose.position.x = pos.pose.pose.position.x;
  // pose1_.pose.position.y = pos.pose.pose.position.y;
  pose1_ = *msg;
  nav_msgs::Odometry pose;
  pose.pose.pose.position.x = pos.pose.position.x;
  pose.pose.pose.position.y = pos.pose.position.y;
  if (carColor1 == carColor0) {    // 两辆车是同伴
    allypose_pub_0.publish(pose);  // 给0车发队友
  } else {                         // 两辆车是敌人
    // bool include=false;
    // for(auto &it:enemy_pose_0.obs_car){
    //     if(getDistanceinGeo(it,pos)<exclude_self_distance){
    //         include = true;
    //         it.pose.position.x = pos.pose.position.x;// +
    //         it.pose.position.x)/2; it.pose.position.y =
    //         pos.pose.position.y;// + it.pose.position.y)/2;
    //         it.pose.position.z = pos.pose.position.z;
    //     }
    // }
    // if(!include){
    //     enemy_pose_0.obs_car.push_back(*msg);
    // }
    enemy_pub_0.publish(pose);  // 给0车发敌人
  }
}
void MultistageNode::Trajectory0Callback(
    const nav_msgs::Path::ConstPtr &msg) {  // 接到0的轨迹给1发
  nav_msgs::Path path = *msg;
  if (carColor0 == carColor1) {
    // if(path.poses.size()<6)
    // 拟合，给出6个系数，不需要tstart
    unsigned int order_polynomial = 5;  // 多项式次数
    std::vector<double> t;
    t.resize(path.poses.size(), 0);
    std::vector<double> x;
    x.resize(path.poses.size(), 0);
    std::vector<double> y;
    y.resize(path.poses.size(), 0);

    double t_now = 0;
    for (unsigned int k = 0; k < path.poses.size(); ++k) {
      t[k] = t_now;
      x[k] = path.poses[k].pose.position.x;
      y[k] = path.poses[k].pose.position.y;
      t_now += path.poses[k].pose.position.z;  // 加上时间间隔
    }

    std::vector<double> rho;
    rho.resize(t.size(), 1);
    std::vector<double> alpha;
    alpha.resize(order_polynomial);
    std::vector<double> beta;
    beta.resize(order_polynomial - 1);
    std::vector<double> xa;
    xa.resize(order_polynomial + 1);  // 多项式族系数
    std::vector<double> ya;
    ya.resize(order_polynomial + 1);
    leastSquareSmooth(order_polynomial, t, x, y, rho, xa, ya, alpha, beta, 0.01,
                      0);
    // leastSquare(order_polynomial,t,x,y,rho,xa,ya,alpha,beta);
    std::vector<double> paramx = paramfai(alpha, beta, xa, order_polynomial);
    std::vector<double> paramy = paramfai(alpha, beta, ya, order_polynomial);
    // xa加ya共12个数
    roborts_msgs::Allytrajectory msg;
    msg.timestamp.data = t.back();
    msg.timestart.data = 0.0;
    for (unsigned int i = 0; i < paramx.size(); ++i) {
      std_msgs::Float64 xat;
      xat.data = paramx[i];
      std_msgs::Float64 yat;
      yat.data = paramy[i];
      msg.xa.push_back(xat);
      msg.ya.push_back(yat);
    }
    smoothpath0_.poses.clear();
    for (double ti = 0; ti < t.back(); ti += 0.005) {
      geometry_msgs::PoseStamped point;
      point.header.frame_id = "map";
      point.pose.position.x = getfai(xa, alpha, beta, ti);
      point.pose.position.y = getfai(ya, alpha, beta, ti);
      // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
      smoothpath0_.poses.push_back(point);
    }
    smoothpath0_.header.frame_id = "map";
    smoothpath0_pub_.publish(smoothpath0_);
    // allytrajectory_pub_1.publish(msg);
    // roborts_msgs::trajectorys msgs;
    // msgs.trajectorys.push_back(msg);
    // trajectorys1_.trajectorys[0]=msg;
    // trajectorys1_pub_.publish(trajectorys1_);
  }
}
void MultistageNode::Trajectory1Callback(const nav_msgs::Path::ConstPtr &msg) {
  nav_msgs::Path path = *msg;
  if (carColor0 == carColor1) {
    // 拟合，给出6个系数，不需要tstart
    unsigned int order_polynomial = 5;  // 多项式次数
    std::vector<double> t;
    t.resize(path.poses.size());
    std::vector<double> x;
    x.resize(path.poses.size());
    std::vector<double> y;
    y.resize(path.poses.size());
    double t_now = 0;
    for (size_t k = 0; k < path.poses.size(); ++k) {
      t[k] = t_now;
      x[k] = path.poses[k].pose.position.x;
      y[k] = path.poses[k].pose.position.y;
      t_now += path.poses[k].pose.position.z;  // 加上时间间隔
    }
    std::vector<double> rho;
    rho.resize(t.size(), 1);
    std::vector<double> alpha;
    alpha.resize(order_polynomial);
    std::vector<double> beta;
    beta.resize(order_polynomial - 1);
    std::vector<double> xa;
    xa.resize(order_polynomial + 1);  // 多项式族系数
    std::vector<double> ya;
    ya.resize(order_polynomial + 1);
    // leastSquare(order_polynomial,t,x,y,rho,xa,ya,alpha,beta);
    leastSquareSmooth(order_polynomial, t, x, y, rho, xa, ya, alpha, beta, 0.01,
                      0);
    std::vector<double> paramx = paramfai(alpha, beta, xa, order_polynomial);
    std::vector<double> paramy = paramfai(alpha, beta, ya, order_polynomial);
    // xa加ya共12个数
    roborts_msgs::Allytrajectory msg;
    msg.timestamp.data = t.back();
    msg.timestart.data = 0.0;
    for (unsigned int i = 0; i < paramx.size(); ++i) {
      std_msgs::Float64 xat;
      xat.data = paramx[i];
      std_msgs::Float64 yat;
      yat.data = paramy[i];
      msg.xa.push_back(xat);
      msg.ya.push_back(yat);
    }
    smoothpath1_.poses.clear();
    for (double ti = 0; ti < t.back(); ti += 0.005) {
      geometry_msgs::PoseStamped point;
      point.header.frame_id = "map";
      point.pose.position.x = getfai(xa, alpha, beta, ti);
      point.pose.position.y = getfai(ya, alpha, beta, ti);
      // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
      smoothpath1_.poses.push_back(point);
    }
    smoothpath1_.header.frame_id = "map";
    smoothpath1_pub_.publish(smoothpath1_);
    // allytrajectory_pub_0.publish(msg);
    // roborts_msgs::trajectorys msgs;
    // trajectorys0_.trajectorys[0]=msg;
    // trajectorys0_pub_.publish(trajectorys0_);
  }
}
// void GlobalPlannerCostmapNode::BonusZoneCallback(const
// roborts_msgs::GameZoneArray::ConstPtr & bonus){

// }

// #define LEAST_SQAURE
// #define RANSAC
#define KALMAN
#define OSQP
// #define BFGS
void MultistageNode::odomfixCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // #ifdef LEAST_SQUARE || RANSAC
  // 最小二乘法/3次曲线较直//没法解析求光滑的
  nav_msgs::Odometry current_pose = *msg;
  current_pose.header.stamp = ros::Time::now();
  double now_time = ros::Time::now().toSec();
  double h = 0.05;
  int N = 30;
  double predict_length = 0.2;
  if (pose_history_.size() > N - 1)
    pose_history_.erase(pose_history_.begin());  // 删除第一个
  pose_history_.push_back(current_pose);
  originpoint_.points.clear();

  if (pose_history_.size() < N / 2) {
    kalman_point_.SetX(current_pose);  // 第一个点不处理
    return;
  }
  // #endif

  // if(pose_history_.size()<4){
  //     return;
  // }
  // double xa0=0,xa1=0,xa2=0,xa3=0;
  // double ya0=0,ya1=0,ya2=0,ya3=0;
  // double alpha0 = 0, alpha1 = 0, alpha2 = 0;
  // double beta1 = 0, beta2 = 0;
  // double A0 = 0, A1 = 0, A2 = 0, A3 = 0;
  // double f20=0;// = alpha0*alpha1 - beta1;
  // double f21=0;// = - alpha0 - alpha1;
  // double f30=0;// = alpha0*beta2 + alpha2*beta1 - alpha0*alpha1*alpha2;
  // double f31=0;// = alpha0*alpha1 + alpha0*alpha2 + alpha1*alpha2 - beta1 -
  // beta2; double f32=0;// = - alpha0 - alpha1 - alpha2; int size =
  // pose_history_.size();
  // // printf("\nsize=%d\n",size);
  // alpha0=(size-1)*h/2;
  // A0=size;
  // for(int i=0;i<size;++i){
  //     double ti=i*h;
  //     double f1=ti - alpha0;
  //     A1+=f1*f1;
  //     alpha1+=ti*f1*f1;
  // }
  // alpha1/=A1;
  // beta1=A1/A0;
  // f20 = alpha0*alpha1 - beta1;
  // f21 = - alpha0 - alpha1;
  // for(int i=0;i<size;++i){
  //     double ti=i*h;
  //     double f2=ti*ti+f21*ti+f20;
  //     A2+=f2*f2;
  //     alpha2+=ti*f2*f2;
  // }
  // alpha2/=A2;
  // beta2=A2/A1;
  // f30 = alpha0*beta2 + alpha2*beta1 - alpha0*alpha1*alpha2;
  // f31 = alpha0*alpha1 + alpha0*alpha2 + alpha1*alpha2 - beta1 - beta2;
  // f32 = - alpha0 - alpha1 - alpha2;
  // for(int i=0;i<size;++i){
  //     double ti=i*h;
  //     double f3=ti*ti*ti+f32*ti*ti+f31*ti+f30;
  //     A3+=f3*f3;
  // }
  // for(int i = 0; i < size; i++){
  //     double ti = i*h;
  //     double x=pose_history_[i].pose.pose.position.x;
  //     double y=pose_history_[i].pose.pose.position.y;
  //     xa0 += x;
  //     xa1 += x*(ti-alpha0);
  //     xa2 += x*(ti*ti+f21*ti+f20);
  //     xa3 += x*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //     ya0 += y;
  //     ya1 += y*(ti-alpha0);
  //     ya2 += y*(ti*ti+f21*ti+f20);
  //     ya3 += y*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  // }
  // double alpha=0.01;//光滑因子
  // double t_=(size-1)*h;
  // A2+=alpha*(t_+3*t_*t_+f32*t_);
  // A3+=alpha*(12*t_*t_*t_+(3+12*f32)*t_*t_+(1+4*f32)*f32*t_);
  // xa0/=A0;  xa1/=A1;  xa2/=A2;  xa3/=A3;
  // ya0/=A0;  ya1/=A1;  ya2/=A2;  ya3/=A3;
  // //把轨迹显示出来/原始10个点和插值之后的
  // // originpoint_.points.clear();
  // for(int i=0;i<pose_history_.size();++i){
  //     geometry_msgs::Point32 point;
  //     point.x=pose_history_[i].pose.pose.position.x;
  //     point.y=pose_history_[i].pose.pose.position.y;
  //     originpoint_.points.push_back(point);
  // }
  // originpoint_.header.frame_id="map";
  // smoothpath_.poses.clear();
  // for(int i=0;i<(size-1)*10+30;++i){
  //     double ti = i*h/10;
  //     geometry_msgs::PoseStamped point;
  //     point.header.frame_id="map";
  //     point.pose.position.x=xa0+xa1*(ti-alpha0)+xa2*(ti*ti+f21*ti+f20)+xa3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //     point.pose.position.y=ya0+ya1*(ti-alpha0)+ya2*(ti*ti+f21*ti+f20)+ya3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //     smoothpath_.poses.push_back(point);
  // }
  // double costx=0;
  // double costy=0;
  // for(int i=0;i<pose_history_.size();++i){
  //     double ti=i*h;
  //     double dx = pose_history_[i].pose.pose.position.x -
  //     xa0+xa1*(ti-alpha0)+xa2*(ti*ti+f21*ti+f20)+xa3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //     double dy = pose_history_[i].pose.pose.position.y -
  //     ya0+ya1*(ti-alpha0)+ya2*(ti*ti+f21*ti+f20)+ya3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  //     costx += dx*dx;
  //     costy += dy*dy;
  // }
  // costx+=alpha*(12*xa3*xa3*t*t*t+(6*xa2*xa3+12*xa3*xa3*f32)*t*t+(xa2+2*xa3*f32)*(xa2+2*xa3*f32)*t);
  // costy+=alpha*(12*ya3*ya3*t*t*t+(6*ya2*ya3+12*ya3*ya3*f32)*t*t+(ya2+2*ya3*f32)*(ya2+2*ya3*f32)*t);
  // smoothpath_.header.frame_id="map";
  // originpoint_pub_.publish(originpoint_);
  // smoothpath_pub_.publish(smoothpath_);
  // mypoint_.header.frame_id="map";
  // mypoint_.header.stamp=ros::Time::now();
  // double ti=9*h;
  // mypoint_.pose.pose.position.x=xa0+xa1*(ti-alpha0)+xa2*(ti*ti+f21*ti+f20)+xa3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  // mypoint_.pose.pose.position.y=ya0+ya1*(ti-alpha0)+ya2*(ti*ti+f21*ti+f20)+ya3*(ti*ti*ti+f32*ti*ti+f31*ti+f30);
  // //
  // mypoint_.twist.twist.linear.x=xa1+2*xa2*ti+xa2*f21+3*xa3*ti*ti+2*xa3*f32*t+xa3*f31;
  // //
  // mypoint_.twist.twist.linear.y=ya1+2*ya2*ti+ya2*f21+3*ya3*ti*ti+2*ya3*f32*t+ya3*f31;
  // mypoint_.twist.twist.linear.x=costx;
  // mypoint_.twist.twist.linear.y=costy;
  // mypoint_pub_.publish(mypoint_);//速度不太准阿
#ifdef LEAST_SQAURE
  unsigned int order_polynomial = 5;  // 多项式次数
  double smooth = 0.01;
  std::vector<double> t;
  t.resize(pose_history_.size());
  std::vector<double> x;
  x.resize(pose_history_.size());
  std::vector<double> y;
  y.resize(pose_history_.size());

  // std::vector<unsigned int> index;index.reserve(10);
  originpoint_.points.clear();
  double t_start = pose_history_[0].header.stamp.toSec();
  double t_max = 0;
  for (size_t k = 0; k < pose_history_.size(); ++k) {
    t[k] = pose_history_[k].header.stamp.toSec() - t_start;  // 第一个点t=0
    if (t[k] > t_max) {
      t_max = t[k];
    }
    // t[k]=k*h;
    x[k] = pose_history_[k].pose.pose.position.x;
    y[k] = pose_history_[k].pose.pose.position.y;
    geometry_msgs::Point32 point;
    point.x = pose_history_[k].pose.pose.position.x;
    point.y = pose_history_[k].pose.pose.position.y;
    originpoint_.points.push_back(point);
  }
  // for(unsigned int i=0;i<t.size();++i){
  //     t[i]/=t_max;//时间归一化
  // }
  // if(t.size()==10)
  // printf("\nt:%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",t[0],t[1],t[2],t[3],t[4],t[5],t[6],t[7],t[8],t[9]);
  std::vector<double> rho;
  rho.resize(t.size(), 1);
  std::vector<double> alpha;
  alpha.resize(order_polynomial);
  std::vector<double> beta;
  beta.resize(order_polynomial - 1);
  std::vector<double> xa;
  xa.resize(order_polynomial + 1);  // 多项式族系数
  std::vector<double> ya;
  ya.resize(order_polynomial + 1);
  // leastSquare(order_polynomial,t,x,y,rho,xa,ya,alpha,beta);
  leastSquareSmooth(order_polynomial, t, x, y, rho, xa, ya, alpha, beta, smooth,
                    predict_length);

  // 把轨迹显示出来/原始10个点和插值之后的
  originpoint_.header.frame_id = "map";
  smoothpath_.poses.clear();
  double t_end = pose_history_.back().header.stamp.toSec() - t_start;
  for (double ti = 0; ti < t_end; ti += 0.005) {
    // for(int i=0;i<(pose_history_.size()-1)*10+30;++i){
    //     double ti = i*h/10;
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = getfai(xa, alpha, beta, ti);
    point.pose.position.y = getfai(ya, alpha, beta, ti);
    // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
    smoothpath_.poses.push_back(point);
  }
  // 专门给tend画的，预测就不用单独画了
  geometry_msgs::PoseStamped point;
  point.header.frame_id = "map";
  point.pose.position.x = getfai(xa, alpha, beta, t_end);
  point.pose.position.y = getfai(ya, alpha, beta, t_end);
  // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
  smoothpath_.poses.push_back(point);

  double dxe =
      getfai(xa, alpha, beta, t_end) - current_pose.pose.pose.position.x;
  double dye =
      getfai(ya, alpha, beta, t_end) - current_pose.pose.pose.position.y;
  double costx = 0;
  double costy = 0;
  for (size_t i = 0; i < pose_history_.size(); ++i) {
    double ti = pose_history_[i].header.stamp.toSec() - t_start;
    double dx =
        pose_history_[i].pose.pose.position.x - getfai(xa, alpha, beta, ti);
    double dy =
        pose_history_[i].pose.pose.position.y - getfai(ya, alpha, beta, ti);

    costx += dx * dx;
    costy += dy * dy;
  }
  double smoothx = 0;
  double smoothy = 0;
  std::vector<double> faix = paramfai(alpha, beta, xa, order_polynomial);
  std::vector<double> faix2order = difffaik(faix, 2);
  std::vector<double> faix2faix2 = faifai(faix2order, faix2order);
  std::vector<double> faiy = paramfai(alpha, beta, ya, order_polynomial);
  std::vector<double> faiy2order = difffaik(faiy, 2);
  std::vector<double> faiy2faiy2 = faifai(faiy2order, faiy2order);
  double tn = t_end;
  for (unsigned int i = 0; i < faix2faix2.size(); ++i) {
    smoothx += faix2faix2[i] * tn / (i + 1);
    tn *= t_end;
  }
  tn = t_end;
  for (unsigned int i = 0; i < faiy2faiy2.size(); ++i) {
    smoothy += faiy2faiy2[i] * tn / (i + 1);
    tn *= t_end;
  }

  // 比对预测效果，目前预测0.2s的大约5个点
  // 记录未来5个点的预测位置（全局变量）
  double predict_err = 0;
  if (predict_points_.size() == 10) {
    // 用现在30个点的最后5个跟预测队列列首比对
    unsigned int sizen = pose_history_.size();
    predict_err = 0;
    for (unsigned int i = 0; i < 10; ++i) {
      double pxi = predict_points_[0].poses[i].pose.position.x;
      double pyi = predict_points_[0].poses[i].pose.position.y;
      double txi = pose_history_[sizen - 10 + i].pose.pose.position.x;
      double tyi = pose_history_[sizen - 10 + i].pose.pose.position.y;
      predict_err +=
          std::sqrt((pxi - txi) * (pxi - txi) + (pyi - tyi) * (pyi - tyi));
    }
    predict_points_.erase(predict_points_.begin());
  }
  // 预测当前，更新队列
  nav_msgs::Path predict_point;
  for (double ti = t_end + 0.04; ti < t_end + 0.04 + predict_length;
       ti += 0.04) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = getfai(xa, alpha, beta, ti);
    point.pose.position.y = getfai(ya, alpha, beta, ti);
    // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
    predict_point.poses.push_back(point);  // 记录5个点
  }
  predict_points_.push_back(predict_point);

  // costx+=smooth*smoothx;
  // costy+=smooth*smoothy;

  smoothpath_.header.frame_id = "map";
  originpoint_pub_.publish(originpoint_);
  smoothpath_pub_.publish(smoothpath_);
  mypoint_.header.frame_id = "map";
  mypoint_.header.stamp = pose_history_.back().header.stamp;
  // double ti=9*h;
  mypoint_.pose.pose.position.x = std::sqrt(dxe * dxe + dye * dye);  // F1x
  mypoint_.pose.pose.position.y = smooth * smoothx;                  // F2x
  mypoint_.pose.pose.position.z = costx + smooth * smoothx;          // F1+F2x
  mypoint_.twist.twist.linear.x = costy;                             // F1y
  mypoint_.twist.twist.linear.y = smooth * smoothy;                  // F2y
  mypoint_.twist.twist.linear.z = costy + smooth * smoothy;          // F1+F2y
  mypoint_.twist.twist.angular.x =
      (ros::Time::now().toSec() - now_time) * 1000;  // usedtime
  mypoint_.twist.twist.angular.y = predict_err;
  mypoint_pub_.publish(mypoint_);  // 速度不太准阿

#endif
  // ransac
#ifdef RANSAC
  // 对所有点随即重排，然后取前几个
  std::vector<unsigned int> sample_idx;
  sample_idx.reserve(pose_history_.size());
  std::vector<unsigned int> inlier_idx;
  inlier_idx.reserve(pose_history_.size());
  std::vector<double> inlier_t;
  inlier_t.reserve(pose_history_.size());
  std::vector<double> inlier_x;
  inlier_x.reserve(pose_history_.size());
  std::vector<double> inlier_y;
  inlier_y.reserve(pose_history_.size());
  unsigned int order_polynomial = 5;  // 多项式次数
  std::vector<double> rho;
  rho.resize(pose_history_.size(), 1);
  std::vector<double> alpha;
  alpha.resize(order_polynomial);
  std::vector<double> beta;
  beta.resize(order_polynomial - 1);
  std::vector<double> xa;
  xa.resize(order_polynomial + 1);  // 多项式族系数
  std::vector<double> ya;
  ya.resize(order_polynomial + 1);
  double t_start = pose_history_[0].header.stamp.toSec();
  // double cost_min=std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < pose_history_.size(); ++i) {
    sample_idx.push_back(i);
    geometry_msgs::Point32 point;
    point.x = pose_history_[i].pose.pose.position.x;
    point.y = pose_history_[i].pose.pose.position.y;
    originpoint_.points.push_back(point);
  }
  unsigned seed;
  int count = 5;             // 最大迭代次数
  double err_thre2 = 0.001;  // 误差平方阈值要调
  double inlierNum = 1.2;    // 内点比例因子>1
  double smooth = 0.01;
  while (count > 0) {  // 30个点选10个
    --count;
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(sample_idx.begin(), sample_idx.end(),
                 std::default_random_engine(seed));
    unsigned int inlier_size =
        pose_history_.size() < 10 ? pose_history_.size() : 10;
    for (unsigned int i = 0; i < inlier_size; ++i) {
      unsigned int idx_now = sample_idx[i];
      inlier_t.push_back(pose_history_[idx_now].header.stamp.toSec() - t_start);
      inlier_x.push_back(pose_history_[idx_now].pose.pose.position.x);
      inlier_y.push_back(pose_history_[idx_now].pose.pose.position.y);
    }
    // leastSquare(order_polynomial,inlier_t,inlier_x,inlier_y,rho,xa,ya,alpha,beta);
    leastSquareSmooth(order_polynomial, inlier_t, inlier_x, inlier_y, rho, xa,
                      ya, alpha, beta, smooth);
    // 计算内点
    inlier_idx.clear();
    for (unsigned int i = 0; i < pose_history_.size(); ++i) {
      double ti = pose_history_[i].header.stamp.toSec() - t_start;
      double dx =
          pose_history_[i].pose.pose.position.x - getfai(xa, alpha, beta, ti);
      double dy =
          pose_history_[i].pose.pose.position.y - getfai(ya, alpha, beta, ti);
      double err2 = dx * dx + dy * dy;
      if (err2 < err_thre2) {
        inlier_idx.push_back(i);
      }
    }
    inlier_t.clear();
    inlier_x.clear();
    inlier_y.clear();
    if (inlierNum * inlier_idx.size() >
        pose_history_.size()) {  // 66%的点都是内点(模型正确)//要调
      for (unsigned int i = 0; i < inlier_idx.size(); ++i) {
        unsigned int idx_now = inlier_idx[i];
        inlier_t.push_back(pose_history_[idx_now].header.stamp.toSec() -
                           t_start);
        inlier_x.push_back(pose_history_[idx_now].pose.pose.position.x);
        inlier_y.push_back(pose_history_[idx_now].pose.pose.position.y);
      }
      // rho.resize(inlier_idx.size(),1);
      leastSquare(order_polynomial, inlier_t, inlier_x, inlier_y, rho, xa, ya,
                  alpha, beta);
      break;
    }
  }
  // printf("\ncount:%d\n",5 - count);
  // 把轨迹显示出来/原始10个点和插值之后的
  originpoint_.header.frame_id = "map";
  smoothpath_.poses.clear();
  double t_end = pose_history_.back().header.stamp.toSec() - t_start;
  for (double ti = 0; ti < t_end + 0.2; ti += 0.005) {
    // for(int i=0;i<(pose_history_.size()-1)*10+30;++i){
    //     double ti = i*h/10;
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = getfai(xa, alpha, beta, ti);
    point.pose.position.y = getfai(ya, alpha, beta, ti);
    // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
    smoothpath_.poses.push_back(point);
  }
  smoothpath_.header.frame_id = "map";
  originpoint_pub_.publish(originpoint_);
  smoothpath_pub_.publish(smoothpath_);

#endif

#ifdef KALMAN
  // nav_msgs::Odometry current_pose=*msg;
  // static int cntkalman=0;
  // current_pose.header.stamp=ros::Time::now();
  // if(cntkalman==0){
  // cntkalman++;

  // return;
  // }
  double dt = current_pose.header.stamp.toSec() -
              kalman_point_.GetPose().header.stamp.toSec();
  kalman_point_.Setdt(dt);
  kalman_point_.predict();
  kalman_point_.updateK();
  kalman_point_.updateZ(current_pose);
  kalman_point_.updateX();
  nav_msgs::Odometry filtered_point = kalman_point_.GetPose();
  // printf("\ndx:%f\tdy:%f\n",filtered_point.pose.pose.position.x-current_pose.pose.pose.position.x,filtered_point.pose.pose.position.y-current_pose.pose.pose.position.y);
  // printf("\nvx:%f\tvy:%f\n",filtered_point.twist.twist.linear.x,filtered_point.twist.twist.linear.y);
  // 预测0.2秒的5个点0.04s间隔
  geometry_msgs::PoseStamped point;
  point.header.frame_id = "map";
  point.pose.position.x = filtered_point.pose.pose.position.x;
  point.pose.position.y = filtered_point.pose.pose.position.y;
  smoothpath_.poses.push_back(point);
  for (size_t k = 0; k < pose_history_.size(); ++k) {
    geometry_msgs::Point32 point1;
    point1.x = pose_history_[k].pose.pose.position.x;
    point1.y = pose_history_[k].pose.pose.position.y;
    originpoint_.points.push_back(point1);
  }
  if (smoothpath_.poses.size() > 30) {
    smoothpath_.poses.erase(smoothpath_.poses.begin());  // 删除第一个
  }
  smoothpath_.header.frame_id = "map";
  originpoint_.header.frame_id = "map";
  smoothpath_pub_.publish(smoothpath_);
  originpoint_pub_.publish(originpoint_);
  double dx =
      filtered_point.pose.pose.position.x - current_pose.pose.pose.position.x;
  double dy =
      filtered_point.pose.pose.position.y - current_pose.pose.pose.position.y;

  double predict_err = 0;
  if (predict_points_.size() == 5) {
    // 用现在30个点的最后5个跟预测队列列首比对
    unsigned int sizen = pose_history_.size();
    predict_err = 0;
    for (unsigned int i = 0; i < 5; ++i) {
      double pxi = predict_points_[0].poses[i].pose.position.x;
      double pyi = predict_points_[0].poses[i].pose.position.y;
      double txi = pose_history_[sizen - 5 + i].pose.pose.position.x;
      double tyi = pose_history_[sizen - 5 + i].pose.pose.position.y;
      predict_err +=
          std::sqrt((pxi - txi) * (pxi - txi) + (pyi - tyi) * (pyi - tyi));
    }
    predict_points_.erase(predict_points_.begin());
  }
  // 预测当前，更新队列
  nav_msgs::Path predict_point;
  for (double ti = 0.04; ti < 0.04 + predict_length; ti += 0.04) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = filtered_point.pose.pose.position.x +
                            ti * filtered_point.twist.twist.linear.x;
    point.pose.position.y = filtered_point.pose.pose.position.y +
                            ti * filtered_point.twist.twist.linear.y;
    // printf("\nx:%f\ty:%f\n",point.pose.position.x,point.pose.position.y);
    predict_point.poses.push_back(point);  // 记录5个点
  }
  predict_points_.push_back(predict_point);
  mypoint_.header.frame_id = "map";
  mypoint_.header.stamp = current_pose.header.stamp;
  // double ti=9*h;
  mypoint_.pose.pose.position.x = std::sqrt(dx * dx + dy * dy);  // F1x
  // mypoint_.pose.pose.position.y=smooth*smoothx;//F2x
  // mypoint_.pose.pose.position.z=costx+smooth*smoothx;//F1+F2x
  // mypoint_.twist.twist.linear.x=costy;//F1y
  // mypoint_.twist.twist.linear.y=smooth*smoothy;//F2y
  // mypoint_.twist.twist.linear.z=costy+smooth*smoothy;//F1+F2y
  mypoint_.twist.twist.angular.x =
      (ros::Time::now().toSec() - now_time) * 1000;  // usedtime
  // 有大滞后
  mypoint_.twist.twist.angular.y = predict_err;
  mypoint_pub_.publish(mypoint_);  // 速度不太准阿
#endif
}

bool MultistageNode::leastSquareSmooth(
    unsigned int n, const std::vector<double> &t, const std::vector<double> &x,
    const std::vector<double> &y, std::vector<double> &rho,
    std::vector<double> &xa, std::vector<double> &ya,
    std::vector<double> &alpha, std::vector<double> &beta, double smooth,
    double predict_length) {
  // printf("0000");
  std::vector<double> A;
  A.resize(n + 1, 1);  // 多项式族内积
  // 计算k=0
  double alpha0u = 0;
  double alpha0d = 0;
  for (size_t i = 0; i < t.size(); ++i) {
    alpha0u += rho[i] * t[i];
    alpha0d += rho[i];
  }
  alpha[0] = alpha0u / alpha0d;
  A[0] = alpha0d;
  // printf("1110");
  for (unsigned int k = 1; k < n; ++k) {
    double alphaup = 0;
    double alphadown = 0;
    double betadown = 0;
    // printf("1112");
    for (size_t i = 0; i < t.size(); ++i) {
      double fki = getfaik(alpha, beta, t[i], k);
      double fkli = getfaik(alpha, beta, t[i], k - 1);
      alphaup += rho[i] * t[i] * fki * fki;
      alphadown += rho[i] * fki * fki;
      betadown += rho[i] * fkli * fkli;
    }
    alpha[k] = alphaup / alphadown;
    A[k] = alphadown;
    beta[k - 1] = alphadown / betadown;
  }
  // printf("\nok\n");
  // 计算Ajk大表//用matrix写
  // Eigen::SparseMatrix<double> Ajk;
  Eigen::MatrixXd Ajk;
  Ajk.resize(n + 1, n + 1);
  // Eigen::VectorXf fai_list;fai_list.resize(n+1);
  // std::vector<std::vector<double>> Ajk(n+1,std::vector<double>(n+1,0));
  std::vector<std::vector<double>> fai_list;
  fai_list.reserve(n + 1);
  for (unsigned int i = 0; i < n + 1; ++i) {
    fai_list.push_back(paramfaik2(alpha, beta, i));
  }
  double t_end = 0;
  for (unsigned int i = 0; i < t.size(); ++i) {
    if (t[i] > t_end) {
      t_end = t[i];
    }
  }
  t_end += predict_length;  // 对预测部分也平滑
  // printf("\ntend:%f\n", t_end);
  // printf("\ntend:%f\n",t_end);
  for (unsigned int j = 0; j < n + 1; ++j) {
    std::vector<double> faij2 = difffaik(fai_list[j], 2);
    for (unsigned int k = 0; k < n + 1; ++k) {
      std::vector<double> faik2 = difffaik(fai_list[k], 2);
      std::vector<double> faijk = faifai(faij2, faik2);
      double Ajk_ = 0;
      double ti = t_end;
      for (unsigned int i = 0; i < faijk.size(); ++i) {
        Ajk_ += faijk[i] * ti / (i + 1);
        ti *= t_end;
      }
      Ajk(j, k) = 2 * smooth * Ajk_;
      if (j == k) {
        Ajk(j, k) += 2 * A[k];
      }
    }
  }  // end Ajk
  // printf("\nok2\n");
  // // std::cout<<Ajk<<std::endl;
  // std::vector<double> xfaiklist(n+1,0);
  // std::vector<double> yfaiklist(n+1,0);
  Eigen::VectorXd xfaiklist;
  xfaiklist.resize(n + 1);
  Eigen::VectorXd yfaiklist;
  yfaiklist.resize(n + 1);
  for (size_t k = 0; k < A.size(); ++k) {
    double xan = 0, yan = 0;
    for (size_t i = 0; i < t.size(); ++i) {
      xan += rho[i] * x[i] * getfaik(alpha, beta, t[i], k);
      yan += rho[i] * y[i] * getfaik(alpha, beta, t[i], k);
    }
    xfaiklist(k) = -2 * xan;
    yfaiklist(k) = -2 * yan;
    xa[k] = xan / A[k];  // 优化用初值
    ya[k] = yan / A[k];
  }
  // printf("1111");
  // 优化器
#ifdef OSQP
  OsqpEigen::Solver solver;
  int numberOfVariables = xa.size();
  Eigen::SparseMatrix<double> hessian = Ajk.sparseView();
  solver.settings()->setWarmStart(true);
  solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfVariables(numberOfVariables);
  solver.data()->setNumberOfConstraints(0);
  if (!solver.data()->setHessianMatrix(hessian)) return false;
  // 求解xa
  Eigen::VectorXd gradient = xfaiklist;
  if (!solver.data()->setGradient(gradient)) return false;
  if (!solver.initSolver()) return false;
  if (!solver.solve()) return false;
  Eigen::VectorXd QPSolutionx = solver.getSolution();
  // 求解ya
  gradient = yfaiklist;
  if (!solver.updateGradient(gradient)) return false;
  // if(!solver.initSolver()) return false;
  if (!solver.solve()) return false;
  Eigen::VectorXd QPSolutiony = solver.getSolution();
  // std::cout<<"xa"<<QPSolutionx<<std::endl;
  // std::cout<<"ya"<<QPSolutiony<<std::endl;
  for (unsigned int i = 0; i < numberOfVariables; ++i) {
    xa[i] = QPSolutionx(i);
    ya[i] = QPSolutiony(i);
    // printf("\nchanged\n");
  }
#endif
  // #ifdef BFGS
  //   unsigned int Optiloop = 30;
  //   unsigned int size = xa.size();
  //   // Eigen::MatrixXd Dx=Eigen::MatrixXd::Identity(size,size);//海塞矩阵
  //   // Eigen::MatrixXd Dy=Eigen::MatrixXd::Identity(size,size);//海塞矩阵
  //   Eigen::VectorXd ax;
  //   ax.resize(size);
  //   Eigen::VectorXd axl;
  //   axl.resize(size);  //优化变量
  //   Eigen::VectorXd ay;
  //   ay.resize(size);
  //   Eigen::VectorXd ayl;
  //   ayl.resize(size);
  //   Eigen::VectorXd gx = Eigen::VectorXd::Zero(size);
  //   Eigen::VectorXd gxl;
  //   gxl.resize(size);  //梯度
  //   Eigen::VectorXd gy = Eigen::VectorXd::Zero(size);
  //   Eigen::VectorXd gyl;
  //   gyl.resize(size);  //梯度
  //   Eigen::VectorXd sx;
  //   Eigen::VectorXd cx;
  //   Eigen::VectorXd sy;
  //   Eigen::VectorXd cy;
  //   Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);  //单位阵
  //   // std::cout<<I<<std::endl;
  //   // std::cout<<Dx<<std::endl;
  //   for (unsigned int i = 0; i < size; ++i) {
  //     ax(i) = xa[i];
  //     ay(i) = ya[i];
  //     // ax(i)=0.5;
  //     // ay(i)=0.5;
  //     // printf("\nxa:%f\tA:%f\n",xa[i],A[i]);
  //   }
  //   // std::cout<<"axorign"<<ax<<std::endl;
  //   std::cout << "ayorgin" << ay << std::endl;
  //   // Eigen::MatrixXd Ajk_inverse=Ajk.inverse();
  //   // ax=Ajk_inverse*xfaiklist;
  //   // ay=Ajk_inverse*yfaiklist;
  //   // printf("\nok4 %d\t%d\n",ax.size(),ay.size());
  //   // std::cout<<ax<<std::endl;
  //   unsigned int cnt = 0;
  //   double tempx = 1;  //步长
  //   while (cnt < Optiloop) {
  //     axl = ax;
  //     gxl = gx;
  //     gx = xfaiklist + Ajk * ax;
  //     if (gx.norm() < 1e-6) {
  //       // std::cout<<"gx"<<gx<<std::endl;
  //       // std::cout<<"ax"<<ax<<std::endl;
  //       printf("\nloopcntx:%d\n", cnt);
  //       break;
  //     }
  //     Eigen::VectorXd px = -Ajk * gx;
  //     double cost1 = 0;
  //     double cost = 0;
  //     std::vector<double> at;
  //     at.resize(size);
  //     Eigen::VectorXd at1 = ax;
  //     tempx = 1;
  //     int looptime = 0;
  //     do {
  //       for (unsigned int i = 0; i < size; ++i) {
  //         at[i] = at1(i);
  //       }
  //       cost = getCost(n, alpha, beta, at, t, x, smooth);  //没更新的
  //       at1 = ax + tempx * px;
  //       for (unsigned int i = 0; i < size; ++i) {
  //         at[i] = at1(i);
  //       }
  //       cost1 = getCost(n, alpha, beta, at, t, x, smooth);  //更新后的

  //       tempx *= 0.5;
  //       looptime++;
  //     } while (cost1 >
  //                  cost + 1e-4 * tempx * (gx.transpose() * px).determinant()
  //                  &&
  //              ros::ok() && looptime < 10);  // Armijo condition
  //     // Eigen::VectorXd px = Ajk*gx;
  //     // double tempd=px.transpose()*Ajk.transpose()*px;
  //     // if(tempd==0){
  //     //     tempx=0;
  //     //     printf("\ntemp=0!!\n");
  //     // }else{
  //     //     tempx=0.1*(gx.transpose()*px).determinant()/tempd;
  //     // }
  //     // // printf("\ntempxorign=%f\t",tempx);
  //     // if(tempx<0)tempx=0;
  //     // if(tempx>0.5)tempx=0.5;
  //     // tempx=0.001;
  //     // printf("tempx=%f\t",tempx);
  //     // printf("\ntemp=%f\n",tempx);
  //     ax = ax + tempx * px;  //得有个步长
  //     // std::cout<<"gx"<<gx<<std::endl;
  //     // std::cout<<"ax"<<ax<<std::endl;
  //     // printf("\nokloop:%d\n",Optiloop);

  //     sx = ax - axl;
  //     cx = gx - gxl;
  //     // std::cout<<"sx"<<sx<<std::endl;
  //     // std::cout<<"cx"<<cx<<std::endl;
  //     // printf("\nokloop1:%d\n",Optiloop);
  //     // double ctsx=(cx.transpose()*sx).determinant();
  //     // if(ctsx==0){
  //     //     // std::cout<<"gx"<<gx<<std::endl;
  //     //     // std::cout<<"ax"<<ax<<std::endl;
  //     //     printf("\nhas0x\n");
  //     //     break;
  //     // }
  //     // printf("\nctsx:%f\n",ctsx);
  //     // printf("\nokloop2:%d\n",Optiloop);
  //     //
  //     std::cout<<sx*cx.transpose()<<std::endl<<cx*sx.transpose()<<std::endl<<sx*sx.transpose()<<std::endl;
  //     // if(ctsx>0){
  //     // Dx = (I - (sx*cx.transpose()) / ctsx) * Dx * (I -
  //     (cx*sx.transpose()) /
  //     // ctsx) + (sx*sx.transpose()) / ctsx; std::cout<<"Dx"<<Dx<<std::endl;
  //     // }

  //     // printf("\nokloop3:%d\n",Optiloop);
  //     ++cnt;
  //   }
  //   // printf("\n");
  //   // std::cout<<"ax"<<ax<<std::endl;
  //   // printf("\ncntx:%d\n",cnt);
  //   cnt = 0;
  //   double tempy = 1;
  //   while (cnt < Optiloop) {
  //     ayl = ay;
  //     gyl = gy;
  //     gy = yfaiklist + Ajk * ay;
  //     if (gy.norm() < 1e-6) {
  //       // std::cout<<"gy"<<gy<<std::endl;
  //       // std::cout<<"ay"<<ay<<std::endl;
  //       printf("\nloopcnty:%d\n", cnt);
  //       break;
  //     }
  //     Eigen::VectorXd py = -Ajk * gy;
  //     double cost1 = 0;
  //     double cost = 0;
  //     std::vector<double> at;
  //     at.resize(size);
  //     Eigen::VectorXd at1 = ay;
  //     // tempy=s.dot(y)/y.dot(y);
  //     int looptime = 0;
  //     tempy = 1;
  //     do {
  //       for (unsigned int i = 0; i < size; ++i) {
  //         at[i] = at1(i);
  //       }
  //       cost = getCost(n, alpha, beta, at, t, y, smooth);  //没更新的
  //       at1 = ay + tempy * py;
  //       for (unsigned int i = 0; i < size; ++i) {
  //         at[i] = at1(i);
  //       }
  //       cost1 = getCost(n, alpha, beta, at, t, y, smooth);  //更新后的
  //       tempy *= 0.5;
  //       looptime++;
  //     } while (cost1 >
  //                  cost + 1e-4 * tempx * (gy.transpose() * py).determinant()
  //                  &&
  //              ros::ok() && looptime < 10);  // Armijo condition
  //     // Eigen::VectorXd py = Ajk*gy;
  //     // double tempd=py.transpose()*Ajk.transpose()*py;
  //     // if(tempd==0){
  //     //     tempy=0;
  //     //     // printf("\ntemp=0!!\n");
  //     // }else{
  //     //     tempy= 0.1*(gy.transpose()*py).determinant()/tempd;
  //     // }
  //     // // printf("\ntempyorign=%f\t",tempy);
  //     // if(tempy<0)tempy=0;
  //     // if(tempy>0.5)tempy=0.5;
  //     // printf("tempy=%f\t",tempy);
  //     ay = ay + tempy * py;
  //     sy = ay - ayl;
  //     cy = gy - gyl;
  //     // double ctsy=(cy.transpose()*sy).determinant();
  //     // if(ctsy==0){
  //     //     // std::cout<<"gy"<<gy<<std::endl;
  //     //     // std::cout<<"ay"<<ay<<std::endl;
  //     //     printf("\nhas0y\n");
  //     //     break;
  //     // }
  //     // if(ctsy>0){
  //     // Dy = (I - (sy*cy.transpose()) / ctsy) * Dy * (I -
  //     (cy*sy.transpose()) /
  //     // ctsy) + (sy*sy.transpose()) / ctsy;
  //     // }
  //     // std::cout<<"Dy"<<Dy<<std::endl;
  //     ++cnt;
  //   }
  //   // printf("\n");
  //   std::cout << "ay" << ay << std::endl;
  //   printf("\ncnty:%d\n", cnt);
  //   for (unsigned int i = 0; i < size; ++i) {
  //     xa[i] = ax(i);
  //     ya[i] = ay(i);
  //   }
  // #endif
}
bool MultistageNode::leastSquare(
    unsigned int n, const std::vector<double> &t, const std::vector<double> &x,
    const std::vector<double> &y, std::vector<double> &rho,
    std::vector<double> &xa, std::vector<double> &ya,
    std::vector<double> &alpha, std::vector<double> &beta) {
  // std::vector<double> alpha;alpha.resize(n+1);
  // std::vector<double> beta;beta.resize(n+1);
  std::vector<double> A;
  A.resize(n + 1, 1);  // 多项式族内积
  // std::vector<double> paramFai;paramFai.resize(n+1);//拟合曲线系数
  // getAlphaAndBeta(n,t,alpha,beta,rho,A);//传递回来就变0了
  // 计算k=0
  double alpha0u = 0;
  double alpha0d = 0;
  for (size_t i = 0; i < t.size(); ++i) {
    alpha0u += rho[i] * t[i];
    alpha0d += rho[i];
  }

  alpha[0] = alpha0u / alpha0d;
  // printf("\na0u:%f\ta0d:%f\ta0:%f\t%f\n",alpha0u,alpha0d,alpha0u/alpha0d,alpha[0]);
  A[0] = alpha0d;
  for (unsigned int k = 1; k < n; ++k) {
    double alphaup = 0;
    double alphadown = 0;
    double betadown = 0;
    for (size_t i = 0; i < t.size(); ++i) {
      double fki = getfaik(alpha, beta, t[i], k);
      double fkli = getfaik(alpha, beta, t[i], k - 1);
      alphaup += rho[i] * t[i] * fki * fki;
      alphadown += rho[i] * fki * fki;
      betadown += rho[i] * fkli * fkli;
    }
    alpha[k] = alphaup / alphadown;
    A[k] = alphadown;
    beta[k - 1] = alphadown / betadown;
  }

  // printf("\nsize:%d\ta0:%f\ta1:%f\ta2:%f\ta3:%f\ta4:%f\ta5:%f\nb0:%f\tb1:%f\tb2:%f\tb3:%f\tb4:%f\n",alpha.size(),alpha[0],alpha[1],alpha[2],alpha[3],alpha[4],alpha[5],beta[0],beta[1],beta[2],beta[3],beta[4]);
  // // std::vector<double> test={1,2,3,4,5};
  // std::vector<double> fai1 = paramfaik2(alpha,beta,1);
  // std::vector<double> fai2 = paramfaik2(alpha,beta,2);
  // std::vector<double> fai3 = paramfaik2(alpha,beta,3);
  // std::vector<double> fai4 = paramfaik2(alpha,beta,4);
  // std::vector<double> fai5 = paramfaik2(alpha,beta,5);
  // printf("\nfai1:%f\t%f\n",fai1[1],fai1[0]);
  // printf("\nfai2:%f\t%f\t%f\n",fai2[2],fai2[1],fai2[0]);
  // printf("\nfai3:%f\t%f\t%f\t%f\n",fai3[3],fai3[2],fai3[1],fai3[0]);
  // printf("\nfai4:%f\t%f\t%f\t%f\t%f\n",fai4[4],fai4[3],fai4[2],fai4[1],fai4[0]);
  // printf("\nfai5:%f\t%f\t%f\t%f\t%f\t%f\n",fai5[5],fai5[4],fai5[3],fai5[2],fai5[1],fai5[0]);
  for (size_t k = 0; k < A.size(); ++k) {
    double xan = 0, yan = 0;
    for (size_t i = 0; i < t.size(); ++i) {
      xan += rho[i] * x[i] * getfaik(alpha, beta, t[i], k);
      yan += rho[i] * y[i] * getfaik(alpha, beta, t[i], k);
    }
    xa[k] = xan / A[k];
    ya[k] = yan / A[k];
  }
  // for(size_t k=0;k<n+1;++k){
  //     faix.push_back()
  // }
}

void MultistageNode::getAlphaAndBeta(unsigned int n,
                                     const std::vector<double> &t,
                                     std::vector<double> &rho,
                                     std::vector<double> &alpha,
                                     std::vector<double> &beta,
                                     std::vector<double> &A) {
  // 计算k=0
  double alpha0u = 0;
  double alpha0d = 0;
  for (size_t i = 0; i < t.size(); ++i) {
    alpha0u += rho[i] * t[i];
    alpha0d += rho[i];
  }

  alpha[0] = alpha0u / alpha0d;
  // printf("\na0u:%f\ta0d:%f\ta0:%f\t%f\n",alpha0u,alpha0d,alpha0u/alpha0d,alpha[0]);
  A[0] = alpha0d;
  for (unsigned int k = 1; k < n; ++k) {
    double alphaup = 0;
    double alphadown = 0;
    double betadown = 0;
    for (size_t i = 0; i < t.size(); ++i) {
      double fki = getfaik(alpha, beta, t[i], k);
      double fkli = getfaik(alpha, beta, t[i], k - 1);
      alphaup += rho[i] * t[i] * fki * fki;
      alphadown += rho[i] * fki * fki;
      betadown += rho[i] * fkli * fkli;
    }
    alpha[k] = alphaup / alphadown;
    A[k] = alphadown;
    beta[k - 1] = alphadown / betadown;
  }
}
double MultistageNode::getfai(std::vector<double> &a,
                              const std::vector<double> &alpha,
                              const std::vector<double> &beta, double t) {
  double result = 0;
  for (size_t k = 0; k < a.size(); ++k) {
    result += a[k] * getfaik(alpha, beta, t, k);
  }
  return result;
}
double MultistageNode::getfaik(const std::vector<double> &alpha,
                               const std::vector<double> &beta, double t,
                               int k) {  // k为fai的下标
  if (k == 0) {
    return 1;
  } else if (k == 1) {
    return t - alpha[0];
  } else {
    double fll = 1;
    double fl = t - alpha[0];
    double f = 0;
    for (int i = 2; i < k + 1; ++i) {
      f = (t - alpha[i - 1]) * fl - beta[i - 2] * fll;
      fll = fl;
      fl = f;
    }  // 得到faik(t)
    return f;
  }
}
std::vector<double> MultistageNode::paramfaik(std::vector<double> &faikl,
                                              std::vector<double> &faikll,
                                              double alphal, double betall) {
  // std::vector<double> faik;faik.resize(faikl.size()+1,0);//k+1个数
  // faik[0] = -alphal*faikl[0];
  // for(size_t i = 0; i < faikl.size()-1; ++i){
  //     faik[i+1] = faikl[i] - alphal*faikl[i+1];
  // }
  // faik.back() = faikl.back();
  // for(size_t i = 0; i < faikll.size(); ++i){
  //     faik[i] -= betall*faikll[i];
  // }
  // return faik;
  std::vector<double> result;
  result.resize(faikl.size() + 1, 0);
  for (unsigned int i = 0; i < faikll.size(); ++i) {
    result[i] -= betall * faikll[i];
    // result[i]+= -alphal*faikl[i-1]-betall*faikll[i-1]+faikl[i];
  }
  for (unsigned int i = 0; i < faikl.size(); ++i) {
    result[i] -= alphal * faikl[i];
  }
  for (unsigned int i = 1; i < result.size(); ++i) {
    result[i] += faikl[i - 1];
  }
  return result;
}
std::vector<double> MultistageNode::paramfaik2(std::vector<double> &alpha,
                                               std::vector<double> &beta,
                                               unsigned int k) {
  std::vector<double> faik;
  faik.resize(k + 1);
  std::vector<double> faikll;
  faikll.resize(1, 1);
  if (k == 0) {
    return faikll;
  }
  std::vector<double> faikl;
  faikl.push_back(-alpha[0]);
  faikl.push_back(1);
  if (k == 1) {
    return faikl;
  } else {
    for (size_t i = 2; i < k + 1; ++i) {
      faik = paramfaik(faikl, faikll, alpha[k - 1], beta[k - 2]);
      faikll = faikl;
      faikl = faik;
    }
    return faik;
  }
}
std::vector<double> MultistageNode::difffaik(std::vector<double> &faik,
                                             unsigned int n) {  // n阶导
  std::vector<double> difffaik;
  if (faik.size() < n + 1) {  // 如果求导阶数高于多项式最高次，则导数为0
    difffaik.resize(1, 0);
    return difffaik;
  }
  difffaik.resize(faik.size() - n);
  unsigned int cnt = n;
  for (unsigned int i = n; i < faik.size(); ++i) {
    difffaik[i - n] = faik[i];
    for (unsigned int cnt = 0; cnt < n; ++cnt) {  // 每个系数前要乘n次数
      difffaik[i - n] *= i - cnt;
    }
  }
  return difffaik;
}
std::vector<double> MultistageNode::faifai(std::vector<double> &fai1,
                                           std::vector<double> &fai2) {
  unsigned int l1 = fai1.size();
  unsigned int l2 = fai2.size();
  std::vector<double> result;
  result.resize(l1 + l2, 0);
  for (unsigned int idx = 0; idx < l2; ++idx) {
    for (unsigned int i = idx; i < idx + l1; ++i) {
      result[i] += fai2[idx] * fai1[i - idx];
    }
  }
  return result;
}
std::vector<double> MultistageNode::paramfai(std::vector<double> &alpha,
                                             std::vector<double> &beta,
                                             std::vector<double> &a,
                                             unsigned int n) {  // 从低次幂开始
  std::vector<double> result;
  result.resize(n + 1, 0);
  for (unsigned int i = 0; i < n + 1; ++i) {
    std::vector<double> faik = paramfaik2(alpha, beta, i);
    for (unsigned int j = 0; j < faik.size(); ++j) {
      result[j] += faik[j] * a[i];
    }
  }
  return result;
}
double MultistageNode::getCost(unsigned int n, std::vector<double> &alpha,
                               std::vector<double> &beta,
                               std::vector<double> &a,
                               const std::vector<double> &t,
                               const std::vector<double> &x, double smooth) {
  double cost = 0;
  // double t_start = pose_history_[0].header.stamp.toSec();
  double t_end = 0;
  for (unsigned int i = 0; i < t.size(); ++i) {
    if (t[i] > t_end) {
      t_end = t[i];
    }
  }
  for (size_t i = 0; i < t.size(); ++i) {
    double ti = t[i];
    double dx = x[i] - getfai(a, alpha, beta, ti);
    cost += dx * dx;
  }
  double smoothx = 0;
  std::vector<double> faix = paramfai(alpha, beta, a, n);
  std::vector<double> faix2order = difffaik(faix, 2);
  std::vector<double> faix2faix2 = faifai(faix2order, faix2order);
  double tn = t_end;
  for (unsigned int i = 0; i < faix2faix2.size(); ++i) {
    smoothx += faix2faix2[i] * tn / (i + 1);
    tn *= t_end;
  }
  cost += smooth * smoothx;
  return cost;
}
void MultistageNode::GoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr
        &msg) {           // 接rviz目标点根据键盘选择发送至robot_x
  if (goal_count == 0) {  // 给robot0发点
    ++goal_count;
    goal_pub_0.publish(*msg);
  } else {  // 给robot1发点
    goal_pub_1.publish(*msg);
    goal_count = 0;
  }
}

double MultistageNode::getDistanceinGeo(
    const geometry_msgs::PoseStamped &pose1,
    const geometry_msgs::PoseStamped &pose2) {
  return hypot(pose1.pose.position.x - pose2.pose.position.x,
               pose1.pose.position.y - pose2.pose.position.y);
}

}  // namespace multistage

void SignalHandler(int signal) {
  if (ros::isInitialized() && ros::isStarted() && ros::ok() &&
      !ros::isShuttingDown()) {
    ros::shutdown();
  }
}
int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  ros::init(argc, argv, "multistage_node", ros::init_options::NoSigintHandler);
  multistage::MultistageNode multistage_node;
  ros::AsyncSpinner async_spinner(6);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}