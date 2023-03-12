#ifndef MULTISTAGE_NODE_H
#define MULTISTAGE_NODE_H

#include <OsqpEigen/OsqpEigen.h>
#include <math.h>
#include <osqp/osqp.h>
#include <ros/ros.h>
#include <stdio.h>

#include <Eigen/Eigen>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "state/error_code.h"

// #include "roborts_msgs/GameZone.h"
// #include "roborts_msgs/GameZoneArray.h"
#include <geometry_msgs/PoseStamped.h>

#include <random>

#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "roborts_msgs/Allytrajectory.h"
#include "roborts_msgs/CarObsInfo.h"
#include "roborts_msgs/trajectorys.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Float64.h"
namespace multistage {

inline double sign(double x) { return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0); }
class MultistageNode {
 public:
  MultistageNode();
  ~MultistageNode();
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Pose0Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Pose1Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Trajectory0Callback(const nav_msgs::Path::ConstPtr &msg);
  void Trajectory1Callback(const nav_msgs::Path::ConstPtr &msg);
  double getDistanceinGeo(const geometry_msgs::PoseStamped &pose1,
                          const geometry_msgs::PoseStamped &pose2);
  // void BonusZoneCallback(const roborts_msgs::GameZoneArray::ConstPtr &
  // bonus);
 private:
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub;       // 接rviz
  ros::Subscriber pose_sub_0;     // 接rboot0的位置
  ros::Subscriber pose_sub_1;     // 接amclpose
  ros::Publisher goal_pub_0;      // 发给robot_0的目标点
  ros::Publisher goal_pub_1;      // 发给robot_1
  ros::Publisher allypose_pub_0;  // 给robot0发队友点
  ros::Publisher allypose_pub_1;
  ros::Publisher allytrajectory_pub_0;   // 给0发友方
  ros::Publisher allytrajectory_pub_1;   // 给1发友方
  ros::Subscriber allytrajectory_sub_0;  // 接0的轨迹
  ros::Subscriber allytrajectory_sub_1;  // 接1的轨迹
  ros::Publisher enemy_pub_0;            // 给robot0发敌人点
  ros::Publisher enemy_pub_1;
  int goal_count;
  int carColor0;  // 1-red 2-blue
  int carColor1;
  double exclude_self_distance;
  roborts_msgs::CarObsInfo enemy_pose_0;  // robot0的敌人当前位置
  roborts_msgs::CarObsInfo enemy_pose_1;  // robot1的敌人当前位置
  bool CCCstart = false;
  // std::vector<int> buff_state;//buff区的6个位置的状态
  // ros::Subscriber bonus_sub;//接裁判系统
  // ros::Publisher bonus_pub;//发给costmap

  // 最小二乘法测试
  ros::Subscriber odom_fix_sub_;
  void odomfixCallback(const nav_msgs::Odometry::ConstPtr &msg);
  // std::vector<unsigned int> randam10Point(std::vector<unsigned int> &idx);
  void getAlphaAndBeta(unsigned int n, const std::vector<double> &t,
                       std::vector<double> &rho, std::vector<double> &alpha,
                       std::vector<double> &beta, std::vector<double> &A);
  double getfai(std::vector<double> &a, const std::vector<double> &alpha,
                const std::vector<double> &beta, double t);
  double getfaik(const std::vector<double> &alpha,
                 const std::vector<double> &beta, double t, int k);
  std::vector<double> paramfaik(std::vector<double> &faikl,
                                std::vector<double> &faikll, double alphal,
                                double betall);
  std::vector<double> paramfaik2(std::vector<double> &alpha,
                                 std::vector<double> &beta, unsigned int k);
  std::vector<double> difffaik(std::vector<double> &faik, unsigned int n);
  std::vector<double> faifai(std::vector<double> &fai1,
                             std::vector<double> &fai2);
  std::vector<double> paramfai(std::vector<double> &alpha,
                               std::vector<double> &beta,
                               std::vector<double> &a, unsigned int n);
  double getCost(unsigned int n, std::vector<double> &alpha,
                 std::vector<double> &beta, std::vector<double> &a,
                 const std::vector<double> &t, const std::vector<double> &x,
                 double smooth);
  bool leastSquare(unsigned int n, const std::vector<double> &t,
                   const std::vector<double> &x, const std::vector<double> &y,
                   std::vector<double> &rho, std::vector<double> &xa,
                   std::vector<double> &ya, std::vector<double> &alpha,
                   std::vector<double> &beta);
  bool leastSquareSmooth(unsigned int n, const std::vector<double> &t,
                         const std::vector<double> &x,
                         const std::vector<double> &y, std::vector<double> &rho,
                         std::vector<double> &xa, std::vector<double> &ya,
                         std::vector<double> &alpha, std::vector<double> &beta,
                         double smooth, double predict_length);
  std::vector<nav_msgs::Odometry> pose_history_;
  ros::Publisher originpoint_pub_;
  sensor_msgs::PointCloud originpoint_;
  ros::Publisher smoothpath_pub_;
  nav_msgs::Path smoothpath_;
  ros::Publisher mypoint_pub_;
  nav_msgs::Odometry mypoint_;

  std::vector<nav_msgs::Path> predict_points_;

  ros::Publisher smoothpath1_pub_;
  ros::Publisher smoothpath0_pub_;
  nav_msgs::Path smoothpath1_;
  nav_msgs::Path smoothpath0_;

  ros::Publisher trajectorys0_pub_;
  ros::Publisher trajectorys1_pub_;
  roborts_msgs::trajectorys trajectorys0_;
  roborts_msgs::trajectorys trajectorys1_;

  ros::Subscriber enemy_sub0_;
  ros::Subscriber enemy_sub1_;
  ros::Subscriber enemy_geo_sub0_;
  ros::Publisher enemy_geo_pub0_;

  ros::Subscriber odom201_sub_;
  ros::Subscriber odom202_sub_;
  ros::Publisher point201_pub_;
  ros::Publisher point202_pub_;
  ros::Publisher path201_pub_;
  ros::Publisher path202_pub_;
  void odom201Callback(const nav_msgs::Odometry::ConstPtr &msg);
  void odom202Callback(const nav_msgs::Odometry::ConstPtr &msg);
  std::vector<nav_msgs::Odometry> pose_history201_;
  std::vector<nav_msgs::Odometry> pose_history202_;
  sensor_msgs::PointCloud originpoint201_;
  sensor_msgs::PointCloud originpoint202_;
  nav_msgs::Path smoothpath201_;
  nav_msgs::Path smoothpath202_;

  ros::Subscriber start_;
  geometry_msgs::PoseStamped pose0_;
  geometry_msgs::PoseStamped pose1_;
  void StartCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void Enemy0Callback(const nav_msgs::Odometry::ConstPtr &msg);
  void Enemygeo0Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  double getDistanceinOdom(const nav_msgs::Odometry &pose1,
                           const nav_msgs::Odometry &pose2);
  void Enemy1Callback(const nav_msgs::Odometry::ConstPtr &msg);
  // nav_msgs::Odometry kalman_point_;
  class carKalman {
   public:
    carKalman() {
      Xx.resize(2, 0);
      Zx.resize(2, 0);
      Xy.resize(2, 0);
      Zy.resize(2, 0);
      P.resize(4, 0);
      K.resize(4, 0);
      A.resize(4, 0);
      A[0] = 1;
      A[1] = dt_;
      A[3] = 1;
      Q.resize(4, 0);
      Q[0] = q0;
      Q[3] = q3;
      R.resize(4, 0);
      R[0] = r0;
      R[3] = r3;
      cartime = std::chrono::steady_clock::now();
      dt_ = 0.02;
      q0 = 0.01;
      q3 = 0.01;
      r0 = 0.01;
      r3 = 0.02;
    }
    ~carKalman() = default;
    // bool first=true;
    void Setdt(double dt) { dt_ = dt; }
    void SetX(nav_msgs::Odometry pose) {
      Xx[0] = pose.pose.pose.position.x;
      // Xx[1] = pose.twist.twist.linear.x;
      Xy[0] = pose.pose.pose.position.y;
      // Xy[1] = pose.twist.twist.linear.y;
      // update pose_
      pose_.pose.pose.position.x = Xx[0];
      // pose_.twist.twist.linear.x = Xx[1];
      pose_.pose.pose.position.y = Xy[0];
      // pose_.twist.twist.linear.y = Xy[1];
      // init Z
      // Zx[0] = Xx[0];
      // Zx[1] = Xx[1];
      // Zy[0] = Xy[0];
      // Zy[1] = Xy[1];
    }
    void predict() {                // X=AX
      Xx[0] = Xx[0] + dt_ * Xx[1];  // X=AX
      Xy[0] = Xy[0] + dt_ * Xy[1];  // X=AX
      // update pose_
      // pose_.pose.pose.position.x = Xx[0];
      // pose_.twist.twist.linear.x = Xx[1];
      // pose_.pose.pose.position.y = Xy[0];
      // pose_.twist.twist.linear.y = Xy[1];
    }
    void updateX() {  // X=X+K(Z-X)
      double Xx0 = Xx[0], Xx1 = Xx[1], Xy0 = Xy[0], Xy1 = Xy[1];
      Xx[0] = Xx0 + K[0] * (Zx[0] - Xx0) + K[1] * (Zx[1] - Xx1);
      Xx[1] = Xx1 + K[2] * (Zx[0] - Xx0) + K[3] * (Zx[1] - Xx1);

      Xy[0] = Xy0 + K[0] * (Zy[0] - Xy0) + K[1] * (Zy[1] - Xy1);
      Xy[1] = Xy1 + K[2] * (Zy[0] - Xy0) + K[3] * (Zy[1] - Xy1);
      // update pose_
      pose_.pose.pose.position.x = Xx[0];
      pose_.twist.twist.linear.x = Zx[1];  // 用TD的速度
      pose_.pose.pose.position.y = Xy[0];
      pose_.twist.twist.linear.y = Zy[1];  // 用TD的速度
    }
    void updateZ(nav_msgs::Odometry pose) {  // ok
      Zx[0] = pose_.pose.pose.position.x;    // 上一时刻的值
      Zy[0] = pose_.pose.pose.position.y;  // 位置用有噪声的原数据，速度用TD的
      Zx[1] = pose_.twist.twist.linear.x;  // 用TD的速度
      Zy[1] = pose_.twist.twist.linear.y;  // 用TD的速度
      TD(pose.pose.pose.position.x, Zx[0], Zx[1], dt_);
      TD(pose.pose.pose.position.y, Zy[0], Zy[1], dt_);
      // Zx[1]=pose.pose.pose.position.x - Xx[0];
      // Zy[1]=pose.pose.pose.position.y - Xy[0];
      Zx[0] = pose.pose.pose.position.x;
      Zy[0] = pose.pose.pose.position.y;  // 位置用有噪声的原数据，速度用TD的
      // printf("\n__Zx(%f,%f)Zy(%f,%f)__\n",Zx[0],Zx[1],Zy[0],Zy[1]);
    }
    nav_msgs::Odometry GetPose() { return pose_; }
    void updateK() {  // K=P(P+R)^-1
      updateP1();
      double norm = (P[0] + r3) * (P[3] + r0) - P[1] * P[2];
      K[0] = (P[0] * (P[3] + r0) - P[1] * P[2]) / norm;
      K[1] = P[1] * r3 / norm;
      K[2] = P[2] * r0 / norm;
      K[3] = (P[3] * (P[0] + r3) - P[1] * P[2]) / norm;
      // printf("\n__K(%f,%f,%f,%f)__\n",K[0],K[1],K[2],K[3]);
      updateP2();
    }
    void addPose(nav_msgs::Odometry pos) {
      if (pos_list_.size() > N - 1)
        pos_list_.erase(pos_list_.begin());  // 删除第一个
      pos_list_.push_back(pos);
    }
    std::vector<nav_msgs::Odometry> getPoseList() { return pos_list_; }
    std::chrono::steady_clock::time_point cartime;

   private:
    std::vector<double> Xx;  // 2x1
    std::vector<double> Zx;  // 2x1
    std::vector<double> Xy;  // 2x1
    std::vector<double> Zy;  // 2x1
    std::vector<double> P;   // 2x2
    std::vector<double> Q;   // 2x2
    std::vector<double> A;   // 2x2
    std::vector<double> K;   // 2x2
    std::vector<double> R;   // 2x2
    double dt_;
    double q0;
    double q3;
    double r0;
    double r3;
    // double v;
    nav_msgs::Odometry pose_;
    std::vector<nav_msgs::Odometry> pos_list_;  // for lsq
    int N = 30;

    void updateP1() {  // P=APA'+Q
      double P0 = P[0], P1 = P[1], P2 = P[2], P3 = P[3];
      P[0] = P0 + dt_ * (P1 + P2 + dt_ * P3) + q0;
      P[1] = P1 + dt_ * P3;
      P[2] = P2 + dt_ * P3;
      P[3] = P3 + q3;
    }
    void updateP2() {  // P=P-KP
      double P0 = P[0], P1 = P[1], P2 = P[2], P3 = P[3];
      P[0] = P0 - K[0] * P0 - K[1] * P2;
      P[1] = P1 - K[0] * P1 - K[1] * P3;
      P[2] = P2 - K[3] * P2 - K[2] * P0;
      P[3] = P3 - K[3] * P3 - K[2] * P1;
    }
    void TD(double v, double &x1, double &x2, double h) {  // h是积分步长
      double x1last = x1;
      double x2last = x2;
      x1 = x1last + h * x2last;
      x2 = x2last + h * fhan(x1last - v, x2last, 200,
                             2 * h);  // 100代表加速度限制，越大跟踪越好
    }
    double fhan(double x1, double x2, double r, double h) {
      double d = r * h * h;
      double a0 = h * x2;
      double y = x1 + a0;
      double a1 = sqrt(d * d + 8 * d * std::fabs(y));
      double a2 = a0 + sign(y) * (a1 - d) / 2;
      double a = (a0 + y - a2) * fsg(y, d) + a2;
      return -r * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d));
    }
    double fsg(double x1, double x2) {
      return (sign(x1 + x2) - sign(x1 - x2)) / 2;
    }
  } kalman_point_;
  carKalman kalman_point201_, kalman_point202_;
  std::vector<carKalman> carKalContainer0;
  std::vector<carKalman> carKalContainer1;
};
}  // namespace multistage

#endif