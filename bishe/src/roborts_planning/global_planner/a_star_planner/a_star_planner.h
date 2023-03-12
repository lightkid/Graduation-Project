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
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>  //new
#include <ros/ros.h>

#include <Eigen/Eigen>

#include "../global_planner_base.h"
#include "alg_factory/algorithm_factory.h"
#include "costmap/costmap_interface.h"
#include "lbfgs.hpp"
#include "proto/a_star_planner_config.pb.h"
#include "roborts_msgs/CarObsInfo.h"
#include "sdqp.hpp"
#include "state/error_code.h"
#include "std_msgs/Int32.h"  //new
// #include "nav_msgs/Path.h"
namespace roborts_global_planner {
inline double sign(double x) { return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0); }

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char CAR_OBSTACLE = 252;
static const unsigned char BUFF_OBSTACLE = 251;
static const unsigned char FREE_SPACE = 0;

/**
 * @brief Global planner alogorithm class for A star under the representation of
 * costmap
 */
class AStarPlanner : public GlobalPlannerBase {
 public:
  /**
   * @brief Constructor of A star planner, set the costmap pointer and relevant
   * costmap size.
   * @param costmap_ptr The shared pointer of costmap interface
   */
  AStarPlanner(CostmapPtr costmap_ptr);
  virtual ~AStarPlanner();
  void allyCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void enemyCallback(const nav_msgs::Odometry::ConstPtr &msg);
  double getDistanceinOdom(const nav_msgs::Odometry &pose1,
                           const nav_msgs::Odometry &pose2);
  /**
   * @brief Main Plan function(override the base-class function)
   * @param start Start pose input
   * @param goal Goal pose input
   * @param path Global plan path output
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo Plan(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      std::vector<geometry_msgs::PoseStamped> &path,
      std::vector<geometry_msgs::PoseStamped> &last_path, bool has_new_goal);

 private:
  /**
   * @brief State enumerate for the cell.
   */
  enum SearchState {
    NOT_HANDLED, /**< The cell is not handled.*/
    OPEN,        /**< The cell is in open priority queue.*/
    CLOSED       /**< The cell is in close queue.*/
  };
  /**
   * @brief Plan based on 1D Costmap list. Input the index in the costmap and
   * get the plan path.
   * @param start_index start pose index in the 1D costmap list
   * @param goal_index goal pose index in the 1D costmap list
   * @param path plan path output
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo SearchPath(
      const int &start_index, const int &goal_index,
      std::vector<geometry_msgs::PoseStamped> &path,
      std::vector<geometry_msgs::PoseStamped> &last_path, bool has_new_goal);
  std::pair<double, double> getVx(nav_msgs::Odometry pose, double start_x_f,
                                  double start_y_f, double goal_x_f,
                                  double goal_y_f, bool &slow);  //动态障碍物
  std::pair<double, double> getVxstatic(double obs_x_f, double obs_y_f,
                                        double start_x_f, double start_y_f,
                                        double goal_x_f, double goal_y_f);
  int GetTheCarNeedToAvoid(std::vector<nav_msgs::Odometry> &carobs,
                           std::vector<geometry_msgs::PoseStamped> &path,
                           double start_x_f, double start_y_f, double goal_x_f,
                           double goal_y_f, double circle_x_f,
                           double circle_y_f, double circle_r2_f);
  bool GetBackPoint(nav_msgs::Odometry pose, double start_x_f, double start_y_f,
                    double goal_x_f, double goal_y_f, unsigned int &tempx,
                    unsigned int &tempy);
  bool GetFrontPoint(nav_msgs::Odometry pose, double start_x_f,
                     double start_y_f, double goal_x_f, double goal_y_f,
                     unsigned int &tempx, unsigned int &tempy);
  bool replan(const int &start_index, const int &goal_index,
              unsigned int &tempx, unsigned int &tempy,
              std::vector<geometry_msgs::PoseStamped> &path);
  bool GetOutFromACar(std::vector<geometry_msgs::PoseStamped> &path,
                      nav_msgs::Odometry pose, double start_x_f,
                      double start_y_f, double goal_x_f, double goal_y_f);
  bool Astar(int &start_index, int &goal_index,
             std::vector<geometry_msgs::PoseStamped> &path);
  bool PathObsCheck(std::vector<geometry_msgs::PoseStamped> &path) const;
  double ObsThreatCheck(std::vector<geometry_msgs::PoseStamped> &path,
                        double start_x_f, double start_y_f, double goal_x_f,
                        double goal_y_f, nav_msgs::Odometry pose);
  int PathHomoCheck(std::vector<geometry_msgs::PoseStamped> &path,
                    int &direction, std::pair<double, double> Vx,
                    std::pair<double, double> Vy, double xk_f, double yk_f);
  /**
   * @brief Calculate the cost for the diagonal or parallel movement.
   * @param current_index Index of the current cell as input
   * @param neighbor_index Index of the neighbor cell as input
   * @param move_cost Movement cost as output
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo GetMoveCost(const int &current_index,
                                        const int &neighbor_index,
                                        int &move_cost) const;
  /**
   * @brief Calculate the Manhattan distance between two cell index used as the
   * heuristic function of A star algorithm.
   * @param index1 Index of the first cell as input
   * @param index2 Index of the second cell as input
   * @param manhattan_distance The Manhattan distance as output
   */
  void GetManhattanDistance(const int &index1, const int &index2,
                            int &manhattan_distance) const;
  double getDistanceinGeo(const geometry_msgs::PoseStamped &pose1,
                          const nav_msgs::Odometry &pose2);
  void GetL2Distance(const int &index1, const int &index2,
                     int &L2_distance) const;
  /**
   * @brief Get the index of nine neighbor cell from the current cell
   * @param current_index Index of the current cell as input
   * @param neighbors_index Index of the neighbor cells as out
   */
  void GetNineNeighbors(const int &current_index,
                        std::vector<int> &neighbors_index) const;

  /**
   * @brief Used for priority queue compare process.
   */
  struct Compare {
    bool operator()(const int &index1, const int &index2) {
      return AStarPlanner::f_score_.at(index1) >
             AStarPlanner::f_score_.at(index2);
    }
  };
  ros::NodeHandle nh_;
  ros::Subscriber ally_sub_;
  ros::Subscriber enemy_sub_;
  ros::Publisher ally_pub_;
  ros::Publisher enemy_pub_;

  Eigen::Matrix<double, 13, 4> Blocks_;  // cx,cy,half_x,half_y
  Eigen::Matrix<double, 10 * 4, 2> Obstacles_;
  double map_length_ = 8.08;
  double map_width_ = 4.48;
  int D_ = 2;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd h_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  int PathSmoother(std::vector<geometry_msgs::PoseStamped> &path,
                   std::vector<geometry_msgs::PoseStamped> &smooth_path);
  void getDistBlocks(const Eigen::Matrix<double, 10 * 4, 2> &Obstacles,
                     const Eigen::Vector2d &pos, const double &safe_radius,
                     Eigen::Matrix<double, 2, 10> &grad,
                     Eigen::Matrix<double, 10, 1> &dist);
  static double smoothFunction(void *instance, const Eigen::VectorXd &x,
                               Eigen::VectorXd &g) {
    //将vec形式的x整合成矩阵形式
    AStarPlanner &obj = *(AStarPlanner *)instance;
    int N = x.size() / obj.D_;
    Eigen::MatrixXd X(N, obj.D_);
    if (X.rows() < 3) {
      return -1;
    }
    for (int i = 0; i < obj.D_; ++i) {
      X.col(i) = x.segment(i * N, N);
    }
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(N - 2, obj.D_);
    //计算平滑函数
    double energyF = 0;
    unsigned int n = X.rows() - 1;
    Eigen::MatrixXd c = obj.P_ * X;
    Eigen::MatrixXd d = obj.Q_ * X;

    energyF = 1 * (4 * (c.transpose() * c).trace() +
                   12 * (c.transpose() * d).trace() +
                   12 * (d.transpose() * d).trace());

    Eigen::MatrixXd xm = X.middleRows(1, n - 1);  //去除x0和xn
    G = obj.H_ * xm + obj.h_;
    //计算碰撞函数
    double weight = 100;  //点太密影响效果
    double potentialF = 0;
    // int sizeObst = obj.obstacles_.cols() / (obj.D_ + 1);
    int sizeXm = xm.rows();
    for (int i = 0; i < sizeXm; ++i) {
      Eigen::Matrix<double, 2, 10> grad;
      Eigen::Matrix<double, 10, 1> dist;
      obj.getDistBlocks(obj.Obstacles_, xm.row(i).transpose(), 0.4, grad, dist);
      for (int j = 0; j < 10; ++j) {
        G.row(i) += weight * grad.col(j).transpose();
        potentialF += weight * dist(j);
      }
    }
    //梯度矩阵变向量
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(N, obj.D_);
    Gm.middleRows(1, N - 2) = G;
    for (int i = 0; i < obj.D_; ++i) {
      g.segment(i * (N), N) = Gm.col(i);
    }
    return energyF + potentialF;
  }
  static int MonitorProgress(void *instance, const Eigen::VectorXd &x,
                             const Eigen::VectorXd &g, const double fx,
                             const double step, const int k, const int ls) {
    // std::cout << std::setprecision(4)
    //   << "================================" << std::endl
    //   << "Iteration: " << k << std::endl
    //   << "Function Value: " << fx << std::endl
    //   << "Gradient Inf Norm: " << std::endl
    //   <<g<< std::endl
    //   << "step: " << std::endl
    //   << step << std::endl;
    return 0;
  }

  // //test
  // ros::Publisher pathnow_pub_;
  // ros::Publisher pathlast_pub_;
  // ros::Publisher pathreplan_pub_;
  // std::vector<geometry_msgs::PoseStamped> path_last;
  //       std::vector<geometry_msgs::PoseStamped> path_now;
  //       std::vector<geometry_msgs::PoseStamped> path_replan;
  //       std::vector<geometry_msgs::PoseStamped> nouse;

  // unsigned int i_start_test=0;//mapbianhao
  // unsigned int i_goal_test=0;
  // unsigned int pathreplanstart=0;
  // unsigned int pathreplanend=0;
  // //test_end

  // ros::Publisher lastpathpub;
  //! heuristic_factor_
  float heuristic_factor_;
  //! inaccessible_cost
  unsigned int inaccessible_cost_;
  //! goal_search_tolerance
  unsigned int goal_search_tolerance_;
  //! gridmap height size
  unsigned int gridmap_height_;
  //! gridmap height width
  unsigned int gridmap_width_;
  //! gridmap cost array
  unsigned char *cost_;
  //! search algorithm related f score, f_score = g_score +
  //! heuristic_cost_estimate
  static std::vector<int> f_score_;
  //! search algorithm related g score, which refers to the score from start
  //! cell to current cell
  std::vector<int> g_score_;
  //! vector that indicates the parent cell index of each cell
  std::vector<int> parent_;
  //! vector that indicates the state of each cell
  std::vector<AStarPlanner::SearchState> state_;
  // std::vector<geometry_msgs::PoseStamped> last_path;
  std::vector<std::pair<int, int>> block_fields_record;  //记录静态障碍物区位置
  std::vector<std::pair<int, int>> special_block;  // new用于解决卡障碍物里时处理
  ros::Publisher relocalization_rqt_pub_;

  std::vector<int> block_state;  //记录静态障碍物区状态/0-未包含1-包含
  std::vector<std::pair<int, int>>
      car_obs_record;  //记录动态障碍物位置，还得记录速度方向
  nav_msgs::Odometry allypose_;
  roborts_msgs::CarObsInfo carinfo_;  //三个/不单独区分友方和敌方了
  //   std::vector<int> obs_car_state_;//三个
  std::vector<nav_msgs::Odometry> carobs;  //三个
  //   std::vector<std::chrono::steady_clock::time_point>
  //   enemy_time_;//车障碍物更新时间/三个

  //   int enemy_num=0;//记录有几个敌人
  //卡尔曼滤波类2维
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
      dt_ = 0.05;
      q0 = 0.0001;
      q3 = 0.0001;
      r0 = 0.1;
      r3 = 0.2;
    }
    ~carKalman() = default;
    // bool first=true;
    void Setdt(double dt) { dt_ = dt; }
    void SetX(nav_msgs::Odometry pose) {
      Xx[0] = pose.pose.pose.position.x;
      Xx[1] = pose.twist.twist.linear.x;
      Xy[0] = pose.pose.pose.position.y;
      Xy[1] = pose.twist.twist.linear.y;
      // update pose_
      pose_.pose.pose.position.x = Xx[0];
      pose_.twist.twist.linear.x = Xx[1];
      pose_.pose.pose.position.y = Xy[0];
      pose_.twist.twist.linear.y = Xy[1];
      // init Z
      Zx[0] = Xx[0];
      Zx[1] = Xx[1];
      Zy[0] = Xy[0];
      Zy[1] = Xy[1];
    }
    void predict() {                // X=AX
      Xx[0] = Xx[0] + dt_ * Xx[1];  // X=AX
      Xy[0] = Xy[0] + dt_ * Xy[1];  // X=AX
      // update pose_
      pose_.pose.pose.position.x = Xx[0];
      pose_.twist.twist.linear.x = Xx[1];
      pose_.pose.pose.position.y = Xy[0];
      pose_.twist.twist.linear.y = Xy[1];
    }
    void updateX() {  // X=X+K(Z-X)
      double Xx0 = Xx[0], Xx1 = Xx[1], Xy0 = Xy[0], Xy1 = Xy[1];
      Xx[0] = Xx0 + K[0] * (Zx[0] - Xx0) + K[1] * (Zx[1] - Xx1);
      Xx[1] = Xx1 + K[2] * (Zx[0] - Xx0) + K[3] * (Zx[1] - Xx1);

      Xy[0] = Xy0 + K[0] * (Zy[0] - Xy0) + K[1] * (Zy[1] - Xy1);
      Xy[1] = Xy1 + K[2] * (Zy[0] - Xy0) + K[3] * (Zy[1] - Xy1);
      // update pose_
      pose_.pose.pose.position.x = Xx[0];
      pose_.twist.twist.linear.x = Zx[1];  //用TD的速度
      pose_.pose.pose.position.y = Xy[0];
      pose_.twist.twist.linear.y = Zy[1];  //用TD的速度
    }
    void updateZ(nav_msgs::Odometry pose) {  // ok
      TD(pose.pose.pose.position.x, Zx[0], Zx[1], dt_);
      TD(pose.pose.pose.position.y, Zy[0], Zy[1], dt_);
      Zx[0] = Xx[0];
      Zy[0] = Xy[0];  //位置用有噪声的原数据，速度用TD的
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
      x2 = x2last + h * fhan(x1last - v, x2last, 100,
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
  };
  std::vector<carKalman> carKalContainer;
};

std::vector<int> AStarPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(
    GlobalPlannerBase, "a_star_planner", AStarPlanner,
    std::shared_ptr<roborts_costmap::CostmapInterface>);

}  // namespace roborts_global_planner

#endif  // ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
