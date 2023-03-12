#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_JERK_EAGE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_JERK_EAGE_H

#include <geometry_msgs/Twist.h>

#include "local_planner/utility_tool.h"

#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_vertex_timediff.h"
#include "timed_elastic_band/teb_penalties.h"
#include "timed_elastic_band/teb_base_eage.h"

namespace roborts_local_planner {

class JerkEdge : public TebMultiEdgeBase<3, double> {
 public:

  JerkEdge() {
    this->resize(7);
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
    const TebVertexPose *pose4 = static_cast<const TebVertexPose *>(_vertices[3]);
    const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);
    const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[5]);
    const TebVertexTimeDiff *dt3 = static_cast<const TebVertexTimeDiff *>(_vertices[6]);

    const Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
    const Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();
    const Eigen::Vector2d diff3 = pose4->GetPose().GetPosition() - pose3->GetPose().GetPosition();
    
    double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
    double sin_theta1 = std::sin(pose1->GetPose().GetTheta());
    double cos_theta2 = std::cos(pose2->GetPose().GetTheta());
    double sin_theta2 = std::sin(pose2->GetPose().GetTheta());
    double cos_theta3 = std::cos(pose3->GetPose().GetTheta());
    double sin_theta3 = std::sin(pose3->GetPose().GetTheta());

    double p1_dx = cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();

    double p2_dx = cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

    double p3_dx = cos_theta3 * diff3.x() + sin_theta3 * diff3.y();
    double p3_dy = -sin_theta3 * diff3.x() + cos_theta3 * diff3.y();

    double vel1_x = p1_dx / dt1->GetDiffTime();
    double vel1_y = p1_dy / dt1->GetDiffTime();
    double vel2_x = p2_dx / dt2->GetDiffTime();
    double vel2_y = p2_dy / dt2->GetDiffTime();
    double vel3_x = p3_dx / dt3->GetDiffTime();
    double vel3_y = p3_dy / dt3->GetDiffTime();


    double dt12 = dt1->GetDiffTime() + dt2->GetDiffTime();
    double dt23 = dt2->GetDiffTime() + dt3->GetDiffTime();


    double acc_x1 = (vel2_x - vel1_x) * 2 / dt12;
    double acc_y1 = (vel2_y - vel1_y) * 2 / dt12;
    double acc_x2 = (vel3_x - vel2_x) * 2 / dt23;
    double acc_y2 = (vel3_y - vel2_y) * 2 / dt23;

    double dt123 = 0.25*dt1->GetDiffTime() + 0.5*dt2->GetDiffTime() + 0.25*dt3->GetDiffTime();
    const double jerk_x = (acc_x2 - acc_x1)  / dt123;
    const double jerk_y = (acc_y2 - acc_y1)  / dt123;

    _error[0] = PenaltyBoundToInterval(jerk_x, config_param_->kinematics_opt().jerk_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(jerk_y, config_param_->kinematics_opt().jerk_lim_y(),
                                       config_param_->optimize_info().penalty_epsilon());

    double omega1 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt1->GetDiffTime();
    double omega2 = g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta()) / dt2->GetDiffTime();
    double omega3 = g2o::normalize_theta(pose4->GetPose().GetTheta() - pose3->GetPose().GetTheta()) / dt3->GetDiffTime();
    double acc_rot1 = (omega2 - omega1) * 2 / dt12;
    double acc_rot2 = (omega3 - omega2) * 2 / dt23;
    const double jerk_rot = (acc_rot2 - acc_rot1) / dt123;
    _error[2] = PenaltyBoundToInterval(jerk_rot, config_param_->kinematics_opt().jerk_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class JerkStartEdge : public TebMultiEdgeBase<3, const geometry_msgs::Twist *> {
 public:

  JerkStartEdge() {
    _measurement = NULL;
    this->resize(5);
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
    const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[3]);
    const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);
    
    Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
    Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();

    double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
    double sin_theta1 = std::sin(pose1->GetPose().GetTheta());
    double cos_theta2 = std::cos(pose2->GetPose().GetTheta());
    double sin_theta2 = std::sin(pose2->GetPose().GetTheta());

    double p1_dx = cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
    double p2_dx = cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

    double vel1_x = _measurement->linear.x;
    double vel1_y = _measurement->linear.y;
    double vel2_x = p1_dx / dt1->GetDiffTime();
    double vel2_y = p1_dy / dt1->GetDiffTime();
    double vel3_x = p2_dx / dt2->GetDiffTime();
    double vel3_y = p2_dy / dt2->GetDiffTime();

    double dt12 = dt1->GetDiffTime();
    double dt23 = dt2->GetDiffTime();
    double acc_x1 = (vel2_x - vel1_x) / dt12;
    double acc_y1 = (vel2_y - vel1_y) / dt12;
    double acc_x2 = (vel3_x - vel2_x) / dt23;
    double acc_y2 = (vel3_y - vel2_y) / dt23;

    double dt123 =  0.5 * dt1->GetDiffTime() + 0.5 *dt2->GetDiffTime();
    double jerk_x = (acc_x2 - acc_x1)  / dt123;
    double jerk_y = (acc_y2 - acc_y1)  / dt123;

    _error[0] = PenaltyBoundToInterval(jerk_x, config_param_->kinematics_opt().jerk_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(jerk_y, config_param_->kinematics_opt().jerk_lim_y(),
                                       config_param_->optimize_info().penalty_epsilon());

    double omega1 = _measurement->angular.z;
    double omega2 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt1->GetDiffTime();
    double omega3= g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta()) / dt2->GetDiffTime();

    double acc_rot1 = (omega2 - omega1) / dt12;
    double acc_rot2 = (omega3 - omega2)  / dt23;

    double jerk_rot = (acc_rot1 - acc_rot2) / dt123;
    _error[2] = PenaltyBoundToInterval(jerk_rot, config_param_->kinematics_opt().jerk_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());
  }

  void setInitialVelocity(const geometry_msgs::Twist &vel_start) {
    _measurement = &vel_start;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class JerkGoalEdge : public TebMultiEdgeBase<3, const geometry_msgs::Twist *> {
 public:

  JerkGoalEdge() {
    _measurement = NULL;
    this->resize(5);
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
    const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[3]);
    const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);
    
    Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
    Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();

    double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
    double sin_theta1 = std::sin(pose1->GetPose().GetTheta());
    double cos_theta2 = std::cos(pose2->GetPose().GetTheta());
    double sin_theta2 = std::sin(pose2->GetPose().GetTheta());

    double p1_dx = cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
    double p2_dx = cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();


    double vel1_x = p1_dx / dt1->GetDiffTime();
    double vel1_y = p1_dy / dt1->GetDiffTime();
    double vel2_x = p2_dx / dt2->GetDiffTime();
    double vel2_y = p2_dy / dt2->GetDiffTime();
    double vel3_x = _measurement->linear.x;
    double vel3_y = _measurement->linear.y;

    double dt12 = dt1->GetDiffTime() + dt2->GetDiffTime();
    double dt23 = dt2->GetDiffTime();
    double acc_x1 = (vel2_x - vel1_x) / dt12;
    double acc_y1 = (vel2_y - vel1_y) / dt12;
    double acc_x2 = (vel3_x - vel2_x) / dt23;
    double acc_y2 = (vel3_y - vel2_y) / dt23;

    double dt123 =  0.5 * dt1->GetDiffTime() + 0.5 * dt2->GetDiffTime();
    double jerk_x = (acc_x2 - acc_x1)  / dt123;
    double jerk_y = (acc_y2 - acc_y1)  / dt123;

    _error[0] = PenaltyBoundToInterval(jerk_x, config_param_->kinematics_opt().jerk_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(jerk_y, config_param_->kinematics_opt().jerk_lim_y(),
                                       config_param_->optimize_info().penalty_epsilon());

    
    double omega1 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt1->GetDiffTime();
    double omega2= g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta()) / dt2->GetDiffTime();
    double omega3 = _measurement->angular.z;

    double acc_rot1 = (omega2 - omega1)  / dt12;
    double acc_rot2 = (omega3 - omega2) / dt23;

    double jerk_rot = (acc_rot1 - acc_rot2) / dt123;
    _error[2] = PenaltyBoundToInterval(jerk_rot, config_param_->kinematics_opt().jerk_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());

  }

  void SetGoalVelocity(const geometry_msgs::Twist &vel_goal) {
    _measurement = &vel_goal;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // roborts_local_planner
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_ACCELERATION_EAGE_H_
