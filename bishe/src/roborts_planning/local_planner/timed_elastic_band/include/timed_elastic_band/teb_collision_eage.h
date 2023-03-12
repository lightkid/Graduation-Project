#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_COLLISION_EAGE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_COLLISION_EAGE_H

#include <geometry_msgs/Twist.h>
#include "local_planner/obstacle.h"
#include "local_planner/robot_footprint_model.h"
#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_base_eage.h"
#include "timed_elastic_band/teb_penalties.h"

namespace roborts_local_planner {
class CollisionEdge : public TebUnaryEdgeBase<2, const Obstacle *, TebVertexPose>{//第一个数是information的最大维数，第二个数是measurement的类型，第三个数有点像？
    public:
        CollisionEdge(){
            _measurement = NULL;
        }
        CollisionEdge(double t): t_(t){
            _measurement = NULL;
        }
        void computeError(){
            const TebVertexPose *bandpt = static_cast<const TebVertexPose *>(_vertices[0]);
            double dist = robot_model_->CalculateDistance(bandpt->GetPose(), _measurement);
            _error[0] = PenaltyBoundFromBelow(dist, config_param_->collision_opt().min_obstacle_dist(),
                                            config_param_->optimize_info().penalty_epsilon());
            _error[1] = PenaltyBoundFromBelow(dist, config_param_->collision_opt().inflation_dist(), 0.0);//这里可以操作一下变成非线性的
        }
        void SetParameters(const Config &config_param, const BaseRobotFootprintModel *robot_model, const Obstacle *obstacle) {
            config_param_ = &config_param;
            robot_model_ = robot_model;
            _measurement = obstacle;
  }
    protected:
        const BaseRobotFootprintModel *robot_model_;
        // DataBase obs_;//动态障碍物某时间点对应的位姿/这个用_measurement代替
        double t_;//动态障碍物某时间点对应的时间累计//可以不用
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // roborts_local_planner
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_COLLISION_EAGE_H_