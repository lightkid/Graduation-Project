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
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H


#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <tf/transform_listener.h>
// #include "actionlib/server/simple_action_server.h"

// #include "roborts_msgs/GlobalPlannerAction.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "io/io.h"
#include "state/node_state.h"

#include "costmap/costmap_interface.h"
#include "global_planner_base.h"
#include "proto/global_planner_config.pb.h"
#include "global_planner_algorithms.h"
//for msgs
// #include "roborts_msgs/strategy2planning.h"
// #include "roborts_msgs/planning2strategy.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
// #include "std_msgs/Int32.h"

namespace roborts_global_planner{

// inline double sign(double x) {
//   return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
// }
class GlobalPlannerNode{
    public:
        typedef std::shared_ptr<roborts_costmap::CostmapInterface> CostmapPtr;
        typedef std::shared_ptr<tf::TransformListener> TfPtr;
        typedef std::unique_ptr<GlobalPlannerBase> GlobalPlannerPtr;
        GlobalPlannerNode();
        ~GlobalPlannerNode();
    private:
        roborts_common::ErrorInfo Init();
        // void allyCallback(const nav_msgs::Odometry::ConstPtr & msg);
        // void enemyCallback(const nav_msgs::Odometry::ConstPtr & msg);
        // void TD(double v, double &x1, double &x2, double h);
        // double fhan(double x1, double x2, double r, double h);
        // double fsg(double x1,double x2);
        // void StrategyCallback(const roborts_msgs::strategy2planning::ConstPtr& msg);//from strategy
        void RvizCallback(const geometry_msgs::PoseStamped::ConstPtr & goal);
        void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal);//from rvizCB or strategyCB
        void StartPlanning();
        void StopPlanning();
        void PlanThread();
        double GetDistance(const geometry_msgs::PoseStamped& pose1,
                    const geometry_msgs::PoseStamped& pose2);
        double GetAngle(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2);
        void PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path,
                                                        const std::vector<geometry_msgs::PoseStamped> &last_path);
        void SetNodeState(roborts_common::NodeState node_state);
        roborts_common::NodeState GetNodeState();
        void SetErrorInfo(roborts_common::ErrorInfo error_info);
        roborts_common::ErrorInfo GetErrorInfo();
        geometry_msgs::PoseStamped GetGoal();
        void SetGoal(geometry_msgs::PoseStamped goal);
        // double getDistanceinOdom(const nav_msgs::Odometry& pose1,const nav_msgs::Odometry& pose2);
        ros::NodeHandle nh_;
        ros::Publisher path_pub_;//发给local_planner_node
        ros::Publisher last_path_pub_;
        // ros::Publisher path_pub_rviz_;//发给rviz
        // ros::Publisher path_pub_local_;
        ros::Subscriber goal_sub_;//get goal point from rvizCB
        ros::Subscriber rviz_sub_;//get goal point from rviz
        ros::Publisher rviz_pub_;//接到决策点往规划发
        // ros::Subscriber strategy_sub_;//从决策模块得到指令
        // ros::Publisher strategy_pub_;//给决策模块的回传
        ros::Publisher aim_pub_;
        geometry_msgs::PoseStamped current_aim;
        bool has_aim_=false;
        ros::Subscriber buff_sub_;//从裁判系统得到buff刷新的信息
        // ros::Subscriber ally_sub_;
        // ros::Subscriber enemy_sub_;

        ros::Publisher kalman_pub_test_;

        std::chrono::steady_clock::time_point enemy_time;
        std::chrono::steady_clock::time_point enemy_time_last;

        GlobalPlannerPtr global_planner_ptr_;
        TfPtr tf_ptr_;
        CostmapPtr costmap_ptr_;
        std::string selected_algorithm_;
        
        // std::vector<geometry_msgs::PoseStamped> goal_list;
        geometry_msgs::PoseStamped goal_;
        std::mutex goal_mtx_;
        bool pause_;
        nav_msgs::Path path_;
        nav_msgs::Path last_path_;
        bool new_path_;//A*成功了
        bool get_path_fail_;//strategycallback得到的A*的回传确定是否发备选策略目标点
        bool get_path_success_;
        bool new_goal_;//新目标点，为了实现打断功能
        bool has_new_goal;
        //! Thread for global planning progress
        std::thread plan_thread_;
        //! Planning condition variable
        std::condition_variable plan_condition_;
        //! Planning mutex
        std::mutex plan_mutex_;

        //! Global planner node state
        roborts_common::NodeState node_state_;
        //! Global planner node state mutex
        std::mutex node_state_mtx_;
        //! Global planner error infomation
        roborts_common::ErrorInfo error_info_;
        //! Global planner error infomation mutex
        std::mutex error_info_mtx_;

        //! Cycle Duration in microseconds
        std::chrono::microseconds cycle_duration_;
        //! Max retry count
        int max_retries_;
        //! Distance tolerance towards goal
        double goal_distance_tolerance_;
        //! Angle tolerance towards goal
        double goal_angle_tolerance_;

        // int count___=0;

    };
} //namespace roborts_global_planner

#endif //ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H