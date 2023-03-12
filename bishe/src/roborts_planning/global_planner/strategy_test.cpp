#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <ros/ros.h>

#include "state/error_code.h"
#include "io/io.h"
#include "state/node_state.h"


#include "roborts_msgs/strategy2planning.h"
#include "roborts_msgs/planning2strategy.h"

class StrategyTest{//不好使,用rosrun rqt_gui rqt_gui手动发指令比较快
    public:
        StrategyTest(){
            pub_ = nh_.advertise<roborts_msgs::strategy2planning>("/strategy2planning",2);
            sub_ = nh_.subscribe<roborts_msgs::planning2strategy>("/planning2strategy",1,&StrategyTest::StrategyCallback,this);


            StartStrategyTest();
        }
        ~StrategyTest() = default;
        void StrategyCallback(const roborts_msgs::planning2strategy::ConstPtr& msg){
            //在屏幕上显示
        }
        void StrategyTestThread() {
            std::this_thread::sleep_for(std::chrono::microseconds(2000));
            roborts_msgs::strategy2planning strategy_pos;
            strategy_pos.goal_pos_x=5.58;
            strategy_pos.goal_pos_y=2.92;
            strategy_pos.aim_pos_x=5.6;
            strategy_pos.aim_pos_y=1.85;
            strategy_pos.goal_pos_x_2=1.54;
            strategy_pos.goal_pos_y_2=4.54;
            strategy_pos.aim_pos_x_2=0.5;
            strategy_pos.aim_pos_y_2=4.55;
            strategy_pos.chase_status=1;
            strategy_pos.goal_status=1;
            pub_.publish(strategy_pos);
            std::this_thread::sleep_for(std::chrono::microseconds(10000));//10s两个位置调换
            strategy_pos.goal_pos_x_2=5.58;
            strategy_pos.goal_pos_y_2=2.92;
            strategy_pos.aim_pos_x_2=5.6;
            strategy_pos.aim_pos_y_2=1.85;
            strategy_pos.goal_pos_x=1.54;
            strategy_pos.goal_pos_y=4.54;
            strategy_pos.aim_pos_x=0.5;
            strategy_pos.aim_pos_y=4.55;
            strategy_pos.chase_status=1;
            strategy_pos.goal_status=1;
            pub_.publish(strategy_pos);
            std::this_thread::sleep_for(std::chrono::microseconds(10000));//10s两个位置调换
            strategy_pos.goal_pos_x=5.58;
            strategy_pos.goal_pos_y=2.92;
            strategy_pos.aim_pos_x=5.6;
            strategy_pos.aim_pos_y=1.85;
            strategy_pos.goal_pos_x_2=1.54;
            strategy_pos.goal_pos_y_2=4.54;
            strategy_pos.aim_pos_x_2=0.5;
            strategy_pos.aim_pos_y_2=4.55;
            strategy_pos.chase_status=1;
            strategy_pos.goal_status=1;
            pub_.publish(strategy_pos);
            while (ros::ok()) {
                


                std::this_thread::sleep_for(std::chrono::microseconds(2000));//2s发一个新位置
            }
        }
        void StartStrategyTest() {
            strategy_test_thread_ = std::thread(&StrategyTest::StrategyTestThread, this);
        }
        void StopStrategyTest() {
            if (strategy_test_thread_.joinable()) {
                strategy_test_thread_.join();
            }
        }
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;//模拟发strategy2planning消息
        ros::Subscriber sub_;//模拟收planning2strategy消息

        std::thread strategy_test_thread_;
};

void SignalHandler(int signal){
    if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
        ros::shutdown();
    }
}
int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM,SignalHandler);
    ros::init(argc, argv, "strategy_test", ros::init_options::NoSigintHandler);
    StrategyTest strategytest_;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    ros::waitForShutdown();
    return 0;
}