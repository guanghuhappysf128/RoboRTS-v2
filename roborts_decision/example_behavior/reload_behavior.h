#ifndef ROBORTS_DECISION_RELOAD_BEHAVIOR_H
#define ROBORTS_DECISION_RELOAD_BEHAVIOR_H

#include "io/io.h"
#include <unistd.h>
#include "math.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

#include <ros/ros.h>
#include "roborts_sim/ReloadCmd.h"

namespace roborts_decision {
    class ReloadBehavior{
        public:
        ReloadBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {

            ros::NodeHandle nh;
            reload_Client = nh.serviceClient<roborts_sim::ReloadCmd>("reload");                      
            if (!LoadParam(proto_file_path)) {
                ROS_ERROR("%s can't open file", __FUNCTION__);
            }
            cancel_goal_ = true;
        }

        void Run(){

            auto executor_state = Update();

            auto robot_map_pose = blackboard_->GetRobotMapPose();
            if (executor_state != BehaviorState::RUNNING) {
                auto dx = reload_spot.pose.position.x - robot_map_pose.pose.position.x;
                auto dy = reload_spot.pose.position.y - robot_map_pose.pose.position.y;
                auto yaw = std::atan2(dy, dx);
                if(dx == 0 && dy == 0){
                    if(cancel_goal_){
                        chassis_executor_->Cancel();
                        cancel_goal_ = false;
                    }
                    return;
                }else{
                    auto orientation = tf::createQuaternionMsgFromYaw(yaw);
                    geometry_msgs::PoseStamped reload_goal;
                    reload_goal.pose.orientation = orientation;
                    reload_goal.header.frame_id = "map";
                    reload_goal.header.stamp = ros::Time::now();
                    reload_goal.pose.position.x = reload_spot.pose.position.x;
                    reload_goal.pose.position.y = reload_spot.pose.position.y;
                    reload_goal.pose.position.z = 1;
                    ROS_WARN("This robot wants to reload.");
                    cancel_goal_ = true;

                    std::thread exec_thread(&ReloadBehavior::execute, this, reload_goal);
                    exec_thread.detach();

                    while(cancel_goal_){
                        sleep(1);
                        robot_map_pose = blackboard_->GetRobotMapPose();              
                        if(pow(robot_map_pose.pose.position.x - reload_spot.pose.position.x, 2) +
                        pow(robot_map_pose.pose.position.y - reload_spot.pose.position.y, 2) <= 0.17){                          
                            roborts_sim::ReloadCmd srv;
                            srv.request.robot = robot_id;
                            if (reload_Client.call(srv)){
                                if(srv.response.success){
                                    ROS_INFO("Reload succeed!");
                                    this->Cancel();
                                    return;
                                }
                                else{
                                    ROS_INFO("Reload failed!");
                                    this->Cancel();
                                    return;
                                }
                            }
                            else
                                ROS_WARN("Reload failed.");
                                this->Cancel();
                                    return;
                        }
                    }
                }
            }
        }

        bool LoadParam(const std::string &proto_file_path) {
            roborts_decision::DecisionConfig decision_config;
            if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
                return false;
            }
  
            // Read Reloading Point Pose information
            int i = (unsigned int) (decision_config.point().size()) - 1;
            reload_spot.header.frame_id = "map";
            reload_spot.pose.position.x = decision_config.point(i).x();
            reload_spot.pose.position.y = decision_config.point(i).y();
            reload_spot.pose.position.z = decision_config.point(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                                    decision_config.point(i).pitch(),
                                                                    decision_config.point(i).yaw());
            reload_spot.pose.orientation.x = quaternion.x();
            reload_spot.pose.orientation.y = quaternion.y();
            reload_spot.pose.orientation.z = quaternion.z();
            reload_spot.pose.orientation.w = quaternion.w();

            return true;
        }

        void execute(geometry_msgs::PoseStamped reload_goal){
            chassis_executor_->Execute(reload_goal);
        }
        void Cancel() {
            chassis_executor_->Cancel();
            cancel_goal_ = false;
        }
        BehaviorState Update() {
            return chassis_executor_->Update();
        }
        void SetSpot(geometry_msgs::PoseStamped reload_spot) {
            reload_spot = reload_spot;
        }
        ~ReloadBehavior() = default;

        private:
        //! executor
        ChassisExecutor* const chassis_executor_;

        //! perception information
        Blackboard* const blackboard_;

        //! reload spot
        geometry_msgs::PoseStamped reload_spot;

        //! cancel flag
        bool cancel_goal_;

        ros::ServiceClient reload_Client;

        uint robot_id = 1;
    };
}

#endif //ROBORTS_DECISION_RELOAD_BEHAVIOR_H