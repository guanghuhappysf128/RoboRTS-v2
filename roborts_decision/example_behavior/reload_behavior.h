#ifndef ROBORTS_DECISION_RELOAD_BEHAVIOR_H
#define ROBORTS_DECISION_RELOAD_BEHAVIOR_H

#include <unistd.h>
#include "math.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

#include <ros/ros.h>
#include "roborts_sim/ReloadCmd.h"
#include "roborts_msgs/ProjectileSupply.h"

namespace roborts_decision {
class ReloadBehavior {
public:
  ReloadBehavior(ChassisExecutor *&chassis_executor,
                 const Blackboard::Ptr &blackboard,
                 const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {

    ros::NodeHandle nh;
    reload_Client = nh.serviceClient<roborts_sim::ReloadCmd>("reload");
//    ns = ros::this_node::getNamespace();

    std::string reload_name_ = "projectile_supply";

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
    //cancel_goal_ = true;

    // Get self Robot ID
//    std::string ns = ros::this_node::getNamespace();
//    if (ns == "//r1") {
//      robot_ = 1;
//    } else if (ns == "//r2") {
//      robot_ = 2;
//    }else if (ns == "//r3") {
//      robot_ = 3;
//    }else if (ns == "//r4") {
//      robot_ = 4;
//    } else {
//      ROS_WARN("Error happens when checking self Robot ID, namely %s, in function %s", ns.c_str(), __FUNCTION__);
//    }

    reload_publisher_ = nh.advertise<roborts_msgs::ProjectileSupply>(reload_name_, 1000);

  }

  void Run() {
      auto executor_state = chassis_executor_->Update();
  //    switch (execution_mode_){
  //     case ExcutionMode::IDLE_MODE:
  //     ROS_INFO("IDLE_MODE running");
  //     break;

  //   case ExcutionMode::GOAL_MODE:
  //       ROS_INFO("goal running");
  //       break;
  

  //   default:
  //     ROS_ERROR("Wrong Execution Mode");
  // }
    

    blackboard_->change_behavior(BehaviorMode::RELOAD);

    auto robot_map_pose = blackboard_->GetRobotMapPose();

    double distance_to_reloading_zone = pow(robot_map_pose.pose.position.x - reload_spot_.pose.position.x, 2) +
                                        pow(robot_map_pose.pose.position.y - reload_spot_.pose.position.y, 2);

    if (executor_state == BehaviorState::SUCCESS) {

      //message calling reloading 
      // it should be called only once per reload, and can't calling when all the reload chance has been consumed
      if (blackboard_->get_supplier_status() == 1 || blackboard_->get_supplier_status() == 2)
      {
        ROS_INFO("Waiting for reloading");
      }
      else
      {
        roborts_msgs::ProjectileSupply ps_msg;
        ps_msg.number = 50;
        reload_publisher_.publish(ps_msg);
      }


      Cancel();
      ros::Rate r(50);
      while(ros::ok()){
        blackboard_->change_behavior(BehaviorMode::RELOADING);
        ros::spinOnce();
        if(blackboard_->get_supplier_status() == 0) {
          ROS_INFO("Reloading Done!");
          blackboard_->reload_once();
          blackboard_->change_behavior(BehaviorMode::RELOAD);
          return;
        }
        else if(blackboard_->get_supplier_status() == 1) {
          ROS_INFO("Supplier Preparing!");
        }
        else if(blackboard_->get_supplier_status() == 2) {
          ROS_INFO("Supplier Supplying!");
        }
        r.sleep();
      }
      /*
      roborts_sim::ReloadCmd srv;
      srv.request.robot = robot_;
      if (reload_Client.call(srv)) {
        if (srv.response.success) {
          ROS_INFO("Reload succeed!");
          behavior_state_ = BehaviorState::SUCCESS;
          blackboard_->change_behavior(BehaviorMode::RELOAD);
          return;
        } else {
          ROS_INFO("Reload failed!");
          behavior_state_ = BehaviorState::FAILURE;
          blackboard_->change_behavior(BehaviorMode::RELOAD);
          return;
        }
      } else {
        blackboard_->change_behavior(BehaviorMode::RELOAD);
        ROS_WARN("Reloading service failed.");
        behavior_state_ = BehaviorState::FAILURE;
        return;
      }
      */
    }

    if (executor_state != BehaviorState::RUNNING) {
      chassis_executor_->Execute(reload_spot_);
      behavior_state_ = BehaviorState::RUNNING;
    }
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
        return false;
    }

    // Read Reloading Point Pose information
    reload_spot_.header.frame_id = "map";
    if (blackboard_->IsRed()) {

        reload_spot_.pose.position.x = decision_config.reload_spot_red().x();
        reload_spot_.pose.position.y = decision_config.reload_spot_red().y();
        reload_spot_.pose.position.z = decision_config.reload_spot_red().z();

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.reload_spot_red().roll(),
                                                                decision_config.reload_spot_red().pitch(),
                                                                decision_config.reload_spot_red().yaw());
        reload_spot_.pose.orientation = quaternion;
    }
    else {

        reload_spot_.pose.position.x = decision_config.reload_spot_blue().x();
        reload_spot_.pose.position.y = decision_config.reload_spot_blue().y();
        reload_spot_.pose.position.z = decision_config.reload_spot_blue().z();

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.reload_spot_blue().roll(),
                                                                decision_config.reload_spot_blue().pitch(),
                                                                decision_config.reload_spot_blue().yaw());
        reload_spot_.pose.orientation = quaternion;
    }
    return true;
  }

//  void execute(geometry_msgs::PoseStamped reload_goal) {
//    chassis_executor_->Execute(reload_goal);
//  }

  void Cancel() {
    chassis_executor_->Cancel();
    behavior_state_ = BehaviorState::IDLE;
//    cancel_goal_ = false;
  }

  BehaviorState Update() {
    return behavior_state_;
  }

  ~ReloadBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;

  //! perception information
  Blackboard::Ptr blackboard_;

  //! reload spot
  geometry_msgs::PoseStamped reload_spot_;

  //! cancel flag
//  bool cancel_goal_;

  //!
  BehaviorState behavior_state_;

  //! reloading service client
  ros::ServiceClient reload_Client;

//  std::string ns;

  ros::Publisher reload_publisher_;
};
}

#endif //ROBORTS_DECISION_RELOAD_BEHAVIOR_H