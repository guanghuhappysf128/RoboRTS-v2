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
class ReloadBehavior {
public:
  ReloadBehavior(ChassisExecutor *&chassis_executor,
                 Blackboard *&blackboard,
                 const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {

    ros::NodeHandle nh;
    reload_Client = nh.serviceClient<roborts_sim::ReloadCmd>("reload");

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
    //cancel_goal_ = true;

    // Get self Robot ID
    std::string ns = ros::this_node::getNamespace();
    if (ns == "//r1") {
      robot_ = 1;
    } else if (ns == "//r3") {
      robot_ = 3;
    } else {
      ROS_WARN("Error happens when checking self Robot ID, namely %s, in function %s", ns.c_str(), __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = chassis_executor_->Update();

    auto robot_map_pose = blackboard_->GetRobotMapPose();

    double distance_to_reloading_zone = pow(robot_map_pose.pose.position.x - reload_spot_.pose.position.x, 2) +
                                        pow(robot_map_pose.pose.position.y - reload_spot_.pose.position.y, 2);

    if (distance_to_reloading_zone <= 0.17) {
      Cancel();
      roborts_sim::ReloadCmd srv;
      srv.request.robot = robot_;
      if (reload_Client.call(srv)) {
        if (srv.response.success) {
          ROS_INFO("Reload succeed!");
          behavior_state_ = BehaviorState::SUCCESS;
          return;
        } else {
          ROS_INFO("Reload failed!");
          behavior_state_ = BehaviorState::FAILURE;
          return;
        }
      } else {
        ROS_WARN("Reloading service failed.");
        behavior_state_ = BehaviorState::FAILURE;
        return;
      }
    }

    if (executor_state != BehaviorState::RUNNING) {
      chassis_executor_->Execute(reload_spot_);
      behavior_state_ = BehaviorState::RUNNING;
      ROS_INFO("Robot %d is heading to reloading zone.", robot_);
    }
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    // Read Reloading Point Pose information
    int i = (unsigned int) (decision_config.point().size()) - 1;
    reload_spot_.header.frame_id = "map";
    reload_spot_.pose.position.x = decision_config.point(i).x();
    reload_spot_.pose.position.y = decision_config.point(i).y();
    reload_spot_.pose.position.z = decision_config.point(i).z();

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                            decision_config.point(i).pitch(),
                                                            decision_config.point(i).yaw());
    reload_spot_.pose.orientation.x = quaternion.x();
    reload_spot_.pose.orientation.y = quaternion.y();
    reload_spot_.pose.orientation.z = quaternion.z();
    reload_spot_.pose.orientation.w = quaternion.w();

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
  Blackboard *const blackboard_;

  //! reload spot
  geometry_msgs::PoseStamped reload_spot_;

  //! cancel flag
//  bool cancel_goal_;

  //!
  BehaviorState behavior_state_;

  //! reloading service client
  ros::ServiceClient reload_Client;

  //! Self robot ID
  int robot_;
};
}

#endif //ROBORTS_DECISION_RELOAD_BEHAVIOR_H