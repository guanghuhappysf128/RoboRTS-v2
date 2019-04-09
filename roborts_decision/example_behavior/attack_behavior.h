#ifndef ROBORTS_DECISION_ATTACK_BEHAVIOR_H
#define ROBORTS_DECISION_ATTACK_BEHAVIOR_H


#include "io/io.h"
#include <ros/ros.h>

#include "roborts_msgs/TwistAccel.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"


namespace roborts_decision {
class AttackBehavior {
 public:
  AttackBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


          if (!LoadParam(proto_file_path)) {
                ROS_ERROR("%s can't open file", __FUNCTION__);
          }

          
      }

  void Run() {
    ROS_WARN("Shaking z is %.5f", shaking_.twist.angular.z);
    chassis_executor_->Execute(shaking_);
    ros::Duration(shaking_time_).sleep();
    shaking_.twist.angular.z = shaking_.twist.angular.z * (-1);
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    shaking_.twist.linear.x = 0;
    shaking_.twist.linear.y = 0;
    shaking_.twist.linear.z = 0;
    shaking_.twist.angular.x = 0;
    shaking_.twist.angular.y = 0;
    shaking_.twist.angular.z = decision_config.shaking_info().rotation_z_speed();

    shaking_.accel.linear.x = 0;
    shaking_.accel.linear.y = 0;
    shaking_.accel.linear.z = 0;
    shaking_.accel.angular.x = 0;
    shaking_.accel.angular.y = 0;
    shaking_.accel.angular.z = 0;

    shaking_time_ = decision_config.shaking_info().delta_time();

    return true;
  }

  ~AttackBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! planning goal
  //geometry_msgs::PoseStamped planning_goal_;
  roborts_msgs::TwistAccel shaking_;
  float shaking_time_;

};
}

#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
