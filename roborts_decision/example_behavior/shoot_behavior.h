#ifndef ROBORTS_DECISION_SHOOTBEHAVIOR_H
#define ROBORTS_DECISION_SHOOTBEHAVIOR_H

#include <vector>
#include <string>

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include <ros/ros.h>

#include "roborts_sim/CheckBullet.h"
#include "roborts_sim/ReloadCmd.h"

#include "roborts_msgs/RobotHeat.h"

namespace roborts_decision{

const int PROJECTILE_SPEED = 25;
const int BARREL_HEAT_LIMIT = 360;
const int BARREL_HEAT_UPPERBOUND = 720;

class ShootBehavior {
public:
  ShootBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    // Load Param from config file (Current: ../config/decision.prototxt)
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

    // Get self Robot ID
    std::string ns = ros::this_node::getNamespace();
    if (ns == "r1") {
      robot = 1;
      enemy_ = 3;
    } else if (ns == "r3") {
      robot = 3;
      enemy_ = 1;
    } else {
      ROS_WARN("Error happens when checking self Robot ID, %2", __FUNCTION__);
    }

    // Service Client Register
    check_bullet_client_ = nh_.serviceClient<roborts_sim::CheckBullet>("/check_bullet");
    shoot_client_ = nh_.serviceClient<roborts_sim::ShootCmd>("/shoot");

    // Topic Subscriber Register
    subs_.push_back(nh_.subscribe<roborts_msgs::RobotHeat>("robot_heat", 30, &ShootBehavior::BarrelHeatCallback, this));

  }

  void Run() {
    if (!HasBullet()) {
      ROS_WARN("I have no ammo, %s", __FUNCTION__);
      return;
    } else {
      if (barrel_heat_ >= BARREL_HEAT_LIMIT - PROJECTILE_SPEED) {
        ROS_INFO("In current mode, Robot's barrel heat won't exceed heat limit.");
      } else {
        ShootEnemy();
      }
    }
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
    // Add Loading statements here
    return true;
  }

  ~ShootBehavior() {}

private:
  bool HasBullet() {
    roborts_sim::CheckBullet check_bullet_srv;
    check_bullet_srv.request.robot_id = robot_;
    if (check_bullet_client_.call(check_bullet_srv)) {
      return (check_bullet_srv.response.remaining_bullet != 0);
    } else {
      ROS_ERROR("Failed to call service checkBullet!");
      return false;
    }
  }

  void ShootEnemy() {
    roborts_sim::ShootCmd shoot_srv;
    shoot_srv.request.robot = robot_;
    shoot_srv.request.enemy = enemy_;
    if (shoot_client.call(shoot_srv)) {
      ROS_INFO("Robot %d attempted to shoot Robot %d", robot_, enemy_);
    } else {
      ROS_ERROR("Failed to call service Shoot!");
    }
  }

  void BarrelHeatCallback(const roborts_msgs::RobotHeat::ConstPtr &robot_heat) {
    barrel_heat_ = robot_heat.shooter_heat;
  }



private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! Node Handle
  ros::NodeHandle nh_;

  //! Service Clients
  ros::ServiceClient check_bullet_client_;
  ros::ServiceClient shoot_client_;

  //! Topic Subscribers
  std::vector <ros::Subscriber> subs_;

  //! ID for current robot
  int robot_;
  int enemy_;

  //! Barrel Heat
  int barrel_heat_;
};
}


#endif //ROBORTS_DECISION_SHOOTBEHAVIOR_H
