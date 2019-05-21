#ifndef ROBORTS_DECISION_SIMPLE_DECISION_TREE_H
#define ROBORTS_DECISION_SIMPLE_DECISION_TREE_H

#include <vector>
#include <string>
#include <cmath>
#include <iostream>

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "roborts_sim/CheckBullet.h"
#include "roborts_sim/ReloadCmd.h"
#include "roborts_sim/ShootCmd.h"
#include "roborts_sim/Countdown.h"

#include "roborts_msgs/GimbalAngle.h"


namespace roborts_decision {

/*  enum Decision {
    patrol, shoot, reload
  };

  enum Status {
    Patrolling, shooting, go_to_reload
  };*/

  class SimpleDecisionTree {
  public:
    SimpleDecisionTree(ChassisExecutor *&chassis_executor, Blackboard *&blackboard,
                       const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                             blackboard_(blackboard) {
      patrol_count_ = 0;
      patrol_point_size_ = 0;
      has_bullet = true;
      game_start = true;
      enemy_detected = false;

      if (!LoadParam(proto_file_path)) {
        ROS_ERROR("%s can't open file", __FUNCTION__);
      }

      subs_.push_back(
        nh_.subscribe<roborts_sim::Countdown>("countdown", 1000, &SimpleDecisionTree::GameStateCallback, this));
      subs_.push_back(
        nh_.subscribe<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1000, &SimpleDecisionTree::ArmorDectionCallback,
                                                 this));
      subs_.push_back(
        nh_.subscribe<nav_msgs::Odometry>("gazebo_robot_pose", 1000, &SimpleDecisionTree::SelfPositionCallback, this));

      //check_bullet_client = nh_.serviceClient<roborts_sim::CheckBullet>("/check_bullet");
      shoot_client = nh_.serviceClient<roborts_sim::ShootCmd>("/shoot");
      reload_client = nh_.serviceClient<roborts_sim::ReloadCmd>("/reload");
    }

    void Run() {
      if (game_start) {
        auto executor_state = Update();
        std::cout << "state: " << (int) (executor_state) << std::endl;
        if (has_bullet) {
          if (enemy_detected) {
            ROS_INFO("Enemy detected!");
            // enemy detected, stop patrolling, check bullet, prepare to shoot
            Cancel();
            roborts_sim::CheckBullet check_bullet_srv;
            check_bullet_srv.request.robot_id = 1;
            if (check_bullet_client.call(check_bullet_srv)) {
              has_bullet = (check_bullet_srv.response.remaining_bullet != 0);
              if (has_bullet) {
                // has bullet, shoot
                roborts_sim::ShootCmd shoot_srv;
                shoot_srv.request.robot = 1;
                shoot_srv.request.enemy = 3;
                sleep(0.1);
                if (shoot_client.call(shoot_srv)) {
                  ROS_INFO("Robot 1 attempted to shoot Robot 3");
                } else {
                  ROS_ERROR("Failed to call service Shoot!");
                }
              } else {
                // No bullet, to reloading zone
                //executor_state = BehaviorState::SUCCESS
                if (executor_state != BehaviorState::RUNNING || 1) {
                  Cancel();
                  std::cout << "send reloading zone as goal" << std::endl;
                  chassis_executor_->Execute(reload_goal_);
                }
              }
            } else {
              ROS_ERROR("Failed to call service checkBullet!");
            }
          } else {
            // no enemy detected, enter patrol mode
            if (executor_state != BehaviorState::RUNNING) {
              if (patrol_goals_.empty()) {
                ROS_ERROR("patrol goal is empty");
                return;
              }
              std::cout << "send next patrol goal" << std::endl;
              chassis_executor_->Execute(patrol_goals_[patrol_count_]);
              patrol_count_ = ++patrol_count_ % patrol_point_size_;
            }
          }
        } else {
          // has no bullet, if haven't set reloading zone, set it. If arrived loading zone, request to reload
          ROS_INFO("No Ammo!");
          if (executor_state != BehaviorState::RUNNING) {
            if (std::abs(self_position_.pose.position.x - reload_goal_.pose.position.x) < 0.5 &&
                std::abs(self_position_.pose.position.y - reload_goal_.pose.position.y) < 0.5) {
              roborts_sim::ReloadCmd reload_srv;
              reload_srv.request.robot = 1;
              if (reload_client.call(reload_srv)) {
                if (reload_srv.response.success) {
                  ROS_INFO("Reload Complete!");
                  has_bullet = true;
                } else {
                  ROS_ERROR("Reload Failed!");
                }
              } else {
                ROS_ERROR("Failed to call service Reload!");
              }
            }
          }
        }
      } else {
        ROS_INFO("Pending!");
      }
    }

    void GameStateCallback(const roborts_sim::Countdown::ConstPtr &cdm) {
      if (cdm->gameState == "Game starts!") {
        ROS_INFO("Game Starts!");
        game_start = true;
      } else if (cdm->gameState == "Countdown ends!") {
        ROS_INFO("Game ends!");
        game_start = false;
      }
    }

    void ArmorDectionCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg) {
      if (msg->yaw_angle != 0 || msg->pitch_angle != 0) {
        enemy_detected = true;
      } else {
        enemy_detected = false;
      }
    }

    void SelfPositionCallback(const nav_msgs::Odometry::ConstPtr &msg) {
      self_position_.header = msg->header;
      self_position_.pose.position = msg->pose.pose.position;
      self_position_.pose.orientation = msg->pose.pose.orientation;
    }

/*    void RunPatrol() {
      auto executor_state = Update();
      std::cout << "state: " << (int) (executor_state) << std::endl;

      // will not update patrol target if the last decision is also patrol
      if (executor_state != BehaviorState::RUNNING || previous_decision_ != patrol) {

        if (patrol_goals_.empty()) {
          ROS_ERROR("patrol goal is empty");
          return;
        }

        std::cout << "send goal" << std::endl;
        chassis_executor_->Execute(patrol_goals_[patrol_count_]);
        patrol_count_ = ++patrol_count_ % patrol_point_size_;
      }
      if (previous_decision_ != patrol) {
        Cancel();

        if (patrol_goals_.empty()) {
          ROS_ERROR("patrol goal is empty");
          return;
        }

        std::cout << "send goal" << std::endl;
        chassis_executor_->Execute(patrol_goals_[patrol_count_]);
        patrol_count_ = ++patrol_count_ % patrol_point_size_;
      }

      previous_decision_ = patrol;
    }*/

/*    // TODO: shoot action
    void RunShoot() {
      // Code goes here
      previous_decision_ = shoot;
      return;
    }

    // TODO: reloading action
    void RunReload() {
      // Code goes here
      previous_decision_ = reload;
      return;
    }*/

/*    // Simple decision making, shooting priority > reloading
    Decision MakeDecision() {
      bool can_shoot = CheckShoot();
      bool can_reload = CheckReload();
      if (enemy_detected) {
        if (hasBullet) {
          return shoot;
        } else {
          return
        }
      } else if (can_reload) {
        return reload;
      } else {
        return patrol;
      }
    }*/

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

      // Read Patrol Point Pose information
      patrol_point_size_ = (unsigned int) (decision_config.point().size()) - 1;
      patrol_goals_.resize(patrol_point_size_);
      for (int i = 0; i != patrol_point_size_; i++) {
        patrol_goals_[i].header.frame_id = "map";
        patrol_goals_[i].pose.position.x = decision_config.point(i).x();
        patrol_goals_[i].pose.position.y = decision_config.point(i).y();
        patrol_goals_[i].pose.position.z = decision_config.point(i).z();

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                                decision_config.point(i).pitch(),
                                                                decision_config.point(i).yaw());
        patrol_goals_[i].pose.orientation.x = quaternion.x();
        patrol_goals_[i].pose.orientation.y = quaternion.y();
        patrol_goals_[i].pose.orientation.z = quaternion.z();
        patrol_goals_[i].pose.orientation.w = quaternion.w();
      }

      // Read Reloading Point Pose information
      int i = patrol_point_size_;
      reload_goal_.header.frame_id = "map";
      reload_goal_.pose.position.x = decision_config.point(i).x();
      reload_goal_.pose.position.y = decision_config.point(i).y();
      reload_goal_.pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      reload_goal_.pose.orientation.x = quaternion.x();
      reload_goal_.pose.orientation.y = quaternion.y();
      reload_goal_.pose.orientation.z = quaternion.z();
      reload_goal_.pose.orientation.w = quaternion.w();

      return true;
    }

    ~

    SimpleDecisionTree() = default;

  private:
    //! executor
    ChassisExecutor *const chassis_executor_;

    //! perception information
    Blackboard *const blackboard_;

    //! previous decision
//    Decision previous_decision_;

    //! node handle
    ros::NodeHandle nh_;

    //! patrol buffer
    std::vector <geometry_msgs::PoseStamped> patrol_goals_;
    unsigned int patrol_count_;
    unsigned int patrol_point_size_;

    //!TODO: shoot info
    //roborts_msgs::GimbalAngle shooting_target_;

    //! reload info
    geometry_msgs::PoseStamped reload_goal_;

    bool game_start;
    bool enemy_detected;
    bool has_bullet;
    std::vector <ros::Subscriber> subs_;
    ros::ServiceClient check_bullet_client;
    ros::ServiceClient shoot_client;
    ros::ServiceClient reload_client;

    geometry_msgs::PoseStamped self_position_;

  };
}

#endif //ROBORTS_DECISION_SIMPLE_DECISION_TREE_H

