#ifndef ROBORTS_DECISION_OPEN_DAY_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_OPEN_DAY_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class OpenDayPatrolBehavior {
 public:
  OpenDayPatrolBehavior(ChassisExecutor* &chassis_executor,
                 const Blackboard::Ptr &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    //ROS_WARN("Get in open day patrol state!");

    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    blackboard_->change_behavior(BehaviorMode::PATROL);

    //ROS_WARN("BehaviorState is %d", executor_state);
    ROS_WARN("In PPPPPPPPPPPPPPPPPPPPPP mode ");

    if (executor_state != BehaviorState::RUNNING) {
      ROS_WARN("The behavior State is not running, and in Patrolllllllllllllllllllllll mode ");

      if (patrol_goals_.empty()) {
        ROS_ERROR("There is no patorl point has been loaded!");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % (point_size_);

      //ROS_WARN("Adding patrol Goals with patrol_count_ is %d, pos is %f, %f",patrol_count_,patrol_goals_[patrol_count_].pose.position.x,patrol_goals_[patrol_count_].pose.position.y);

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
      ROS_ERROR("Could not load prototext for decision");
      return false;
    }

    point_size_ = (unsigned int)(decision_config.point().size()) - 1;
    //ROS_WARN("patrol points size is %d",point_size_);
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
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

    return true;
  }

  ~OpenDayPatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard::Ptr blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

};
}

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
