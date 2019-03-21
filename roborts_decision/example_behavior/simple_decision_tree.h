#ifndef ROBORTS_DECISION_SIMPLE_DECISION_TREE_H
#define ROBORTS_DECISION_SIMPLE_DECISION_TREE_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {

enum Decision {patrol, shoot, reload};

class SimpleDecisionTree {
  public:
    SimpleDecisionTree(ChassisExecutor* &chassis_executor,Blackboard* &blackboard,const std::string & proto_file_path) : chassis_executor_(chassis_executor),blackboard_(blackboard) {
    patrol_count_ = 0;
    point_size_ = 0;
    reload_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
      }

    }

  void Run() {
    Decision decision = MakeDecision();
    switch(decision){
    case patrol:
      RunPatrol();
      break;
    case shoot:
      RunShoot();
    case reload:
      RunReload();
      break;
    }
  }

  void RunPatrol(){
    auto executor_state = Update();
    std::cout << "state: " << (int)(executor_state) << std::endl;

    // will not update patrol target if the last decision is also patrol
    if (executor_state != BehaviorState::RUNNING  || previous_decision_ != patrol) {

      if (patrol_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
    }
    if (previous_decision_ != patrol){
      Cancel();

      if (patrol_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
    }

    previous_decision_ = patrol;
  }
 
  // TODO: shoot action
  void RunShoot(){
    // Code goes here
    previous_decision_ = shoot;
    return;
  }

  // TODO: reloading action
  void RunReload(){
    // Code goes here
    previous_decision_ = reload;
    return;
  }

  // Simple decision making, shooting priority > reloading
  Decision MakeDecision(){
    bool can_shoot = CheckShoot();
    bool can_reload = CheckReload();
    if(can_shoot){
      return shoot;
    }else if(can_reload){
      return reload;
    }else{
      return patrol;
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

    point_size_ = (unsigned int)(decision_config.point().size());
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

    // TODO:Loading reloading zone info, decision_config?
    //reload_size_ = (unsigned int)(decision_config.reload_point().size());
    //reload_goals_.resize(reload_size_);
   // for (int i = 0; i != reload_size_; i++) {
     // reload_goals_[i].header.frame_id = "reload_map";
      //reload_goals_[i].pose.position.x = decision_config.reload_point(i).x();
      //reload_goals_[i].pose.position.y = decision_config.reload_point(i).y();
      //reload_goals_[i].pose.position.z = decision_config.reload_point(i).z();

      //tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.reload_point(i).roll(),
        //                                                      decision_config.reload_point(i).pitch(),
          //                                                    decision_config.reload_point(i).yaw());
      //reload_goals_[i].pose.orientation.x = quaternion.x();
      //reload_goals_[i].pose.orientation.y = quaternion.y();
      //reload_goals_[i].pose.orientation.z = quaternion.z();
      //reload_goals_[i].pose.orientation.w = quaternion.w();
   // }

    return true;
  } 

  ~SimpleDecisionTree() = default;

private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
 
  //! previous decision
  Decision previous_decision_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

  //!TODO: shoot info
  //roborts_msgs::GimbalAngle shooting_target_;

  //! reload info
  std::vector<geometry_msgs::PoseStamped> reload_goals_;
  unsigned int reload_size_;
  int ammo_;

  //TODO: check whether can reload
  bool CheckReload(){
    // Code goes here
    return false;
  }

  //TODO: check whether can shoot
  bool CheckShoot(){
    // Code goes here
    return false;
  }
};
}

#endif //ROBORTS_DECISION_SIMPLE_DECISION_TREE_H

