#ifndef ROBORTS_DECISION_OPEN_DAY_GOAL_FACTORY_H
#define ROBORTS_DECISION_OPEN_DAY_GOAL_FACTORY_H

#include "../behavior_tree/behavior_tree.h"
#include "../behavior_modes.h"
//#include "test_goal_factory.h"


namespace roborts_decision {

/*
 *
 */
class OpenDayRootNode : public roborts_decision::SelectorNode {
public:
  OpenDayRootNode(std::string name, ChassisExecutor *&chassis_executor,
                       const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                       const std::string &proto_file_path);

  virtual ~OpenDayRootNode() = default;

  // A mounting method in order to mount add children behavior nodes, called after construction
  void Load();

protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
//  virtual void OnInitialize();

  /**
   * @brief Tick the node, update the robot information and the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update();

private:
  bool HasBullet();

  //! Chassis Executor pointer
  ChassisExecutor *chassis_executor_;

  //! Proto File Path
  std::string proto_file_path_;

  //! Node Handle
  ros::NodeHandle nh_;

  //! Service Clients
  ros::ServiceClient check_bullet_client_;

  //TODO: We need to find a place to store and update all robot information, in Root Node or Goal Factory.
  //! Robot status information
  bool has_ammo_;
  bool enemy_detected_;
  bool has_buff_;
  bool under_attack_;
  int under_attack_board_;
  ros::Time under_attack_time_;
  int hp_;
  BehaviorMode current_behavior_mode_;
  int reload_time_;
  int buff_time_;
//  int robot_id_;
};


/**
 *
 */
class OpenDayPatrolActionNode : public roborts_decision::ActionNode {
public:
  OpenDayPatrolActionNode(std::string name, ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  ~OpenDayPatrolActionNode() = default;

protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize();

  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update();

  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state);

private:
  //!
  roborts_decision::OpenDayPatrolBehavior open_day_patrol_behavior_;

  //! Blackboard Raw Pointer
//  Blackboard* blackboard_raw_ptr_;
};

/**
 *
 */
class OpenDayChaseActionNode : public roborts_decision::ActionNode {
public:
  OpenDayChaseActionNode(std::string name, ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  ~OpenDayChaseActionNode() = default;

protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize();

  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update();

  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state);

private:
  //!
  roborts_decision::OpenDayChaseBehavior open_day_chase_behavior_;

  //! Blackboard Raw Pointer
//  Blackboard* blackboard_raw_ptr_;
};





/*
 *
 */
class OpenDayGoalFactory {
public:
  OpenDayGoalFactory(ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                          const std::string &proto_file_path);

  void Run();

  ~OpenDayGoalFactory() = default;

private:
  bool CtrlFricWheel(bool to_open) {
    
    roborts_msgs::FricWhl ctrl_fricwheel_srv;
    ctrl_fricwheel_srv.request.open = to_open;
    if (ctrl_fricWheel_client_.call(ctrl_fricwheel_srv)) {
      ROS_INFO("service returned true");
      return true;
    } else {
      return false;
    }
  }
  std::shared_ptr<OpenDayRootNode> root_;

  //! behavior_tree
  BehaviorTree behavior_tree_;
  ros::NodeHandle nh_;
  //! executor
  //  ChassisExecutor *const chassis_executor_;

  //! perception information
  Blackboard::Ptr blackboard_ptr_;
  ros::ServiceClient ctrl_fricWheel_client_;

};
}

#endif //ROBORTS_DECISION_OPEN_DAY_GOAL_FACTORY_H
