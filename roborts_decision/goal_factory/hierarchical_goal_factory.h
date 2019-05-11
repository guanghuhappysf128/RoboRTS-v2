#ifndef ROBORTS_DECISION_HIERARCHICAL_GOAL_FACTORY_H
#define ROBORTS_DECISION_HIERARCHICAL_GOAL_FACTORY_H

#include "../behavior_tree/behavior_tree.h"
#include "../behavior_modes.h"
#include "test_goal_factory.h"


namespace roborts_decision {

/*
 *
 */
class HierarchicalRootNode : public roborts_decision::SelectorNode {
public:
  HierarchicalRootNode(std::string name, ChassisExecutor *&chassis_executor,
                       const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                       const std::string &proto_file_path);

  virtual ~HierarchicalRootNode() = default;

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
  //! Some temporary flags
  bool has_ammo_;
  bool enemy_detected_;
//  bool has_last_position_;
  bool has_buff_;
  bool under_attack_;
  int under_attack_board_;
  ros::Time under_attack_time_;
  int hp_;
  BehaviorMode current_behavior_mode_;
  int reload_time_;
  int buff_time_;
};

/*
 *
 */
class HierarchicalGoalFactory {
public:
  HierarchicalGoalFactory(ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                          const std::string &proto_file_path);

  void Run();

  ~HierarchicalGoalFactory() = default;

private:

  std::shared_ptr<HierarchicalRootNode> root_;

  //! behavior_tree
  BehaviorTree behavior_tree_;

  //! executor
//  ChassisExecutor *const chassis_executor_;

  //! perception information
  Blackboard::Ptr blackboard_ptr_;

};
}

#endif //ROBORTS_DECISION_HIERARCHICAL_GOAL_FACTORY_H
