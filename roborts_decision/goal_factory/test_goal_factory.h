#ifndef ROBORTS_DECISION_TESTGOALFACTORY_H
#define ROBORTS_DECISION_TESTGOALFACTORY_H

#include "../behavior_modes.h"
#include "../behavior_tree/behavior_tree.h"

namespace roborts_decision {
/*
 *
 */
enum class BehaviorMode {
  PATROL,
  SEARCH,
  SHOOT,
  RELOAD,
  ESCAPE,
  CHASE,
  TO_BUFF_ZONE,
};

/*
 *
 */
class DecisionRootNode : public roborts_decision::SelectorNode {
public:
  DecisionRootNode(std::string name, ChassisExecutor *&chassis_executor,
                   const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  virtual ~DecisionRootNode() = default;

  // A mounting method in order to mount add children behavior nodes, called after construction
  void Load();

  virtual BehaviorState Run();

protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize();

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
  bool has_last_position_;
};

/*
 *
 */
class ShootActionNode : public roborts_decision::ActionNode {
public:
  ShootActionNode(std::string name, ChassisExecutor *&chassis_executor,
                  const Blackboard::Ptr &blackboard_ptr,
                  const std::string &proto_file_path);

  ~ShootActionNode() = default;

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
  //! shoot behavior mode instance
  roborts_decision::ShootBehavior shoot_behavior_;

  //! Blackboard Raw Pointer
  Blackboard* blackboard_raw_ptr_;
};

/**
 *
 */
class EscapeActionNode : public roborts_decision::ActionNode {
public:
  EscapeActionNode(std::string name, ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  ~EscapeActionNode() = default;

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
  roborts_decision::EscapeBehavior escape_behavior_;

  //! Blackboard Raw Pointer
  Blackboard* blackboard_raw_ptr_;
};

/**
 *
 */
class SearchActionNode : public roborts_decision::ActionNode {
public:
  SearchActionNode(std::string name, ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  ~SearchActionNode() = default;

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
  roborts_decision::SearchBehavior search_behavior_;

  //! Blackboard Raw Pointer
  Blackboard* blackboard_raw_ptr_;
};

/**
 *
 */
class ReloadActionNode : public roborts_decision::ActionNode {
public:
  ReloadActionNode(std::string name, ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  ~ReloadActionNode() = default;

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
  roborts_decision::ReloadBehavior reload_behavior_;

  //! Blackboard Raw Pointer
  Blackboard* blackboard_raw_ptr_;
};

/**
 *
 */
class PatrolActionNode : public roborts_decision::ActionNode {
public:
  PatrolActionNode(std::string name, ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                   const std::string &proto_file_path);

  ~PatrolActionNode() = default;

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
  roborts_decision::PatrolBehavior patrol_behavior_;

  //! Blackboard Raw Pointer
  Blackboard* blackboard_raw_ptr_;
};


/*
 *
 */
class TestGoalFactory {
public:
  TestGoalFactory(ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                  const std::string &proto_file_path);

  void Run();

  ~TestGoalFactory() = default;

private:

  std::shared_ptr<DecisionRootNode> root_;

  //! behavior_tree
  BehaviorTree behavior_tree_;

  //! executor
//  ChassisExecutor *const chassis_executor_;

  //! perception information
//  Blackboard::Ptr const blackboard_;

};

}

#endif //ROBORTS_DECISION_TESTGOALFACTORY_H
