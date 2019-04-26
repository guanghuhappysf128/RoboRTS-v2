#include "test_goal_factory.h"

namespace roborts_decision {

TestGoalFactory::TestGoalFactory(ChassisExecutor *&chassis_executor, const Blackboard::Ptr &blackboard_ptr,
                                 const std::string &proto_file_path) :
  root_(new DecisionRootNode("root", chassis_executor, blackboard_ptr, proto_file_path)),
  behavior_tree_(root_, 50) {
  ROS_INFO("Test Goal Factory Done!");
}

void TestGoalFactory::Run() {
  root_->Load();
  behavior_tree_.Run();
}

/**
 *
 */
DecisionRootNode::DecisionRootNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                   const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                   const std::string &proto_file_path) :
  SelectorNode::SelectorNode(name, blackboard_ptr),
  chassis_executor_(chassis_executor),
  proto_file_path_(proto_file_path),
  enemy_detected_(false),
  has_ammo_(true),
  has_last_position_(false) {
  check_bullet_client_ = nh_.serviceClient<roborts_sim::CheckBullet>("/check_bullet");
}

void DecisionRootNode::Load() {
  std::shared_ptr<PreconditionNode> to_reload(
    new PreconditionNode("prec_to_reload", blackboard_ptr_, [&]() -> bool { return !this->has_ammo_; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> to_shoot(
    new PreconditionNode("prec_to_shoot", blackboard_ptr_, [&]() -> bool { return this->has_ammo_ && this->enemy_detected_; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> to_search(
    new PreconditionNode("prec_to_search", blackboard_ptr_, [&]() -> bool { return this->has_ammo_ && !this->enemy_detected_ && this->has_last_position_; }, AbortType::LOW_PRIORITY));

  to_reload->SetChild(std::shared_ptr<ReloadActionNode>(new ReloadActionNode("to_reloading_action", chassis_executor_, blackboard_ptr_, proto_file_path_)));
  to_shoot->SetChild(std::shared_ptr<ShootActionNode>(new ShootActionNode("to_shooting_action", chassis_executor_, blackboard_ptr_, proto_file_path_)));
  to_search->SetChild(std::shared_ptr<SearchActionNode>(new SearchActionNode("to_searching_action", chassis_executor_, blackboard_ptr_, proto_file_path_)));

  std::shared_ptr<PatrolActionNode> patrol_action(new PatrolActionNode("to_patrol_action", chassis_executor_, blackboard_ptr_, proto_file_path_));

  AddChildren(to_reload);
  AddChildren(to_shoot);
  AddChildren(to_search);
  AddChildren(patrol_action);
}

BehaviorState DecisionRootNode::Run() {
  if (behavior_state_ != BehaviorState::RUNNING) {
    OnInitialize();
  }

  behavior_state_ = Update();

  if (behavior_state_ != BehaviorState::RUNNING) {
    OnTerminate(behavior_state_);
  }

  return behavior_state_;
}

void DecisionRootNode::OnInitialize() {
  children_node_index_ = children_node_ptr_.size() - 1;

  ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
}

BehaviorState DecisionRootNode::Update() {
  // Update status flags
  has_ammo_ = HasBullet();
  enemy_detected_ = blackboard_ptr_->IsEnemyDetected();

  if (enemy_detected_) {
    has_last_position_ = true;
  }

  SelectorNode::Update();
}

bool DecisionRootNode::HasBullet() {
  roborts_sim::CheckBullet check_bullet_srv;
  check_bullet_srv.request.robot_id = 1; // Really Bad Decision Here
  if (check_bullet_client_.call(check_bullet_srv)) {
    ROS_INFO("Ammo Checked!");
    return (check_bullet_srv.response.remaining_bullet != 0);
  } else {
    ROS_ERROR("Failed to call service checkBullet!");
    return false;
  }
}

/**
 *
 */
ShootActionNode::ShootActionNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                 const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                 const std::string &proto_file_path) :
  ActionNode::ActionNode(name, blackboard_ptr),
  blackboard_raw_ptr_(blackboard_ptr.get()),
  shoot_behavior_(chassis_executor, blackboard_raw_ptr_, proto_file_path) {}

void ShootActionNode::OnInitialize() {
  ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  shoot_behavior_.Run();
}

BehaviorState ShootActionNode::Update() {
  return shoot_behavior_.Update();
}

void ShootActionNode::OnTerminate(roborts_decision::BehaviorState state) {
  shoot_behavior_.Cancel();
  switch (state) {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
      return;
  }
}

/**
 * EscapeActionNode class Definition
 */
EscapeActionNode::EscapeActionNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                   const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                   const std::string &proto_file_path) :
  ActionNode::ActionNode(name, blackboard_ptr),
  blackboard_raw_ptr_(blackboard_ptr.get()),
  escape_behavior_(chassis_executor, blackboard_raw_ptr_, proto_file_path) {}

void EscapeActionNode::OnInitialize() {
  ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  escape_behavior_.Run();
}

BehaviorState EscapeActionNode::Update() {
  return escape_behavior_.Update();
}

void EscapeActionNode::OnTerminate(roborts_decision::BehaviorState state) {
  escape_behavior_.Cancel();
  switch (state) {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
      return;
  }
}

/**
 * Search behavior mode Definition
 */
SearchActionNode::SearchActionNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                   const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                   const std::string &proto_file_path) :
  ActionNode::ActionNode(name, blackboard_ptr),
  blackboard_raw_ptr_(blackboard_ptr.get()),
  search_behavior_(chassis_executor, blackboard_raw_ptr_, proto_file_path) {}

void SearchActionNode::OnInitialize() {
  ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  search_behavior_.SetLastPosition(blackboard_ptr_->GetEnemy());
  ROS_INFO("Last Position Set!");
  search_behavior_.Run();
}

BehaviorState SearchActionNode::Update() {
  return search_behavior_.Update();
}

void SearchActionNode::OnTerminate(roborts_decision::BehaviorState state) {
  search_behavior_.Cancel();
  switch (state) {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
      return;
  }
}

/**
 * Reload behavior mode Definition
 */
ReloadActionNode::ReloadActionNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                   const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                   const std::string &proto_file_path) :
  ActionNode::ActionNode(name, blackboard_ptr),
  blackboard_raw_ptr_(blackboard_ptr.get()),
  reload_behavior_(chassis_executor, blackboard_raw_ptr_, proto_file_path) {}

void ReloadActionNode::OnInitialize() {
  ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  reload_behavior_.Run();
}

BehaviorState ReloadActionNode::Update() {
  reload_behavior_.Update();
}

void ReloadActionNode::OnTerminate(roborts_decision::BehaviorState state) {
  reload_behavior_.Cancel();
  switch (state) {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
      return;
  }
}

/**
 *
 */
PatrolActionNode::PatrolActionNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                   const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                   const std::string &proto_file_path) :
  ActionNode::ActionNode(name, blackboard_ptr),
  blackboard_raw_ptr_(blackboard_ptr.get()),
  patrol_behavior_(chassis_executor, blackboard_raw_ptr_, proto_file_path) {}

void PatrolActionNode::OnInitialize() {
  ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  patrol_behavior_.Run();
}

BehaviorState PatrolActionNode::Update() {
  patrol_behavior_.Update();
}

void PatrolActionNode::OnTerminate(roborts_decision::BehaviorState state) {
  patrol_behavior_.Cancel();
  switch (state) {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
      return;
  }
}

}

