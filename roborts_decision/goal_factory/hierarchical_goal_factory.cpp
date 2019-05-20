#include "hierarchical_goal_factory.h"

namespace roborts_decision {
HierarchicalGoalFactory::HierarchicalGoalFactory(roborts_decision::ChassisExecutor *&chassis_executor,
                                                 const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                                 const std::string &proto_file_path) :
  blackboard_ptr_(blackboard_ptr),
  root_(new HierarchicalRootNode("root", chassis_executor, blackboard_ptr, proto_file_path)),
  behavior_tree_(root_, 100) {
  ROS_INFO("Hierarchical Goal Factory Done!");
}

void HierarchicalGoalFactory::Run() {
  root_->Load();
  // After behavior tree is mounted, we need to wait for launching simulation node
  ros::service::waitForService("/check_bullet", -1);

  //Wating for gamestate signal
  ros::Rate r(10);
  while(ros::ok()) {
    ros::spinOnce();
    int game_status = blackboard_ptr_->get_game_status();
    if (game_status == 4){
      break;
    }
    ROS_INFO("GAME STATUS %d", game_status);
    ROS_INFO("REMAINING TIME %d s", blackboard_ptr_->get_remain_time());
    r.sleep();
  }
  ROS_INFO("Game Starts!");
  behavior_tree_.Run();
}

HierarchicalRootNode::HierarchicalRootNode(std::string name, roborts_decision::ChassisExecutor *&chassis_executor,
                                           const roborts_decision::Blackboard::Ptr &blackboard_ptr,
                                           const std::string &proto_file_path) :
  SelectorNode::SelectorNode(name, blackboard_ptr),
  chassis_executor_(chassis_executor),
  proto_file_path_(proto_file_path),
  enemy_detected_(false),
  has_ammo_(true),
  has_buff_(false),
  under_attack_(false),
  under_attack_board_(-1),
  reload_time_(0),
  buff_time_(0) {
  under_attack_time_ = ros::Time::now();
  check_bullet_client_ = nh_.serviceClient<roborts_sim::CheckBullet>("/check_bullet");

  // Get self Robot ID
//  std::string ns = ros::this_node::getNamespace();
//  if (ns == "//r1") {
//    robot_id_ = 1;
//  } else if (ns == "//r2") {
//    robot_id_ = 2;
//  }else if (ns == "//r3") {
//    robot_id_ = 3;
//  }else if (ns == "//r4") {
//    robot_id_ = 4;
//  } else {
//    ROS_WARN("Error happens when checking self Robot ID, namely %s, in function %s", ns.c_str(), __FUNCTION__);
//  }

}

void HierarchicalRootNode::Load() {
  // Instantiate Action Nodes
  std::shared_ptr<ReloadActionNode> reload(new ReloadActionNode("to_reloading_action", chassis_executor_, blackboard_ptr_, proto_file_path_));
  std::shared_ptr<ShootActionNode>  shoot(new ShootActionNode("to_shooting_action", chassis_executor_, blackboard_ptr_, proto_file_path_));
  std::shared_ptr<SearchActionNode> search(new SearchActionNode("to_searching_action", chassis_executor_, blackboard_ptr_, proto_file_path_));
  std::shared_ptr<ToBuffActionNode> tobuff(new ToBuffActionNode("to_to_buff_action", chassis_executor_, blackboard_ptr_, proto_file_path_));
  std::shared_ptr<ChaseActionNode>  chase(new ChaseActionNode("to_chase_action", chassis_executor_, blackboard_ptr_, proto_file_path_));
  std::shared_ptr<EscapeActionNode> escape(new EscapeActionNode("to_escape_action", chassis_executor_, blackboard_ptr_, proto_file_path_));
  std::shared_ptr<PatrolActionNode> patrol(new PatrolActionNode("to_patrol_action", chassis_executor_, blackboard_ptr_, proto_file_path_));

  // Build first-level precondition nodes
  std::shared_ptr<PreconditionNode> to_reload(new PreconditionNode("prec_to_reload", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::RELOAD; }, AbortType::LOW_PRIORITY));

  std::shared_ptr<PreconditionNode> to_shoot(new PreconditionNode("prec_to_shoot", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::SHOOT; }, AbortType::LOW_PRIORITY));

  std::shared_ptr<PreconditionNode> to_search(new PreconditionNode("prec_to_search", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::SEARCH; }, AbortType::LOW_PRIORITY));

  std::shared_ptr<PreconditionNode> to_to_buff(new PreconditionNode("prec_to_to_buff", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::TO_BUFF_ZONE; }, AbortType::LOW_PRIORITY));

  std::shared_ptr<PreconditionNode> to_chase(new PreconditionNode("prec_to_chase", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::CHASE; }, AbortType::LOW_PRIORITY));

  std::shared_ptr<PreconditionNode> to_escape(new PreconditionNode("prec_to_escape", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::ESCAPE; }, AbortType::LOW_PRIORITY));

  std::shared_ptr<PreconditionNode> to_patrol(new PreconditionNode("prec_to_patrol", blackboard_ptr_,
      [&]() -> bool { return this->current_behavior_mode_ == BehaviorMode::STOP; }, AbortType::LOW_PRIORITY));


  // Build second-level reload selector node and add it to first-level to_reload precondition node
  std::shared_ptr<SelectorNode> reload_sub_root_node(new SelectorNode("sub_root_reload", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> reload_to_search(new PreconditionNode("reload_to_search", blackboard_ptr_,
      [&]() -> bool { return this->has_ammo_; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> reload_to_to_buff(new PreconditionNode("reload_to_to_buff", blackboard_ptr_,
      [&]() -> bool { return !this->has_ammo_ && this->under_attack_ && !this->has_buff_ && this->buff_time_ < 2; }, AbortType::LOW_PRIORITY));
  reload_to_search->SetChild(search);
  reload_to_to_buff->SetChild(tobuff);

  reload_sub_root_node->AddChildren(reload_to_search);
  reload_sub_root_node->AddChildren(reload_to_to_buff);
  reload_sub_root_node->AddChildren(reload);

  to_reload->SetChild(reload_sub_root_node);

  // Build second-level shoot selector node and add it to first-level to_shoot precondition node
  std::shared_ptr<SelectorNode> shoot_sub_root_node(new SelectorNode("sub_root_shoot", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> shoot_to_reload(new PreconditionNode("shoot_to_reload", blackboard_ptr_,
      [&]() -> bool { return !this->has_ammo_ && this->reload_time_ < 2; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> shoot_to_chase(new PreconditionNode("shoot_to_chase", blackboard_ptr_,
      [&]() -> bool { return !this->enemy_detected_; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> shoot_to_escape(new PreconditionNode("shoot_to_escape", blackboard_ptr_,
      [&]() -> bool { return (this->under_attack_ && (this->under_attack_board_ != 0 && this->under_attack_board_ != -1)) ||
                              (!this->has_ammo_ && this->reload_time_ > 1); }, AbortType::LOW_PRIORITY));
  shoot_to_reload->SetChild(reload);
  shoot_to_chase->SetChild(chase);
  shoot_to_escape->SetChild(escape);

  shoot_sub_root_node->AddChildren(shoot_to_reload);
  shoot_sub_root_node->AddChildren(shoot_to_chase);
  shoot_sub_root_node->AddChildren(shoot_to_escape);
  shoot_sub_root_node->AddChildren(shoot);

  to_shoot->SetChild(reload_sub_root_node);

  // Build second-level search selector node and add it to first-level to_search precondition node
  std::shared_ptr<SelectorNode> search_sub_root_node(new SelectorNode("sub_root_search", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> search_to_shoot(new PreconditionNode("search_to_shoot", blackboard_ptr_,
      [&]() -> bool { return this->enemy_detected_; }, AbortType::LOW_PRIORITY));
  search_to_shoot->SetChild(shoot);

  search_sub_root_node->AddChildren(search_to_shoot);
  search_sub_root_node->AddChildren(search);

  to_search->SetChild(search_sub_root_node);

  // Build second-level to_buff_zone selector node and add it to first-level to_to_buff precondition node
  std::shared_ptr<SelectorNode> to_buff_sub_root_node(new SelectorNode("sub_root_to_buff", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> to_buff_to_reload(new PreconditionNode("to_buff_to_reload", blackboard_ptr_,
      [&]() -> bool { return this->has_buff_ && this->reload_time_ < 2; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> to_buff_to_escape(new PreconditionNode("to_buff_to_escape", blackboard_ptr_,
      [&]() -> bool { return (this->under_attack_ && !this->has_buff_) ||
                                (this->has_buff_ && this->reload_time_ > 1) ; }, AbortType::LOW_PRIORITY));
  to_buff_to_reload->SetChild(reload);
  to_buff_to_escape->SetChild(escape);

  to_buff_sub_root_node->AddChildren(to_buff_to_reload);
  to_buff_sub_root_node->AddChildren(to_buff_to_escape);
  to_buff_sub_root_node->AddChildren(tobuff);

  to_to_buff->SetChild(to_buff_sub_root_node);

  // Build second-level chase selector node and add it to first-level to_chase precondition node
  std::shared_ptr<SelectorNode> chase_sub_root_node(new SelectorNode("sub_root_chase", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> chase_to_shoot(new PreconditionNode("chase_to_shoot", blackboard_ptr_,
      [&]() -> bool { return (this->under_attack_board_ == 0 || this->under_attack_board_ == -1) && this->enemy_detected_ ; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> chase_to_escape(new PreconditionNode("chase_to_escape", blackboard_ptr_,
      [&]() -> bool { return this->under_attack_ && (this->under_attack_board_ != 0 && this->under_attack_board_ != -1); }, AbortType::LOW_PRIORITY));
  chase_to_shoot->SetChild(shoot);
  chase_to_escape->SetChild(escape);

  chase_sub_root_node->AddChildren(chase_to_shoot);
  chase_sub_root_node->AddChildren(chase_to_escape);
  chase_sub_root_node->AddChildren(chase);

  to_chase->SetChild(chase_sub_root_node);

  // Build second-level escape selector node and add it to first-level to_escape precondition node
  std::shared_ptr<SelectorNode> escape_sub_root_node(new SelectorNode("sub_root_escape", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> escape_to_reload(new PreconditionNode("escape_to_reload", blackboard_ptr_,
      [&]() -> bool { return !this->has_ammo_ && !this->under_attack_ && this->reload_time_ < 2; }, AbortType::LOW_PRIORITY));
  std::shared_ptr<PreconditionNode> escape_to_search(new PreconditionNode("escape_to_search", blackboard_ptr_,
      [&]() -> bool { return this->has_ammo_ && !this->under_attack_ ; }, AbortType::LOW_PRIORITY));
  escape_to_reload->SetChild(reload);
  escape_to_search->SetChild(search);

  escape_sub_root_node->AddChildren(escape_to_reload);
  escape_sub_root_node->AddChildren(escape_to_search);
  escape_sub_root_node->AddChildren(escape);

  to_escape->SetChild(escape_sub_root_node);

  // Build second-level patrol selector node and add it to first-level to_patrol precondition node
  std::shared_ptr<SelectorNode> patrol_sub_root_node(new SelectorNode("sub_root_patrol", blackboard_ptr_));

  std::shared_ptr<PreconditionNode> patrol_to_shoot(new PreconditionNode("patrol_to_shoot", blackboard_ptr_,
    [&]() -> bool { return this->enemy_detected_ ; }, AbortType::LOW_PRIORITY));
  patrol_to_shoot->SetChild(shoot);

  patrol_sub_root_node->AddChildren(patrol_to_shoot);
  patrol_sub_root_node->AddChildren(patrol);

  to_patrol->SetChild(patrol_sub_root_node);


  // Add first-level precondition nodes to root node
  AddChildren(to_reload);
  AddChildren(to_shoot);
  AddChildren(to_search);
  AddChildren(to_to_buff);
  AddChildren(to_chase);
  AddChildren(to_escape);
  AddChildren(to_patrol);

}

BehaviorState HierarchicalRootNode::Update() {
  // Update status flags
  has_ammo_ = blackboard_ptr_->get_bullet();
  enemy_detected_ = blackboard_ptr_->IsEnemyDetected();

  has_buff_ = blackboard_ptr_->get_bonus();
  hp_ = blackboard_ptr_->get_hp();
  current_behavior_mode_ = blackboard_ptr_->get_behavior_mode();
  reload_time_ = blackboard_ptr_->get_reload_time();
  buff_time_ = blackboard_ptr_->get_bonus_time();

  if (under_attack_) {
    if (under_attack_time_ < blackboard_ptr_->get_damage_timepoint() + ros::Duration(3)) {
      under_attack_board_ = blackboard_ptr_->get_damage_armor();
      under_attack_time_ = blackboard_ptr_->get_damage_timepoint();
    } else {
      under_attack_ = false;
      under_attack_board_ = -1;
      blackboard_ptr_->un_damaged();
    }
  } else {
    if (blackboard_ptr_->is_damaged()) {
      under_attack_ = true;
      under_attack_board_ = blackboard_ptr_->get_damage_armor();
      under_attack_time_ = blackboard_ptr_->get_damage_timepoint();
    }
  }
//
//  if (enemy_detected_) {
//    has_last_position_ = true;
//  }

  std::string current_behavior_output;
  if (current_behavior_mode_ == BehaviorMode::RELOAD) {
    current_behavior_output = "BehaviorMode::RELOAD";
  } else if (current_behavior_mode_ == BehaviorMode::SHOOT) {
    current_behavior_output = "BehaviorMode::SHOOT";
  } else if (current_behavior_mode_ == BehaviorMode::SEARCH) {
    current_behavior_output = "BehaviorMode::SEARCH";
  } else if (current_behavior_mode_ == BehaviorMode::CHASE) {
    current_behavior_output = "BehaviorMode::CHASE";
  } else if (current_behavior_mode_ == BehaviorMode::ESCAPE) {
    current_behavior_output = "BehaviorMode::ESCAPE";
  } else if (current_behavior_mode_ == BehaviorMode::PATROL) {
    current_behavior_output = "BehaviorMode::PATROL";
  } else if (current_behavior_mode_ == BehaviorMode::TO_BUFF_ZONE) {
    current_behavior_output = "BehaviorMode::TO_BUFF_ZONE";
  } else if (current_behavior_mode_ == BehaviorMode::RELOADING) {
    current_behavior_output = "BehaviorMode::RELOADING";
  } else if (current_behavior_mode_ == BehaviorMode::BUFFING) {
    current_behavior_output = "BehaviorMode::BUFFING";
  } else if (current_behavior_mode_ == BehaviorMode::STOP) {
    current_behavior_output = "BehaviorMode::STOP";
  } else {
    current_behavior_output = "Error";
  }

  ROS_INFO("has_ammo: %d, has_buff: %d, hp: %d, current_behavior: %s, under_attack: %d", has_ammo_, has_buff_, hp_, current_behavior_output.c_str(), under_attack_);
  return SelectorNode::Update();
}

// Deprecated
bool HierarchicalRootNode::HasBullet() {
//  roborts_sim::CheckBullet check_bullet_srv;
//  check_bullet_srv.request.robot_id = robot_id_; // Relatively better Decision Here
//  if (check_bullet_client_.call(check_bullet_srv)) {
//    ROS_INFO("Ammo Checked!");
//    return (check_bullet_srv.response.remaining_bullet != 0);
//  } else {
//    ROS_ERROR("Failed to call service checkBullet!");
//    return false;
//  }
  return blackboard_ptr_->get_bullet() != 0;
}

}



