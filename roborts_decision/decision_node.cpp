#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "blackboard/blackboard.h"
#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"

#include "example_behavior/attack_behavior.h"
#include "goal_factory/test_goal_factory.h"
#include "goal_factory/hierarchical_goal_factory.h"


//void Command();
//char command = '0';

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_node");

  ros::NodeHandle nh;
  //std::string config_name;
  //nh.param<std::string>("decision_config_name",config_name,"decision");

  std::string full_path;
  std::string ns = ros::this_node::getNamespace();
  if (ns.size()>=2){
    ROS_INFO("name space is %s", ns.c_str());
    // for kinetic, substr offset is 2; for melodic, offset is 1
    full_path = ros::package::getPath("roborts_decision") +"/config/decision_" + \
      ns.substr(
            ROS_VERSION_MINOR == 14 ? 1 : 2,
            ns.size()-1) + ".prototxt";
  } else {
    full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
  }
  //std::string full_path = ros::package::getPath("roborts_decision") + "/config/"+config_name+".prototxt";

  ROS_INFO("start decision node");
  auto chassis_executor = new roborts_decision::ChassisExecutor;
//  auto blackboard = new roborts_decision::Blackboard(full_path);
// good change to use shared point with auto delete
  roborts_decision::Blackboard::Ptr      trans(new roborts_decision::Blackboard(full_path));
  ROS_INFO("blackboard is done");

  //roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, trans, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, trans, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, trans, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, trans, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, trans, full_path);
  roborts_decision::GoalBehavior         goal_behavior(chassis_executor, trans);
  //roborts_decision::SimpleDecisionTree   simple_decision_tree(chassis_executor, trans,full_path);
  roborts_decision::ReloadBehavior       reload_behavior(chassis_executor, trans, full_path);
  roborts_decision::ToBuffZoneBehavior   to_buff_zone_behavior(chassis_executor, trans, full_path);
  roborts_decision::ShootBehavior        shoot_behavior(chassis_executor, trans, full_path);
  roborts_decision::TestGoalFactory      test_goal_factory(chassis_executor, trans, full_path);
  roborts_decision::HierarchicalGoalFactory hierarchical_goal_factory(chassis_executor, trans, full_path);
  //roborts_decision::AttackBehavior       attack_behavior(chassis_executor, blackboard, full_path);
  hierarchical_goal_factory.Run();
  return 0;
}
