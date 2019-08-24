#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "blackboard/blackboard.h"
#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/simple_decision_tree.h"
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
      ns.substr(1, ns.size()-1) + ".prototxt";
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
//   auto command_thread= std::thread(Command);
//   ros::Rate rate(10);
//   while(ros::ok()){
//     //patrol_behavior.Run();
//     ros::spinOnce();
//     switch (command) {
//       //back to boot area
// //      case '1':
//         //back_boot_area_behavior.Run();
// //        break;
//         //patrol
//       case '2':
//         patrol_behavior.Run();
//         break;
//         //chase.
//       case '3':
//         chase_behavior.Run();
//         break;
//         //search
//       case '4':
//         search_behavior.Run();
//         break;
//         //escape.
//       case '5':
//         escape_behavior.Run();
//         break;
//         //goal.
//       case '6':
//         //goal_behavior.Run();
//         shoot_behavior.Run();
//         break;
//       case '7':
//         to_buff_zone_behavior.Run();
//         break;
//       case '8':
//         reload_behavior.Run();
//         break;
//       case '9':
//         test_goal_factory.Run();
//         break;
//       case '1':
//         hierarchical_goal_factory.Run();
//         break;
//       case '0':
//         goal_behavior.Run();
//         break;
//       case 27:
//         if (command_thread.joinable()){
//           command_thread.join();
//         }
//         return 0;
//       default:
//       //patrol_behavior.Run();
//         break;
//     }
//     rate.sleep();
//  }


  return 0;
}

// void Command() {

//   while (command != 27) {
//     std::cout << "**************************************************************************************" << std::endl;
//     std::cout << "*********************************please send a command********************************" << std::endl;
// //     << "1: back boot area behavior" << std::endl
//     std::cout << "0: goal behavior" << std::endl
//               << "2: patrol behavior" << std::endl
//               << "3: chase_behavior" << std::endl
//               << "4: search behavior" << std::endl
//               << "5: escape behavior" << std::endl
//               << "6: shoot behavior" << std::endl
//               << "7: to buff behavior" <<std::endl
//               << "8: reload behavior" <<std::endl
//               << "9: Test behavior tree" <<std::endl
//               << "1: Hierarchical behavior tree" <<std::endl
//               << "esc: exit program" << std::endl;
//     std::cout << "**************************************************************************************" << std::endl;
//     std::cout << "> ";
//     std::cin >> command;
//     if (command != '0' && command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7' && command != '8' && command != '9' && command != 27) {
//       std::cout << "please input again!" << std::endl;
//     }
//   }
//} 

