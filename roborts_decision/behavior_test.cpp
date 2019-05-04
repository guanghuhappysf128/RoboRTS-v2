#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/simple_decision_tree.h"
#include "example_behavior/attack_behavior.h"

void Command();
char command = '0';

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");

  ros::NodeHandle nh;
  //std::string config_name;
  //nh.param<std::string>("decision_config_name",config_name,"decision");

  std::string full_path;
  std::string ns = ros::this_node::getNamespace();
  if (ns.size()>=2){
    ROS_INFO("name space is %s", ns.c_str());
    full_path = ros::package::getPath("roborts_decision") +"/config/decision_" + \
      ns.substr(2, ns.size()-1) + ".prototxt";
  } else {
    full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
  }
  //std::string full_path = ros::package::getPath("roborts_decision") + "/config/"+config_name+".prototxt";

  ROS_INFO("start decision node");
  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
  ROS_INFO("blackboard is done");

  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior         goal_behavior(chassis_executor, blackboard);
  roborts_decision::SimpleDecisionTree   simple_decision_tree(chassis_executor, blackboard,full_path);
  roborts_decision::AttackBehavior       attack_behavior(chassis_executor, blackboard, full_path);


  auto command_thread= std::thread(Command);
  ros::Rate rate(10);
  while(ros::ok()){
    //patrol_behavior.Run();
    ros::spinOnce();
    switch (command) {
      //back to boot area
      case '1':
        back_boot_area_behavior.Run();
        break;
        //patrol
      case '2':
        patrol_behavior.Run();
        break;
        //chase.
      case '3':
        chase_behavior.Run();
        break;
        //search
      case '4':
        search_behavior.Run();
        break;
        //escape.
      case '5':
        escape_behavior.Run();
        break;
        //goal.
      case '6':
        goal_behavior.Run();
        break;
      case '7':
        simple_decision_tree.Run();
        break;
      case '8':
        attack_behavior.Run();
        break;
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
      //patrol_behavior.Run();
        break;
    }
    rate.sleep();
  }


  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "7: simple decision tree" <<std::endl
              << "8: attacking mode" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7' && command != '8' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}

