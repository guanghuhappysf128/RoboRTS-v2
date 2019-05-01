#ifndef ROBORTS_DECISION_TOBUFFZONEBEHAVIOR_H
#define ROBORTS_DECISION_TOBUFFZONEBEHAVIOR_H

namespace roborts_decision {
class ToBuffZoneBehavior {
public:
  ToBuffZoneBehavior (ChassisExecutor* &chassis_executor,
                       Blackboard::Ptr &blackboard,
                       const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                              blackboard_(blackboard) {


    buff_zone_position_.header.frame_id = "map";
    buff_zone_position_.pose.orientation.x = 0;
    buff_zone_position_.pose.orientation.y = 0;
    buff_zone_position_.pose.orientation.z = 0;
    buff_zone_position_.pose.orientation.w = 1;

    buff_zone_position_.pose.position.x = 0;
    buff_zone_position_.pose.position.y = 0;
    buff_zone_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

    // Get self Robot ID
    std::string ns = ros::this_node::getNamespace();
    if (ns == "//r1") {
      robot_ = 1;
      enemy_ = 3;
    } else if (ns == "//r3") {
      robot_ = 3;
      enemy_ = 1;
    } else {
      ROS_WARN("Error happens when checking self Robot ID, namely %s, in function %s", ns.c_str(), __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();
    blackboard_->change_behavior(BehaviorMode::TO_BUFF_ZONE);
    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = buff_zone_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = buff_zone_position_.pose.position.y - robot_map_pose.pose.position.y;

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(buff_zone_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_->Execute(buff_zone_position_);

      }
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

    buff_zone_position_.header.frame_id = "map";

    if (robot_ == 1) {
      buff_zone_position_.pose.position.x = decision_config.buff_point_red().x();
      buff_zone_position_.pose.position.z = decision_config.buff_point_red().z();
      buff_zone_position_.pose.position.y = decision_config.buff_point_red().y();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.buff_point_red().roll(),
                                                                       decision_config.buff_point_red().pitch(),
                                                                       decision_config.buff_point_red().yaw());
      buff_zone_position_.pose.orientation = quaternion;
    }
    else {
      buff_zone_position_.pose.position.x = decision_config.buff_point_blue().x();
      buff_zone_position_.pose.position.z = decision_config.buff_point_blue().z();
      buff_zone_position_.pose.position.y = decision_config.buff_point_blue().y();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.buff_point_blue().roll(),
                                                                       decision_config.buff_point_blue().pitch(),
                                                                       decision_config.buff_point_blue().yaw());
      buff_zone_position_.pose.orientation = quaternion;
    }
    
    return true;
  }

  ~ToBuffZoneBehavior() = default;

private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard::Ptr const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped buff_zone_position_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  //! ID for current robot
  int robot_;
  int enemy_;

};
}



#endif //ROBORTS_DECISION_TOBUFFZONEBEHAVIOR_H
