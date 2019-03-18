#include "gimbal_executor.h"
namespace roborts_decision{
GimbalExecutor::GimbalExecutor():execution_mode_(ExecutionMode::IDLE_MODE),
                                 execution_state_(BehaviorState::IDLE){
  ros::NodeHandle nh;
  cmd_gimbal_angle_pub_ = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
  cmd_gimbal_rate_pub_  = nh.advertise<roborts_msgs::GimbalRate>("cmd_gimbal_rate", 1);

}

void GimbalExecutor::Execute(const roborts_msgs::GimbalAngle &gimbal_angle){
  execution_mode_ = ExecutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_.publish(gimbal_angle);
}

void GimbalExecutor::Execute(const roborts_msgs::GimbalRate &gimbal_rate){
  execution_mode_ = ExecutionMode::RATE_MODE;
  cmd_gimbal_rate_pub_.publish(gimbal_rate);
}

BehaviorState GimbalExecutor::Update(){
  switch (execution_mode_){
    case ExecutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExecutionMode::ANGLE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExecutionMode::RATE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel(){
  switch (execution_mode_){
    case ExecutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case ExecutionMode::ANGLE_MODE:
      cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      execution_mode_ = ExecutionMode::IDLE_MODE;
      break;

    case ExecutionMode::RATE_MODE:
      cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      execution_mode_ = ExecutionMode::IDLE_MODE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}
}