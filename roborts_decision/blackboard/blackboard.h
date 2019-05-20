/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/BonusStatus.h"

// todo: #include "roborts_msg"

namespace roborts_decision{

enum class BehaviorMode {
  STOP,
  PATROL,
  SEARCH,
  SHOOT,
  RELOAD,
  ESCAPE,
  CHASE,
  TO_BUFF_ZONE,
  BUFFING,
  RELOADING,
};

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      remaining_time_(-1),
      reload_time(0),
      bonus_time(0),
      hp_(2000),
      bullet_(40),
      bonus_(false) {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    std::string costmap_config_path;

    // find namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns.size() >= 2){
      ROS_INFO("name space is %s", ns.c_str());
      costmap_config_path = "/config/costmap_parameter_config_for_decision_" + \
        ns.substr(2, ns.size()-1) + ".prototxt";
    } else {
      costmap_config_path = "/config/costmap_parameter_config_for_decision.prototxt";
    }
    ROS_INFO("1");
    std::string map_path = ros::package::getPath("roborts_costmap") + \
      costmap_config_path;
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
    ROS_INFO("2");
    // Enemy fake pose
    ros::NodeHandle rviz_nh("move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    if (!decision_config.simulate()){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }

    //Read Config Parameters from config file
    base_link_id_ = decision_config.base_link_id();
    is_red_ = decision_config.is_red();
    bullet_ = decision_config.init_ammo();

    //subscribers
    // why 1000??? can't 100 be enough?
    damage_subscriber          = nh.subscribe("robot_damage", 1000, &Blackboard::damage_callback, this);
    robot_status_subscriber    = nh.subscribe("robot_status", 1000, &Blackboard::robot_status_callback, this);
    game_status_subscriber     = nh.subscribe("game_status", 1000, &Blackboard::game_status_callback, this);
    supplier_status_subscriber = nh.subscribe("field_supplier_status", 1000, &Blackboard::supplier_status_callback, this);
    robot_bonus_subscriber     = nh.subscribe("robot_bonus", 1000, &Blackboard::robot_bonus_callback, this);
    bonus_status_subscriber    = nh.subscribe("field_bonus_status",1000, &Blackboard::bonus_status_callback, this);
  }

  

  ~Blackboard() = default;

  //callbacks
  void damage_callback(const roborts_msgs::RobotDamage& msg){
    is_damaged_ = true;
    damage_armor = msg.damage_source;
    damage_timepoint = ros::Time::now();
  }

  void robot_status_callback(const roborts_msgs::RobotStatus& msg){
    hp_ = msg.remain_hp;
  }

  void game_status_callback(const roborts_msgs::GameStatus& msg){
    game_status = msg.game_status;
    remaining_time_ = msg.remaining_time;
    if(remaining_time_ % 60 == 0){
      reset_reload();
      reset_bonus();
    }
  }

  void supplier_status_callback(const roborts_msgs::SupplierStatus& msg){
    supplier_status = msg.status;
  }
  void robot_bonus_callback(const roborts_msgs::RobotBonus& msg){
    bonus_ = msg.bonus;
  }
  void bonus_status_callback(const roborts_msgs::BonusStatus& msg){
    if(is_red_ == true){
      bonus_zone_status = msg.red_bonus;
    }else{
      bonus_zone_status = msg.blue_bonus;
    }
  }

  //get
  bool IsRed(){
    return is_red_;
  }

  bool is_damaged(){
    return is_damaged_;
  }

  bool get_damage_armor(){
    return damage_armor;
  }
  ros::Time get_damage_timepoint(){
    return damage_timepoint;
  }
  int get_hp(){
    return hp_;
  }
  int get_game_status(){
    return game_status;
  }
  int get_remain_time(){
    return remaining_time_;
  }
  BehaviorMode get_behavior_mode(){
    return current_behavior;
  }
  int get_bullet(){
    return bullet_;
  }
  int get_supplier_status(){
    return supplier_status;
  }
  int get_reload_time(){
    return reload_time;
  }
  bool get_bonus(){
    return bonus_;
  }
  int get_bonus_status(){
    return bonus_zone_status;
  }
  int get_bonus_time(){
    return bonus_time;
  }
  double GetEnemyYaw() {
    return enemy_yaw_;
  }
  //reset value
  void un_damaged(){
    is_damaged_ = false;
  }
  void change_behavior(BehaviorMode b){
    current_behavior = b;
  }
  void reload_once(){
    bullet_ += 50;
    reload_time++;
  }
  void reset_reload(){
    reload_time = 0;
  }
  void bonus_once(){
    bonus_time++;
  }
  void reset_bonus(){
    bonus_time = 0;
  }
  // Bullet Decrease
  void BulletDown(int amount) {
    bullet_ -= (bullet_ == 0) ? 0 : amount;
  }
  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
//      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
          camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan2(camera_pose_msg.pose.position.y, camera_pose_msg.pose.position.x);

      // Get the enemy direction
      if (camera_pose_msg.pose.position.z == 0 || camera_pose_msg.pose.position.y > -0.05 && camera_pose_msg.pose.position.y < 0.05) {
        enemy_yaw_ = 0;
      } else {
        enemy_yaw_ = yaw;
      }

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        ROS_INFO("Enemy x: %f, Enemy y: %f.", global_pose_msg.pose.position.x, global_pose_msg.pose.position.y);
        
        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;

        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
      enemy_yaw_ = 0;
    }

  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
//    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

  std::string GetBaseLinkId()
  {
    return base_link_id_;
  }

 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();
    // This needed to be put into proto file to handle with the namesapce and tf prefix issue
    robot_tf_pose.frame_id_ = base_link_id_;
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;
  double enemy_yaw_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

  // sim
  std::string base_link_id_;

  //! robot info
  bool is_red_;
  int hp_;
  int bullet_;
  bool bonus_ = false;

  //! robot status triggers
  ros::Time damage_timepoint;
  bool is_damaged_ = false;
  int damage_armor = -1;

  //! game status relevant
  int remaining_time_;
  int game_status;  // records game status info, such as Pre_match, Round etc. Check in GameStatus.msg
  int supplier_status; // Projectile Supplier Status, namely Preparing, Supplying, Close, check in SupplierStatus.msg
  int bonus_zone_status = 0;  // Buff Zone Status, namely Unoccupied, Being occupied and occupied.
  int reload_time;
  int bonus_time;

  BehaviorMode current_behavior = BehaviorMode::STOP;

  //subscribers
  ros::Subscriber buff_subscriber;
  ros::Subscriber damage_subscriber;
  ros::Subscriber robot_status_subscriber;
  ros::Subscriber reload_subscriber;
  ros::Subscriber game_status_subscriber;
  ros::Subscriber supplier_status_subscriber;
  ros::Subscriber robot_bonus_subscriber;
  ros::Subscriber bonus_status_subscriber;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
