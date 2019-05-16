#include "aim_aid.h"
namespace roborts_detection {
  void AimAid::Init(ArmorDetectionAlgorithms& armor_detection_param){
      fb_offset_ = armor_detection_param.aim_aid().fb_offset();
      lr_offset_ = armor_detection_param.aim_aid().lr_offset();
      z_offset_  = armor_detection_param.aim_aid().z_offset(); // tbc
      visual_angle_ = armor_detection_param.aim_aid().visual_angle(); // unit: degree
      max_range_ = armor_detection_param.aim_aid().max_range();
      int robots_num = armor_detection_param.aim_aid().robot_name().size();
      std::vector<std::string> robot_names_(robots_num);
      for (int i = 0; i < robots_num; i++) {
        std::lock_guard<std::mutex> guard(mutex_);
        robot_names_[i] = armor_detection_param.aim_aid().robot_name(i);
      }
      // decide robot index
      std::string base_name = armor_detection_param.aim_aid().base_name();
      for (int i =0; i < robots_num; i++) {
        if (robot_names_[i] == base_name) {
          robot_ind_ = i;
        }
      }
      if (armor_detection_param.aim_aid().has_friend_name()) {
        has_friend_ = true;
        std::string friend_name = armor_detection_param.aim_aid().friend_name();
        for (int i = 0; i < robots_num; i++) {
          if (robot_names_[i] == friend_name) {
            friend_ind_ = i;
          }
        }
        ROS_INFO("robot id is %d; friend id is %d", robot_ind_, friend_ind_);
      } else {
        has_friend_ = false;
        ROS_INFO("robot id is %d; it has no friend", robot_ind_);
      }
      
      poses_ = std::vector<geometry_msgs::PoseWithCovariance>(robot_names_.size());
      for (int i = 0; i < robot_names_.size(); i++) {
        std::string name = robot_names_[i];
        std::string pose_topic = "/" + name + "/gazebo_robot_pose";
        ROS_INFO("pose_topic is %s", pose_topic.c_str());
        // initialize the pose with empty pose
        geometry_msgs::PoseWithCovariance empty_pose;
        poses_[i] = empty_pose;
        gazebo_real_pose_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>(pose_topic, 100, boost::bind(&AimAid::PoseCallback,this,_1,i)));
      }
      los_client_ = nh_.serviceClient<roborts_sim::HasLoS>("/has_los");
  }
  AimAid::AimAid() {}
  ErrorInfo AimAid::DetectArmor(bool& enemy_detected, cv::Point3f& target) {
      ErrorInfo error_info = ErrorInfo(roborts_common::OK);
      std::vector<int> indexes_in_view;
      auto base_pose = poses_[robot_ind_];
      //ROS_INFO("pose x %f", base_pose.pose.orientation.x);
      GetPointsInCone(base_pose, indexes_in_view);
      if (indexes_in_view.size() == 0) {
        //ROS_INFO("no robot in view");
        enemy_detected = false;
        return error_info;
      }
      // assemble the plates for each pose in cone
      std::vector<cv::Point3f> plates;
      for (auto ind : indexes_in_view) {
        ROS_INFO("index in view is %d", ind);
        //ROS_INFO("robot %s is in view", robot_names_[ind].c_str());
        AttachPlates(poses_[ind], plates);
      }
      
      // todo apply offset to camera point; currently no offset is accounted
      cv::Point3f camera_point;
      camera_point.x = base_pose.pose.position.x;
      camera_point.y = base_pose.pose.position.y;
      camera_point.z = z_offset_; 
      // retrieve the plate that closest to the camera
      int plate_ind = 0;
      double min_dist = GetDistance(camera_point, plates[0]);
      for (int i = 0; i < plates.size(); i++) {
        double dist = GetDistance(camera_point, plates[i]);
        if (dist < min_dist) {
          min_dist = dist;
          plate_ind = i;
        }
      }
      enemy_detected = true;
      target = plates[plate_ind];
      ROS_INFO("target is (x, y, z) = %f, %f, %f", target.x, target.y, target.z);
      return error_info;
  }
  void AimAid::PoseCallback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int robot_index) {
      std::lock_guard<std::mutex> guard(mutex_);
      poses_[robot_index] = pose_msg->pose;
      //ROS_INFO("pose received x is %f", pose_msg->pose.pose.position.x);
    }
  double AimAid::GetDistance(cv::Point3f p1, cv::Point3f p2) {
      return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
  }
  double AimAid::GetDistance(cv::Point2f p1, cv::Point2f p2) {
      return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
  }
    // for the exact derivation, please ask the author or see the documentation
  void AimAid::GetAlphaBeta(double angle, double distance, double& alpha, double& beta) {
      double tan_angle = tan(angle * PI / 180.0);
      alpha = distance / sqrt(1 + tan_angle * tan_angle);
      beta  = distance * tan_angle / sqrt(1 + tan_angle * tan_angle);
  }
  double AimAid::GetYaw(geometry_msgs::Quaternion orientation) {
      tf::Quaternion q(orientation.x,orientation.y,orientation.z,orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
  }
  void AimAid::AttachPlates(geometry_msgs::PoseWithCovariance base_pose, std::vector<cv::Point3f> & plates) {
      double alpha, beta;

      double x = base_pose.pose.position.x;
      double y = base_pose.pose.position.y;
      double yaw = GetYaw(base_pose.pose.orientation);
      cv::Point3f point;
      point.z = z_offset_;

      GetAlphaBeta(yaw, fb_offset_, alpha, beta);
      // front
      point.x = x + alpha;
      point.y = y + beta;
      plates.push_back(point);
      // back
      point.x = x - alpha;
      point.y = x - beta;
      plates.push_back(point);

      GetAlphaBeta(yaw - PI, lr_offset_, alpha, beta);
      // right
      point.x = x + alpha;
      point.y = y + beta;
      plates.push_back(point);
      // left
      point.x = x - alpha;
      point.y = x - beta;
      plates.push_back(point);
  }
  bool AimAid::HasLoS(cv::Point2f base, cv::Point2f target) {

      roborts_sim::HasLoS srv;
      srv.request.x1 = base.x;
      srv.request.y1 = base.y;
      srv.request.x2 = target.x;
      srv.request.y2 = target.y;
      ROS_INFO("test line of sight between (%f, %f) and (%f, %f)", base.x, base.y, target.x, target.y);
      if (los_client_.call(srv)) {
        return srv.response.has_los;
      } else {
        ROS_ERROR("Failed to call service has_los");
        return false;
      }
  }
  void AimAid::GetPointsInCone(geometry_msgs::PoseWithCovariance base_pose, std::vector<int>& indexes) {
      double x = base_pose.pose.position.x;
      double y = base_pose.pose.position.y;
      double yaw = GetYaw(base_pose.pose.orientation)*180/PI;
      // upper bound l1: y = k1 x + b1
      // lower bound l2: y = k2 x + b2
      double k1, k2, b1, b2;
      k1 = tan((yaw + visual_angle_ / 2) * PI/180);
      b1 = y - k1 * x;
      k2 = tan((yaw - visual_angle_ / 2) * PI/180);
      b2 = y - k2 * x;

      cv::Point2f base, target;
      base.x = x;
      base.y = y;
      for (int i = 0; i < poses_.size(); i++) {
        if (has_friend_ && i == friend_ind_ || i == robot_ind_) {
          continue;
        }
        double x_p = poses_[i].pose.position.x;
        double y_p = poses_[i].pose.position.y;
        target.x = x_p;
        target.y = y_p;
        // point pose_[i] is in range 
        bool pose_in_cone = y_p < k1 * x_p + b1 && y_p > k2 * x_p + b2;
        bool is_within_max_range = GetDistance(base, target) < max_range_;
        bool has_los = HasLoS(base, target);
        ROS_INFO("%s; %s; %s", pose_in_cone ? "Pose is in fov":"Pose is outside of fov",
         is_within_max_range ? "pose is in range" : "pose is too distant",
         has_los ? "base has los": "base doesn't have los");
        ROS_INFO("yaw is %f fov is consisted of l1: %f x + %f; l2: %f x + %f",yaw, k1, b1, k2, b2);
        if (pose_in_cone &&  is_within_max_range && has_los) {
          indexes.push_back(i);
        }
      }
    }
}