#ifndef ROBORTS_DETECTION_AIM_AID_H
#define ROBORTS_DETECTION_AIM_AID_H

#include <ros/ros.h>
#include "state/node_state.h"
#include "state/error_code.h"

#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "roborts_msgs/SimHasLoS.h"
#include "proto/armor_detection.pb.h"
#include "cv_toolbox.h"

#include <mutex>
#include <condition_variable>

namespace roborts_detection {

  using roborts_common::NodeState;
  using roborts_common::ErrorInfo;

  class AimAid {
  public:
    void Init(ArmorDetectionAlgorithms& armor_detection_param);
    explicit AimAid();
    ErrorInfo DetectArmor(bool& enemy_detected, cv::Point3f& target);
  private:
    ros::NodeHandle nh_;
    // subscribe to the real poses
    std::vector<ros::Subscriber> gazebo_real_pose_sub_;

    int robot_ind_ = -1;
    int friend_ind_ = -1;
    std::vector<geometry_msgs::PoseWithCovariance> poses_;

    double PI = 3.14159265;
    double fb_offset_ = 0.25;
    double lr_offset_ = 0.15;
    double z_offset_ = 0.10; // tbc
    double visual_angle_ = 80; // unit: degree
    double max_range_ = 0;
    bool has_friend_ = false;
    std::vector<std::string> robot_names_;
    ros::ServiceClient los_client_;
    std::mutex mutex_;
    //std::condition_variable condition_var_;

    void PoseCallback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int robot_index);
    double GetDistance(cv::Point3f p1, cv::Point3f p2);
    double GetDistance(cv::Point2f p1, cv::Point2f p2);
    // for the exact derivation, please ask the author or see the documentation
    void GetAlphaBeta(double angle, double distance, double& alpha, double& beta);
    double GetYaw(geometry_msgs::Quaternion orientation);
    void AttachPlates(geometry_msgs::PoseWithCovariance base_pose, std::vector<cv::Point3f> & plates);
    bool HasLoS(cv::Point2f base, cv::Point2f target);
    void GetPointsInCone(geometry_msgs::PoseWithCovariance base_pose, std::vector<int>& indexes);
};
} // namespace roborts_detection
#endif 