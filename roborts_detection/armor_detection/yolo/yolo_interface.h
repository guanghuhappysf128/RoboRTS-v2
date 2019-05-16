#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_YOLO_INTERFACE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_YOLO_INTERFACE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "cv_toolbox.h"
#include "c2cpp.h"
#include "proto/yolo.pb.h"
#include <ros/ros.h>
#include "io/io.h"

namespace roborts_detection {
class YOLOInterface {
  public:
    YOLOInterface(std::shared_ptr<CVToolbox> cv_toolbox) : cv_toolbox_(cv_toolbox), object_num_(0), x_offset_(NULL) {}
    void Init() {
      LoadParam();
      // invoke c function
      // names
      std::string prefix = ros::package::getPath("roborts_detection") + "/armor_detection/yolo/config/";
      std::string names = prefix + "mobilenet.names";
      InitYOLO(prefix.c_str(), datacfg_.c_str(), cfg_.c_str(), weights_.c_str(), names.c_str() , .5, enable_debug_);
    }
    void FilterImg(cv::Mat& img) {
      // convert mat to IplImage format
      IplImage *nextImg;
      if(!img.empty()) {
        IplImage src = IplImage(img);
        nextImg = cvCloneImage(&src);
      } else{
        ROS_ERROR("image is empty");
        return;
      }
      if (&nextImg == NULL) {
        ROS_ERROR("Ipl image is empty!");
      }
      // invoke c function
      RunYOLO(enable_debug_, nextImg, &x_offset_, &object_num_);
      // then return an IplImage that will be converted to mat
    }
  private:
    std::shared_ptr<CVToolbox> cv_toolbox_;
    bool enable_debug_;
    std::string datacfg_;
    std::string cfg_;
    std::string weights_;
    cv::Mat src_img_;
    int object_num_;
    int *x_offset_;

    void LoadParam() {
      YOLOConfig yolo_config_;
      std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/yolo/config/yolo.prototxt";
      bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &yolo_config_);
      if (!read_state) {
        ROS_ERROR("Cannot open %s", file_name.c_str());
        return;
      }
      enable_debug_ = yolo_config_.enable_debug();
      // following three are in fact path to the configuration files
      datacfg_      = ros::package::getPath("roborts_detection") + "/armor_detection/yolo/config/" + yolo_config_.datacfg();
      cfg_          = ros::package::getPath("roborts_detection") + "/armor_detection/yolo/config/" + yolo_config_.cfg();
      weights_      = ros::package::getPath("roborts_detection") + "/armor_detection/yolo/config/weights/" + yolo_config_.weights();
      //ROS_INFO("weight path is %s", weights_.c_str());

    }
};

} //namespace roborts_detection
#endif // ROBORTS_DETECTION_ARMOR_DETECTION_YOLO_INTERFACE_H
