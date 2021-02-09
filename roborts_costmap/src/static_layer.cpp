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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "static_layer_setting.pb.h"
#include "static_layer.h"
#include <cstdio>

namespace roborts_costmap {

void StaticLayer::OnInitialize() {
  ros::NodeHandle nh;
  is_current_ = true;
  ParaStaticLayer para_static_layer;

  std::string static_map = ros::package::getPath("roborts_costmap") + \
      "/config/static_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(static_map.c_str(), &para_static_layer);
  global_frame_ = layered_costmap_-> GetGlobalFrameID();
  first_map_only_ = para_static_layer.first_map_only();
  subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
  track_unknown_space_ = para_static_layer.track_unknown_space();
  use_maximum_ = para_static_layer.use_maximum();
  int temp_threshold = para_static_layer.lethal_threshold();
  lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
  trinary_costmap_ = para_static_layer.trinary_map();
  unknown_cost_value_ = para_static_layer.unknown_cost_value();
  map_received_ = false;
  bool is_debug_ = para_static_layer.is_debug();
  map_topic_ = para_static_layer.topic_name();
  map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);
  ros::Rate temp_rate(10);
  while(!map_received_) {
    ros::spinOnce();
    temp_rate.sleep();
  }
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;
  global_tf_.waitForTransform("map", "odom", ros::Time(0), ros::Duration(10));
}

void StaticLayer::MatchSize() {
  if (!layered_costmap_->IsRolling()) {
    Costmap2D* master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
              master->GetOriginX(), master->GetOriginY());
  }
}

void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if(!layered_costmap_->IsRolling() && (master_map->GetSizeXCell() != size_x || master_map->GetSizeYCell() != size_y ||
      master_map->GetResolution() != resolution || master_map->GetOriginX() != origin_x || master_map->GetOriginY() != origin_y ||
      !layered_costmap_->IsSizeLocked())) {
        // if static map is passive, it should not force other maps to resize as it is likely to use a different global frame
      if (!layered_costmap_->IsStaticLayerPassive()) {
        layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
      }
  } else if(size_x_ != size_x || size_y_ != size_y || resolution_ != resolution || origin_x_ != origin_x || origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      value = new_map->data[temp_index];
      costmap_[temp_index] = InterpretValue(value);
      ++temp_index;
    }
  }
  map_received_ = true;
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

unsigned char StaticLayer::InterpretValue(unsigned char value) {
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::Activate() {
  OnInitialize();
}

void StaticLayer::Deactivate() {
//    delete cost_map_;
  //shut down the map topic message subscriber
  map_sub_.shutdown();
}

void StaticLayer::Reset() {
  if(first_map_only_) {
    has_updated_data_ = true;
  } else {
    OnInitialize();
  }
}

void StaticLayer::UpdateBounds(double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double *min_x,
                               double *min_y,
                               double *max_x,
                               double *max_y) {
  double wx, wy;
  if(!layered_costmap_->IsRollingWindow()) {
    if(!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
  //just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_+ width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  has_updated_data_ = false;
}

void StaticLayer::UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  // ROS_WARN("Before UseExtraBounds:  min_x: %f, min_y: %f, max_x: %f, max_y: %f",min_i,min_j,max_i,max_j);
  if(!map_received_) {
    return;
  }
  if(!layered_costmap_->IsRollingWindow()) {
    if(!use_maximum_) {
      UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
    } else {
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } 
  // is rolling window with the static costmap
  else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    tf::Transform reverse_transform;
    // should be linked from map to base_link?
    try {
      //ROS_WARN("try to link tf for rolling static map from map frame: [%s] to global frame [%s]",map_frame_.c_str(),global_frame_.c_str());
      // tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), temp_transform);
      // tf_->lookupTransform(map_frame_, "base_link", ros::Time(0), temp_transform);
      global_tf_.lookupTransform("map", "base_link", ros::Time(0), temp_transform);


      tf::Vector3 v = temp_transform.getOrigin();
      tf::Quaternion q = temp_transform.getRotation();
      // double yaw, pitch, roll;
      // temp_transform.getBasis().getRPY(roll, pitch, yaw);
      // std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
      // ROS_INFO(" transform is x: %f, y: %f, z: %f", v.getX(), v.getY(), v.getZ());
      // ROS_INFO(" rotation is [%f, %f, %f, %f]",q.getX(),q.getY(),q.getZ(),q.getW());
      // ROS_INFO(" In RPY (degree) is roll: %f, pitch: %f, yaw: %f",roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);



      // reverse_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1),tf::Vector3(v.getX()+5,v.getY()+5,v.getZ()));
      // reverse_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1),tf::Vector3(4,4,0));
      // reverse_transform = tf::Transform(tf::Quaternion(0, 0, -0.707, 0.707),tf::Vector3(4,4,0));
      // reverse_transform = tf::Transform(q,v);
      // tf::Vector3 rv = reverse_transform.getOrigin();
      // tf::Quaternion rq = reverse_transform.getRotation();
      // double ryaw, rpitch, rroll;
      // reverse_transform.getBasis().getRPY(rroll, rpitch, ryaw);
      // // std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
      // ROS_INFO("reversed transform is x: %f, y: %f, z: %f", rv.getX(), rv.getY(), rv.getZ());
      // ROS_INFO("reversed rotation is [%f, %f, %f, %f]",rq.getX(),rq.getY(),rq.getZ(),rq.getW());
      // ROS_INFO("reversed In RPY (degree) is roll: %f, pitch: %f, yaw: %f",rroll*180.0/M_PI, rpitch*180.0/M_PI, ryaw*180.0/M_PI);
      // ROS_WARN("Generating transform between "+temp_transform.frame_id_ + " and "+ temp_transform.child_frame_id_);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for(auto i = min_i; i < max_i; ++i) {
      for(auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        // double lx,ly;
        // ROS_INFO("Before p position: %f, %f",wx,wy);
        // lx = reverse_transform * wx;
        // ly = reverse_transform * wy;
        // p = reverse_transform(p);
        // ROS_INFO("after p position: %f, %f",p.x(),p.y());
        // ROS_INFO("after p position: %f, %f",lx,ly);
        p = temp_transform(p);
        if(World2Map(p.x(), p.y(), mx, my)){
        // if(World2Map(lx, ly, mx, my)){

          if(!use_maximum_) {
            master_grid.SetCost(i, j, GetCost(mx, my));
          }
          else {
            // the above line is wrong, not using any tf
            // master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
            master_grid.SetCost(i, j, std::max(master_grid.GetCost(mx, my), GetCost(mx, my)));
          }
        }
      }
    }
  }
}

} //namespace roborts_costmap

