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
#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H

#include <iterator>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <ros/publisher.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include "timed_elastic_band/teb_vertex_console.h"
#include "local_planner/robot_footprint_model.h"



namespace roborts_local_planner {

/**
 * @brief Class use to visualize local planner algorithm's trajectory
 */
class LocalVisualization {
 public:
  /**
   * @brief Constructor
   */
  LocalVisualization();
  /**
   * @brief Constructor initialize this class
   * @param nh Ros node handle
   * @param visualize_frame Visualize frame
   */
  LocalVisualization(ros::NodeHandle& nh, const std::string& visualize_frame);
  /**
   * @brief Initialize visualize frame and ros param
   * @param nh ros node handle
   * @param visualize_frame Visualize frame
   */
  void Initialization(ros::NodeHandle& nh, const std::string& visualize_frame);
  /**
   * @brief publish trajectory
   * @param vertex_console Robot trajectory from local planner algorithm
   */
  void PublishLocalPlan(const TebVertexConsole &vertex_console) const;

  /**
   * @brief Publish the visualization of the robot model
   * 
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseRobotFootprintModel
   * @param ns Namespace for the marker objects
   * @param color Color of the footprint
   */
  void publishRobotFootprintModel(const DataBase& current_pose, const BaseRobotFootprintModel& robot_model, const std::string& ns = "RobotFootprintModel",
                                  const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0)) const;
  
  /**
   * @brief Helper function to generate a color message from single values
   * @param a Alpha value
   * @param r Red value
   * @param g Green value
   * @param b Blue value
   * @return Color message
   */
  static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

 protected:

  bool printErrorWhenNotInitialized() const;

  //! trajectory publisher
  ros::Publisher local_planner_;
  //! trajectory pose publisher
  ros::Publisher pose_pub_;
  ros::Publisher teb_marker_pub_; //!< Publisher for visualization markers

  //! visualize frame
  std::string visual_frame_ = "map";

  //! initialize state
  bool initialized_;

};

typedef boost::shared_ptr<const LocalVisualization> LocalVisualizationPtr;

} // namespace roborts_local_planner

#endif // ROBORTS_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H

