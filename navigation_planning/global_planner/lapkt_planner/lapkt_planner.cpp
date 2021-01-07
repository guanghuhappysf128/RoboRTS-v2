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


#include <nav_msgs/GetPlan.h>
#include <pluginlib/class_list_macros.h>
#include <boost/format.hpp>
#include "lapkt_planner.hxx"
#include "state_model.hxx"
#include "gbfs.hxx"

namespace roborts_global_planner{

  using roborts_common::ErrorCode;
  using roborts_common::ErrorInfo;

  LapktPlanner::LapktPlanner(CostmapPtr costmap_ptr) :
      GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
      gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
      gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
      cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {

    LapktPlannerConfig lapkt_planner_config;
    std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/lapkt_planner/"\
        "config/lapkt_planner_config.prototxt";

    if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                            &lapkt_planner_config)) {
      ROS_ERROR("Cannot load a star planner protobuf configuration file.");
    }

    //  LapktPlanner param config
    heuristic_factor_ = lapkt_planner_config.heuristic_factor();
    inaccessible_cost_ = lapkt_planner_config.inaccessible_cost();
    goal_search_tolerance_ = lapkt_planner_config.goal_search_tolerance()/costmap_ptr->GetCostMap()->GetResolution();
    _old_navfn_behavior = lapkt_planner_config.old_navfn_behavior();


    if(!_old_navfn_behavior)
        _convert_offset = 0.5;
    else
        _convert_offset = 0.0;
    ROS_INFO("Lapkt planner initialized correctly.");
    _orientation_filter = std::make_shared<global_planner::OrientationFilter>();
    _initialized= true;
  }

  LapktPlanner::~LapktPlanner(){
    cost_ = nullptr;
  }

  ErrorInfo LapktPlanner::Plan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &path) {

    unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
    unsigned int valid_goal[2];
    unsigned  int shortest_dist = std::numeric_limits<unsigned int>::max();
    bool goal_valid = false;

    if (!_initialized) {
      ROS_ERROR("SimplePlanner::makePlan: planner has not been initialized");
      return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Lapkt planner has not been initialized");
    }

    if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                              start.pose.position.y,
                                              start_x,
                                              start_y)) {
      ROS_WARN("Failed to transform start pose from map frame to costmap frame");
      return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                      "Start pose can't be transformed to costmap frame.");
    }
    if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                              goal.pose.position.y,
                                              goal_x,
                                              goal_y)) {
      ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
      return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                      "Goal pose can't be transformed to costmap frame.");
    }
    if (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)<inaccessible_cost_){
      valid_goal[0] = goal_x;
      valid_goal[1] = goal_y;
      goal_valid = true;
    }else{
      tmp_goal_x = goal_x;
      tmp_goal_y = goal_y - goal_search_tolerance_;

      while(tmp_goal_y <= goal_y + goal_search_tolerance_){
        tmp_goal_x = goal_x - goal_search_tolerance_;
        while(tmp_goal_x <= goal_x + goal_search_tolerance_){
          unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
          #if (ROS_VERSION_MINOR == 14)
          unsigned int dist = abs(static_cast<int>(goal_x - tmp_goal_x)) + abs(static_cast<int>(goal_y - tmp_goal_y));
          #else
          unsigned int dist = abs(goal_x - tmp_goal_x) + abs(goal_y - tmp_goal_y);
          #endif
          if (cost < inaccessible_cost_ && dist < shortest_dist ) {
            shortest_dist = dist;
            valid_goal[0] = tmp_goal_x;
            valid_goal[1] = tmp_goal_y;
            goal_valid = true;
          }
          tmp_goal_x += 1;
        }
        tmp_goal_y += 1;
      }
    }
    ErrorInfo error_info;
    if (!goal_valid){
      error_info=ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
      path.clear();
    }
    else{

      _map_params.origin = lapkt::Vector(costmap_ptr_->GetCostMap()->GetOriginX(), costmap_ptr_->GetCostMap()->GetOriginY());
      _map_params.width = (int)costmap_ptr_->GetCostMap()->GetSizeXCell();
      _map_params.height = (int)costmap_ptr_->GetCostMap()->GetSizeYCell();
      _map_params.res = costmap_ptr_->GetCostMap()->GetResolution();
      _occ_map = lapkt::OccupancyMap(_map_params.height, _map_params.width);
      for (int iy = 0; iy < _map_params.height; iy++)
        for (int ix = 0; ix < _map_params.width; ix++) {
            auto cxy = costmap_ptr_->GetCostMap()->GetCost(ix, iy);
            _occ_map(iy,ix) = cxy < inaccessible_cost_;
        }
      path.clear();
      ROS_WARN("map width and height and res: %d, %d and %f",_map_params.width,_map_params.height,_map_params.res);

      // map initial and goal poses into cell indices
      // 1/ transform from world to map coordinates
      lapkt::Point pG(goal.pose.position.x, goal.pose.position.y);

      if (!world_to_map(pG,costmap_ptr_)) {
          ROS_WARN_THROTTLE(1.0,
                            "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
          return ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR,
                    "Goal is not reachable");
      }


      lapkt::Point p0(start.pose.position.x, start.pose.position.y);

      if (!world_to_map(p0,costmap_ptr_)) {
          ROS_WARN(
                  "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
          return ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR,
                    "Initial is not reachable");
      }


      ROS_WARN("initial state (in occ map coordinates): %f, %f and %d",p0.x(),p0.y(),_occ_map(p0.x(),p0.y()));
      ROS_WARN("Goal state (in occ map coordinates): %f, %f",pG.x(),pG.y());
      lapkt::SearchModel   search_problem(_map_params, _occ_map, p0, pG);
      if (!search_problem.valid(search_problem.I)) {
          //ROS_WARN("SimplePlanner::makePlan: Invalid initial state");
          return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                      "Invalid initial state");
      }

      if (!search_problem.valid(search_problem.G)) {
          //ROS_WARN("SimplePlanner::makePlan: Invalid goal state");
          return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                      "Invalid goal state");
      }
      std::vector<lapkt::State> best_path;

      // Call Path Planner
      lapkt::gbfs::SearchAlgorithm planner(search_problem);
      bool result = planner.find_path(best_path);

      if (!result) {
          ROS_WARN("SimplePlan::makePlan: Goal is not reachable!");
          return ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR,
                    "Goal is not reachable");
      }

      ROS_INFO("Plan length: %d",best_path.size());

      //extract the plan

        if (extract_plan(best_path, goal, path)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            path.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }

      _orientation_filter->processPath(start, path);
      //publish_plan(path);
      error_info=ErrorInfo(ErrorCode::OK);


      // unsigned int start_index, goal_index;
      // start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
      // goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

      // costmap_ptr_->GetCostMap()->SetCost(start_x, start_y,roborts_costmap::FREE_SPACE);

      // if(start_index == goal_index){
      //   error_info=ErrorInfo::OK();
      //   path.clear();
      //   path.push_back(start);
      //   path.push_back(goal);
      // }
      // else{
      //   error_info = SearchPath(start_index, goal_index, path);
      //   if ( error_info.IsOK() ){
      //     path.back().pose.orientation = goal.pose.orientation;
      //     path.back().pose.position.z = goal.pose.position.z;
      //   }
      // }

    }


    return error_info;
  }

  ErrorInfo LapktPlanner::SearchPath(const int &start_index,
                                    const int &goal_index,
                                    std::vector<geometry_msgs::PoseStamped> &path) {

    g_score_.clear();
    f_score_.clear();
    parent_.clear();
    state_.clear();
    gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
    gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
    ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
    cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
    g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

    std::priority_queue<int, std::vector<int>, Compare> openlist;
    g_score_.at(start_index) = 0;
    openlist.push(start_index);

    std::vector<int> neighbors_index;
    int current_index, move_cost, h_score, count = 0;

    while (!openlist.empty()) {
      current_index = openlist.top();
      openlist.pop();
      state_.at(current_index) = SearchState::CLOSED;

      if (current_index == goal_index) {
        ROS_INFO("Search takes %d cycle counts", count);
        break;
      }

      GetNineNeighbors(current_index, neighbors_index);

      for (auto neighbor_index : neighbors_index) {

        if (neighbor_index < 0 ||
            neighbor_index >= gridmap_height_ * gridmap_width_) {
          continue;
        }

        if (cost_[neighbor_index] >= inaccessible_cost_ ||
            state_.at(neighbor_index) == SearchState::CLOSED) {
          continue;
        }

        GetMoveCost(current_index, neighbor_index, move_cost);

        if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {

          g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
          parent_.at(neighbor_index) = current_index;

          if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
            GetManhattanDistance(neighbor_index, goal_index, h_score);
            f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
            openlist.push(neighbor_index);
            state_.at(neighbor_index) = SearchState::OPEN;
          }
        }
      }
      count++;
    }

    if (current_index != goal_index) {
      ROS_WARN("Global planner can't search the valid path!");
      return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
    }

    unsigned int iter_index = current_index, iter_x, iter_y;

    geometry_msgs::PoseStamped iter_pos;
    iter_pos.pose.orientation.w = 1;
    iter_pos.header.frame_id = "map";
    path.clear();
    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);

    while (iter_index != start_index) {
      iter_index = parent_.at(iter_index);
  //    if(cost_[iter_index]>= inaccessible_cost_){
  //      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned int>(cost_[iter_index]);
  //    }
      costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
      costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
      path.push_back(iter_pos);
    }

    std::reverse(path.begin(),path.end());
    return ErrorInfo(ErrorCode::OK);

  }

  ErrorInfo LapktPlanner::GetMoveCost(const int &current_index,
                                      const int &neighbor_index,
                                      int &move_cost) const {
    if (abs(neighbor_index - current_index) == 1 ||
        abs(neighbor_index - current_index) == gridmap_width_) {
      move_cost = 10;
    } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
        abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
      move_cost = 14;
    } else {
      return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                      "Move cost can't be calculated cause current neighbor index is not accessible");
    }
    return ErrorInfo(ErrorCode::OK);
  }
  #if (ROS_VERSION_MINOR == 14)
  void LapktPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
    manhattan_distance = heuristic_factor_* 10 * (abs(static_cast<int>(index1 / gridmap_width_ - index2 / gridmap_width_) )  +
        abs(static_cast<int>(index1 % gridmap_width_ - index2 % gridmap_width_)));
  }
  #else
  void LapktPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
    manhattan_distance = heuristic_factor_* 10 * (abs(index1 / gridmap_width_ - index2 / gridmap_width_) +
        abs(index1 % gridmap_width_ - index2 % gridmap_width_));
  }
  #endif


  void LapktPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
    neighbors_index.clear();
    if(current_index - gridmap_width_ >= 0){
      neighbors_index.push_back(current_index - gridmap_width_);       //up
    }
    if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
      neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
    }
    if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
      neighbors_index.push_back(current_index - 1);        //left
    }
    if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
        && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
      neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
    }
    if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
      neighbors_index.push_back(current_index + gridmap_width_);     //down
    }
    if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
        && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
      neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
    }
    if(current_index  + 1 < gridmap_width_* gridmap_height_
        && (current_index  + 1 ) % gridmap_width_!= 0) {
      neighbors_index.push_back(current_index + 1);                   //right
    }
    if(current_index - gridmap_width_ + 1 >= 0
        && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
      neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
    }
  }

  bool
  LapktPlanner::extract_plan(const std::vector<lapkt::State>& path,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) {
      if (!_initialized) {
          ROS_ERROR(
                  "This planner has not been initialized yet, but it is being used, please call initialize() before use");
          return false;
      }

      std::string global_frame = "map";

      //clear the plan, just in case
      plan.clear();

      ros::Time plan_time = ros::Time::now();
      for (const lapkt::State& s: path) {
          double world_x, world_y;

          map_to_world(s.pos[0], s.pos[1], world_x, world_y);

          geometry_msgs::PoseStamped pose;
          pose.header.stamp = plan_time;
          pose.header.frame_id = global_frame;
          pose.pose.position.x = world_x;
          pose.pose.position.y = world_y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          plan.push_back(pose);
      }
      if(_old_navfn_behavior){
          plan.push_back(goal);
      }
      return !plan.empty();
      return true;
  }

  bool
  LapktPlanner::world_to_map(double wx, double wy, double& mx, double& my) {
      const auto& origin = _map_params.origin;
      double resolution = _map_params.res;

      if (wx < origin.x() || wy < origin.y())
          return false;

      mx = (wx - origin.x()) / resolution - _convert_offset;
      my = (wy - origin.y()) / resolution - _convert_offset;

      if (mx < _map_params.width && my < _map_params.height)
          return true;

      return false;
  }

  bool
  LapktPlanner::world_to_map(lapkt::Point& pt, CostmapPtr costmap_ptr) {
      double wx = pt.x();
      double wy = pt.y();

      unsigned int p_x_i, p_y_i;
      double p_x, p_y;

      // if (!costmap_ptr->worldToMap(wx, wy, p_x_i, p_y_i)) {
      //     return false;
      // }

      if (!costmap_ptr_->GetCostMap()->World2Map(wx, wy, p_x_i, p_y_i)) {
        ROS_WARN("Failed to transform start pose from map frame to costmap frame");
        return false;
      }
      if(_old_navfn_behavior){
          p_x = p_x_i;
          p_y = p_y_i;
      } else{
          world_to_map(wx, wy, p_x, p_y);
      }
      pt = lapkt::Point(p_x, p_y);

      return true;
  }

  void
  LapktPlanner::map_to_world(double mx, double my, double& wx, double& wy) {
      wx = _map_params.origin.x() + (mx+_convert_offset) * _map_params.res;
      wy = _map_params.origin.y() + (my+_convert_offset) * _map_params.res;
  }


} //namespace roborts_global_planner
