project(navigation_planning)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_conversions
  pcl_ros
  nav_msgs
  geometry_msgs
  visualization_msgs
  actionlib
  roborts_common
  roborts_msgs
  interactive_markers
  roborts_costmap
  )

#message("catkin libraries ${catkin_LIBRARIES}")

find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Eigen3 REQUIRED)
if(${roscpp_VERSION_MINOR} EQUAL 14)
  message("request packages for ros melodic")
  find_package(PCL 1.8 REQUIRED)
else()
  message("request packages for ros kinetic")
  find_package(PCL 1.7 REQUIRED)
endif()
find_package(ProtoBuf REQUIRED)
find_package(Boost REQUIRED)

catkin_package()

add_subdirectory(global_planner)
add_subdirectory(local_planner)


