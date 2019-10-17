project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  #roborts_sim
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)
if(${roscpp_VERSION_MINOR} EQUAL 14)
  message("request packages for ros melodic")
  find_package(PCL 1.8 REQUIRED)
else()
  message("request packages for ros kinetic")
  find_package(PCL 1.7 REQUIRED)
endif()

set(EXTERNAL_LIBS
  #${SUITESPARSE_LIBRARIES}
  #${G2O_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  #roborts_costmap
  )
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#message("external libraries are ${EXTERNAL_LIBS}")
#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_executable(behavior_test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
 
  example_behavior/shoot_behavior.h
  example_behavior/reload_behavior.h
  behavior_tree/behavior_node.h
  goal_factory/test_goal_factory.h
  goal_factory/test_goal_factory.cpp
  goal_factory/hierarchical_goal_factory.h
  goal_factory/hierarchical_goal_factory.cpp
  behavior_test.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(behavior_test_node
  PRIVATE
  #roborts_costmap
  #${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  ${EXTERNAL_LIBS}
  )

add_dependencies(behavior_test_node
  roborts_msgs_generate_messages_cpp)


add_executable(open_day_decision_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/open_day_chase_behavior.h
  example_behavior/open_day_patrol_behavior.h

 
  example_behavior/shoot_behavior.h
  example_behavior/reload_behavior.h
  behavior_tree/behavior_node.h
  goal_factory/test_goal_factory.h
  goal_factory/test_goal_factory.cpp
  goal_factory/hierarchical_goal_factory.h
  goal_factory/hierarchical_goal_factory.cpp

  goal_factory/open_day_goal_factory.h
  goal_factory/open_day_goal_factory.cpp
  open_day_decision_node.cpp
  behavior_modes.h
  executor/chassis_executor.cpp
  )

target_link_libraries(open_day_decision_node
  PRIVATE
#  roborts_costmap
  #${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  ${EXTERNAL_LIBS}
  )
add_dependencies(open_day_decision_node
  roborts_msgs_generate_messages_cpp)

add_executable(decision_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/reload_behavior.h
  behavior_tree/behavior_node.h
  goal_factory/test_goal_factory.h
  goal_factory/test_goal_factory.cpp
  goal_factory/hierarchical_goal_factory.h
  goal_factory/hierarchical_goal_factory.cpp
  decision_node.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(decision_node
  PRIVATE
#  roborts_costmap
 # ${catkin_LIBRARIES}
 # ${PROTOBUF_LIBRARIES}
  ${EXTERNAL_LIBS}
  )
add_dependencies(decision_node
  roborts_msgs_generate_messages_cpp)


add_executable(navigation_test_decision
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  behavior_tree/behavior_node.h
  navigation_test_decision.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(navigation_test_decision
  PRIVATE
 # roborts_costmap
  #${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  ${EXTERNAL_LIBS}
  )
add_dependencies(navigation_test_decision
  roborts_msgs_generate_messages_cpp)