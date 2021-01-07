project(lapkt_planner)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto LapktPlannerConfigProtoSrc LapktPlannerConfigProtoHds ${ProtoFiles})



include_directories(${catkin_INCLUDE_DIRS})

add_definitions(
        -DBOOST_SYSTEM_NO_DEPRECATED
        -DCGAL_HEADER_ONLY
)

# MRJ: disabled warnings -Wformat-security -Wno-parentheses -Wpedantic because they
# were a pain in the back (they're being propagated by ROS headers)
add_compile_options(
        -Wall -pedantic -Wno-format-security -Wno-parentheses -Wno-pedantic -fPIC "${EIGEN3_CFLAGS}"
)

add_library(${PROJECT_NAME}
  SHARED
  ${LapktPlannerConfigProtoSrc}
  ${LapktPlannerConfigProtoHds}
  lapkt_planner.cpp
  state_model.cxx
  gbfs.cxx
  orientation_filter.cpp
  )
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${catkin_LIBRARIES}
  #roborts_costmap
)
set(SOURCES
        lapkt_planner.cpp
        state_model.cxx
        gbfs.cxx
        orientation_filter.cpp)

set(HEADERS
        lapkt_planner.hxx
        geometry.hxx
        state_model.hxx
        grid_map.hxx
        octile_heuristic.hxx
        open_lists.hxx
        closed_lists.hxx
        gbfs.hxx
        orientation_filter.h)



add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
