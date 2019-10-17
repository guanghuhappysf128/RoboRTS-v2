project(roborts_camera)
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
set($ENV{CMAKE_INCLUDE_PATH} "/usr/include")
add_definitions(-w -Wno-dev)
# set this variable to TRUE to enable mercure
set(use_mercure TRUE)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  roborts_common
  cv_bridge
  image_transport
  )

find_package(ProtoBuf REQUIRED)
find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
    if (use_mercure)
      add_subdirectory(mercure)
      ADD_DEFINITIONS(-DAARCH64) # this is used to included mercure.h for using mercure camera on tx2
    else()
      message("mercure is not enabled yet")
    endif()
else()
    message("Warning: The mercure driver only support arm64")
endif()
add_subdirectory(uvc)

#camera parameter
file(GLOB CameraParamProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/camera_param.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  CameraParamProtoSrc
  CameraParamProtoHds
  ${CameraParamProtoFiles}
  )

#camera param
add_library(roborts_camera_param
  SHARED
  ${CameraParamProtoSrc}
  ${CameraParamProtoHds}
  camera_param.cpp
)

target_link_libraries(roborts_camera_param
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

target_include_directories(roborts_camera_param
  PUBLIC
  ${catkin_INCLUDE_DIRS}
)

#camera_node
add_executable(${PROJECT_NAME}_node
  camera_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  driver::uvc_driver
  roborts_camera_param
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
  if (use_mercure)
    target_link_libraries(${PROJECT_NAME}_node
      PRIVATE
      driver::mercure
    )
  else()
      message("mercure is not enabled yet")
  endif()
endif()

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

#cap_img

add_executable(image_capture_test
  test/image_capture.cpp
)

target_link_libraries(image_capture_test
  PRIVATE
  driver::uvc_driver
  roborts_camera_param
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(image_capture_test
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
)
