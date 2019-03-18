project(mercure)

add_library(${PROJECT_NAME}
  SHARED
  fps.cpp
  mercure_driver.cpp
)

add_library(driver::mercure ALIAS mercure)

target_link_libraries(${PROJECT_NAME}
  libgxiapi.so
  pthread
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)