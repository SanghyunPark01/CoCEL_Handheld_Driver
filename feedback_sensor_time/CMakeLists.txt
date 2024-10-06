cmake_minimum_required(VERSION 3.0.2)
project(feedback_sensor_time)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_estimator
  src/time_delay_estimator.cpp
)
target_link_libraries(${PROJECT_NAME}_estimator
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_library(${PROJECT_NAME}_kalman_filter
  src/kalman_filter.cpp
  )
target_link_libraries(${PROJECT_NAME}_kalman_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(estimator_node
  src/main.cpp
)
target_link_libraries(estimator_node
  ${PROJECT_NAME}_estimator
  ${PROJECT_NAME}_kalman_filter
)

add_executable(time_share_test
  src/test_time_share.cpp
)
target_link_libraries(time_share_test
  ${catkin_LIBRARIES}
)