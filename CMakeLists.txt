cmake_minimum_required(VERSION 3.0.2)
project(simple_lio)

set(CMAKE_BUILD_TYPE "Release")

set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -Wno-unused-local-typedefs")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  livox_ros_driver
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(3rd_party/spdlog)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # spdlog
  ${PROJECT_SOURCE_DIR}/third_party/spdlog/include
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES simple_lio
  # CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
  # DEPENDS system_lib
)

add_executable(custom_msg_to_sensor_msg src/custom_msg_to_sensor_msg.cpp)
target_link_libraries(custom_msg_to_sensor_msg
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(imu_process src/imu_process.cpp include/ikd-Tree/ikd_Tree.cpp)
target_link_libraries(imu_process
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  spdlog::spdlog
  ${YAML_CPP_LIBRARIES}
)
