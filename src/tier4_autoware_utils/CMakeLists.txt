cmake_minimum_required(VERSION 3.14)
project(tier4_autoware_utils)
add_compile_options(-std=c++17 -Wall)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  perception_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tier4_autoware_utils
  # CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_library(tier4_autoware_utils SHARED
  src/geometry/geometry.cpp
  # src/geometry/pose_deviation.cpp
  # src/geometry/boost_polygon_utils.cpp
  # src/math/sin_table.cpp
  # src/math/trigonometry.cpp
  # src/ros/msg_operation.cpp
  # src/ros/marker_helper.cpp
)

target_link_libraries(tier4_autoware_utils
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
# if(BUILD_TESTING)
#   find_package(ament_cmake_ros REQUIRED)

#   file(GLOB_RECURSE test_files test/**/*.cpp)

#   ament_add_ros_isolated_gtest(test_tier4_autoware_utils ${test_files})

#   target_link_libraries(test_tier4_autoware_utils
#     tier4_autoware_utils
#   )
# endif()

