cmake_minimum_required(VERSION 3.14)
project(object_recognition_utils)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  perception_msgs
)

# find_package(Boost REQUIRED)

# add_library(object_recognition_utils SHARED
#   src/predicted_path_utils.cpp
#   src/conversion.cpp
# )

# if(BUILD_TESTING)
#   find_package(ament_cmake_ros REQUIRED)

#   file(GLOB_RECURSE test_files test/**/*.cpp)

#   ament_add_ros_isolated_gtest(test_object_recognition_utils ${test_files})

#   target_link_libraries(test_object_recognition_utils
#     object_recognition_utils
#   )
# endif()
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
