cmake_minimum_required(VERSION 3.17)
project(tensorrt_common)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(CUDA REQUIRED)
set(TRT_PATH /usr/local/TensorRT-8.6.1.6)
# if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
#   message(WARNING "cuda, cudnn, tensorrt libraries are not found")
#   return()
# endif()
find_library(NVINFER nvinfer HINTS ${TRT_PATH}/lib)
find_library(NVONNXPARSER nvonnxparser HINTS ${TRT_PATH}/lib)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  # ${TRT_PATH}/include
  # /usr/include
  # /usr/include/x86_64-linux-gnu
)

add_library(${PROJECT_NAME} SHARED
  src/tensorrt_common.cpp
  src/simple_profiler.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${NVINFER}
  ${NVONNXPARSER}
  ${CUDA_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${TRT_PATH}/include
)

# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
