cmake_minimum_required(VERSION 3.14)
project(lidar_centerpoint)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tensorrt_common
  perception_msgs
  object_recognition_utils
  tier4_autoware_utils
)

add_compile_options(-Wno-deprecated-declarations)
find_package(Eigen3 REQUIRED)
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
set(TRT_PATH /usr/local/TensorRT-8.6.1.6)
message("TENSORRT_LIB_PATH: ${TRT_PATH}")

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA REQUIRED)
if(CUDA_FOUND)
  set(CUDA_VERBOSE ON)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
    message("CUBLAS: ${CUBLAS_LIBRARIES}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER nvinfer HINTS ${TRT_PATH}/lib)
find_library(NVONNXPARSER nvonnxparser HINTS ${TRT_PATH}/lib)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)

  include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TRT_PATH}/include
    ${EIGEN_INCLUDE_DIRS}

  )

  ### centerpoint ###
  add_library(centerpoint_lib SHARED
    lib/centerpoint_trt.cpp
    lib/detection_class_remapper.cpp
    lib/utils.cpp
    lib/ros_utils.cpp
    lib/network/network_trt.cpp
    lib/network/tensorrt_wrapper.cpp
    lib/postprocess/non_maximum_suppression.cpp
    lib/preprocess/pointcloud_densification.cpp
    lib/preprocess/voxel_generator.cpp
    lib/ground_segmentation/FastMorFilter.cpp
  )

  cuda_add_library(centerpoint_cuda_lib SHARED
    lib/postprocess/circle_nms_kernel.cu
    lib/postprocess/postprocess_kernel.cu
    lib/network/scatter_kernel.cu
    lib/preprocess/preprocess_kernel.cu
  )

  target_link_libraries(centerpoint_lib
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    ${catkin_LIBRARIES}
    centerpoint_cuda_lib
  )

  target_include_directories(centerpoint_lib
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  target_include_directories(centerpoint_lib
    SYSTEM PUBLIC
      ${CUDA_INCLUDE_DIRS}
  )

  ## node ##
  add_executable(lidar_centerpoint_component
  src/main.cpp
  src/node.cpp
)

  target_link_libraries(lidar_centerpoint_component
    centerpoint_lib
    ${catkin_LIBRARIES}
  )

  install(PROGRAMS
    scripts/lidar_centerpoint_visualizer.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # install(
  #   TARGETS centerpoint_cuda_lib
  #   DESTINATION lib
  # )
else()
  message("Dependencies not found. Unable to build.")
endif()
