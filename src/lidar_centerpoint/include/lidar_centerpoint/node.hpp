#ifndef _NODE_HPP_
#define _NODE_HPP_

#include "lidar_centerpoint/postprocess/non_maximum_suppression.hpp"

#include <lidar_centerpoint/centerpoint_trt.hpp>
#include <lidar_centerpoint/detection_class_remapper.hpp>
#include "lidar_centerpoint/ground_segmentation/FastMorFilter.h"
#include <ros/ros.h>
// #include <tier4_autoware_utils/ros/debug_publisher.hpp>
// #include <tier4_autoware_utils/system/stop_watch.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <perception_msgs/DetectedObjectKinematics.h>
#include <perception_msgs/DetectedObjects.h>
#include <perception_msgs/ObjectClassification.h>
#include <perception_msgs/Shape.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

namespace centerpoint
{

class LidarCenterPointNode
{
public:
  explicit LidarCenterPointNode();

private:
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;
  ros::NodeHandle *nh;
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input_pointcloud_msg);
  void cloudsProcess(PointCloudT::Ptr &source_pointcloud);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  ros::Subscriber pointcloud_sub_;
  ros::Publisher objects_pub_;
  float score_threshold;
  float circle_nms_dist_threshold;
  std::vector<double> yaw_norm_thresholds;
  std::string densification_world_frame_id;
  int densification_num_past_frames;
  std::string trt_precision;
  std::string encoder_onnx_path;
  std::string encoder_engine_path;
  std::string head_onnx_path;
  std::string head_engine_path;
  std::vector<std::string> class_names_;
  bool has_twist_;
  int point_feature_size;
  int max_voxel_size;
  std::vector<double> point_cloud_range;
  std::vector<double> voxel_size;
  int downsample_factor;
  int encoder_in_feature_size;
  std::vector<int> allow_remapping_by_area_matrix;
  std::vector<double> min_area_matrix;
  std::vector<double> max_area_matrix;

  NonMaximumSuppression iou_bev_nms_;
  DetectionClassRemapper detection_class_remapper_;

  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};

  // debugger
//   std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
//     nullptr};
//   std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
};

}  // namespace centerpoint

#endif  // NODE_HPP_
