#include "lidar_centerpoint/node.hpp"

#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/preprocess/pointcloud_densification.hpp>
#include <lidar_centerpoint/ros_utils.hpp>
#include <lidar_centerpoint/utils.hpp>
#include <pcl_ros/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
LidarCenterPointNode::LidarCenterPointNode()
{
  *nh = ros::NodeHandle("~");
  nh->param<float>("score_threshold", score_threshold, 0.35);
  nh->param<float>("circle_nms_dist_threshold", circle_nms_dist_threshold, 0.0);
  nh->getParam("yaw_norm_thresholds", yaw_norm_thresholds);
  nh->param("densification_world_frame_id", densification_world_frame_id, std::string("map"));
  nh->param<int>("densification_num_past_frames", densification_num_past_frames, 1);
  nh->param("trt_precision", trt_precision, std::string("fp16"));
  nh->param("encoder_onnx_path", encoder_onnx_path, std::string(""));
  nh->param("encoder_engine_path", encoder_engine_path, std::string(""));
  nh->param("head_onnx_path", head_onnx_path, std::string(""));
  nh->param("head_engine_path", head_engine_path, std::string(""));
  nh->getParam("class_names", class_names_);
  nh->param<bool>("has_twist", has_twist_, false);
  nh->param<int>("point_feature_size", point_feature_size, 0);
  nh->param<int>("max_voxel_size", max_voxel_size, 0);
  nh->getParam("point_cloud_range", point_cloud_range);
  nh->getParam("voxel_size", voxel_size);
  nh->param<int>("downsample_factor", downsample_factor, 0);
  nh->param<int>("encoder_in_feature_size", encoder_in_feature_size, 0);
  nh->getParam("allow_remapping_by_area_matrix", allow_remapping_by_area_matrix);
  nh->getParam("min_area_matrix", min_area_matrix);
  nh->getParam("max_area_matrix", max_area_matrix);

  // detection_class_remapper_.setParameters(
  //   allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  // { 
  //   NMSParams p;
  //   p.nms_type_ = NMS_TYPE::IoU_BEV;
  //   private_nh.getParam("iou_nms_target_class_names", p.target_class_names_);
  //   private_nh.param("iou_nms_search_distance_2d", p.search_distance_2d_, 0.0);
  //   private_nh.param("iou_nms_threshold", p.iou_threshold_, 0.0);
  //   iou_bev_nms_.setParameters(p);
  // }

  // NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);
  // NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);
  // DensificationParam densification_param(
  //   densification_world_frame_id, densification_num_past_frames);

  if (point_cloud_range.size() != 6) {
    ROS_WARN_STREAM("The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    ROS_WARN_STREAM("The size of voxel_size != 3: use the default parameters.");
  }
  // CenterPointConfig config(
  //   class_names_.size(), point_feature_size, max_voxel_size, point_cloud_range, voxel_size,
  //   downsample_factor, encoder_in_feature_size, score_threshold, circle_nms_dist_threshold,
  //   yaw_norm_thresholds);
  // detector_ptr_ =
  //   std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);

  pointcloud_sub_ = nh->subscribe<sensor_msgs::PointCloud2>(
    "~/input/pointcloud", 10, &LidarCenterPointNode::pointCloudCallback, this);
  // objects_pub_ = nh.advertise<perception_msgs::DetectedObjects>(
  //   "~/output/objects", 10);

  // initialize debug tool
  // {
  //   using tier4_autoware_utils::DebugPublisher;
  //   using tier4_autoware_utils::StopWatch;
  //   stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
  //   debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "lidar_centerpoint");
  //   stop_watch_ptr_->tic("cyclic_time");
  //   stop_watch_ptr_->tic("processing_time");
  // }

  // if (this->declare_parameter("build_only", false)) {
  //   ROS_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
  //   ros::shutdown();
  // }
}

void LidarCenterPointNode::pointCloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr &input_pointcloud_msg)
{
  // const auto objects_sub_count = objects_pub_.getNumSubscribers();
  // if (objects_sub_count < 1) {
  //   return;
  // }


  // std::vector<Box3D> det_boxes3d;
  // bool is_success = detector_ptr_->detect(*input_pointcloud_msg, tf_buffer_, det_boxes3d);
  // if (!is_success) {
  //   return;
  // }

  // std::vector<perception_msgs::DetectedObject> raw_objects;
  // raw_objects.reserve(det_boxes3d.size());
  // for (const auto & box3d : det_boxes3d) {
  //   perception_msgs::DetectedObject obj;
  //   box3DToDetectedObject(box3d, class_names_, has_twist_, obj);
  //   raw_objects.emplace_back(obj);
  // }

  // perception_msgs::DetectedObjects output_msg;
  // output_msg.header = input_pointcloud_msg->header;
  // output_msg.objects = iou_bev_nms_.apply(raw_objects);

  // detection_class_remapper_.mapClasses(output_msg);

  // if (objects_sub_count > 0) {
  //   objects_pub_->publish(output_msg);
  // }

  // add processing time for debug
  // if (debug_publisher_ptr_ && stop_watch_ptr_) {
  //   const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
  //   const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  //   debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
  //     "debug/cyclic_time_ms", cyclic_time_ms);
  //   debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
  //     "debug/processing_time_ms", processing_time_ms);
  // }
}

}  // namespace centerpoint
