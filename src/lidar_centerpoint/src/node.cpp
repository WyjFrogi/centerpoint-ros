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
  nh.param<float>("score_threshold", score_threshold, 0.35);
  nh.param<float>("circle_nms_dist_threshold", circle_nms_dist_threshold, 0.0);
  nh.getParam("yaw_norm_thresholds", yaw_norm_thresholds);
  nh.param("densification_world_frame_id", densification_world_frame_id, std::string("map"));
  nh.param<int>("densification_num_past_frames", densification_num_past_frames, 1);
  nh.param("trt_precision", trt_precision, std::string("fp16"));
  nh.param("encoder_onnx_path", encoder_onnx_path, std::string("pts_voxel_encoder_centerpoint.onnx"));
  nh.param("encoder_engine_path", encoder_engine_path, std::string("pts_voxel_encoder_centerpoint.engine"));
  nh.param("head_onnx_path", head_onnx_path, std::string("pts_backbone_neck_head_centerpoint.onnx"));
  nh.param("head_engine_path", head_engine_path, std::string("pts_backbone_neck_head_centerpoint.engine"));
  nh.getParam("class_names", class_names_);
  nh.param<bool>("has_twist", has_twist_, false);
  nh.param<int>("point_feature_size", point_feature_size, 0);
  nh.param<int>("max_voxel_size", max_voxel_size, 0);
  nh.getParam("point_cloud_range", point_cloud_range);
  nh.getParam("voxel_size", voxel_size);
  nh.param<int>("downsample_factor", downsample_factor, 0);
  nh.param<int>("encoder_in_feature_size", encoder_in_feature_size, 0);
  nh.getParam("allow_remapping_by_area_matrix", allow_remapping_by_area_matrix);
  nh.getParam("min_area_matrix", min_area_matrix);
  nh.getParam("max_area_matrix", max_area_matrix);
  ROS_INFO("init param");
  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  { 
    NMSParams p;
    p.nms_type_ = NMS_TYPE::IoU_BEV;
    nh.getParam("iou_nms_target_class_names", p.target_class_names_);
    nh.param("iou_nms_search_distance_2d", p.search_distance_2d_, 0.0);
    nh.param("iou_nms_threshold", p.iou_threshold_, 0.0);
    iou_bev_nms_.setParameters(p);
  }

  NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);
  NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);

  if (point_cloud_range.size() != 6) {
    ROS_WARN_STREAM("The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    ROS_WARN_STREAM("The size of voxel_size != 3: use the default parameters.");
  }
  CenterPointConfig config(
    class_names_.size(), point_feature_size, max_voxel_size, point_cloud_range, voxel_size,
    downsample_factor, encoder_in_feature_size, score_threshold, circle_nms_dist_threshold,
    yaw_norm_thresholds);
  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);
  ROS_INFO("init config param");
  pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
    "~/input/pointcloud", 10, &LidarCenterPointNode::pointCloudCallback, this);
  objects_pub_ = nh.advertise<perception_msgs::DetectedObjects>(
    "~/output/objects", 10);
  ROS_INFO("init complete");
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
  PointCloudT::Ptr input_pointcloud = PointCloudT().makeShared();
  pcl::fromROSMsg(*input_pointcloud_msg, *input_pointcloud);
  LidarCenterPointNode::cloudsProcess(input_pointcloud);
  sensor_msgs::PointCloud2 preprocess_cloud;
  pcl::toROSMsg(*input_pointcloud, preprocess_cloud);
  const auto objects_sub_count = objects_pub_.getNumSubscribers();
  if (objects_sub_count < 1) {
    return;
  }


  std::vector<Box3D> det_boxes3d;
  bool is_success = detector_ptr_->detect(preprocess_cloud, tf_buffer_, det_boxes3d);
  if (!is_success) {
    return;
  }

  std::vector<perception_msgs::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    perception_msgs::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, has_twist_, obj);
    raw_objects.emplace_back(obj);
  }

  perception_msgs::DetectedObjects output_msg;
  output_msg.header = input_pointcloud_msg->header;
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  if (objects_sub_count > 0) {
    objects_pub_.publish(output_msg);
  }

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

void LidarCenterPointNode::cloudsProcess(PointCloudT::Ptr &source_pointcloud)
{
  // ROS_INFO_STREAM("source_pointcloud->header.frame_id: " << source_pointcloud->header.frame_id);
  // std::cout << "source_pointcloud->header.frame_id: " << source_pointcloud->header.frame_id << std::endl;
  // std::cout << "ROS TIME NOW: " << source_pointcloud->header.stamp << std::endl;
  bool lidarMF = false;

  if (source_pointcloud->header.frame_id == "livox_frame") 
      lidarMF = true;



  PointCloudT::Ptr cloud_filtered(new PointCloudT());
  // PointCloudT::Ptr cloud_filtered(new PointCloudT());
  *cloud_filtered = *source_pointcloud;
  ROS_INFO("Before voxelgrid filter, the point number: %ld", cloud_filtered->size());

  auto ROITime_start = std::chrono::steady_clock::now(); // ROI开始时间

  PointCloudT::Ptr clz_cloud(new PointCloudT()); // use for close area

  //  ROI空间 - 大空间
  if (lidarMF)
  {
      Eigen::Vector4f min_point;
      Eigen::Vector4f max_point;
      min_point << point_cloud_range[0], point_cloud_range[1], point_cloud_range[2],1.0;
      max_point << point_cloud_range[3],  point_cloud_range[4], point_cloud_range[5],1.0;

      pcl::CropBox<pcl::PointXYZI> crop_left;
      crop_left.setInputCloud(cloud_filtered);
      crop_left.setMin(min_point);
      crop_left.setMax(max_point);
      
      crop_left.setNegative(false); // false为保留
      crop_left.filter(*cloud_filtered);
  }
  ROS_INFO("After ROI choise, the point number: %ld", cloud_filtered->size());

  if (lidarMF)
  {
      pcl::PointIndices indices;
      pcl::ExtractIndices<pcl::PointXYZI> cliper;
      cliper.setInputCloud(cloud_filtered);
      for (size_t i = 0; i < cloud_filtered->points.size(); i++)
      {
          if ((cloud_filtered->points[i].z > -0.5 && cloud_filtered->points[i].z < 0.8) &&
              (cloud_filtered->points[i].x > -0.35 && cloud_filtered->points[i].x <= 1.5) &&
              (cloud_filtered->points[i].y > -0.5 && cloud_filtered->points[i].y < 0.55))
          {
              indices.indices.push_back(i);
          }
      }
      cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
      cliper.setNegative(true);
      cliper.filter(*cloud_filtered);
      ROS_INFO("After ROI and no car point choise, the  point number: %ld", cloud_filtered->size());
  }
  auto ROITime_end = std::chrono::steady_clock::now(); // ROI结束时刻时间（ROI和去除车身）
  auto ROI_Time = std::chrono::duration_cast<std::chrono::milliseconds>(ROITime_end - ROITime_start);
  ROS_INFO("ROI and no car costs time: %lu ms", ROI_Time.count());

  // #######################################
  // 降采样 - 体素滤波/统计滤波
  // #######################################
  if (lidarMF)
  {
      // 体素滤波
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud_filtered);
      vg.setLeafSize(0.08f, 0.08f, 0.08f); // 设置滤波时创建的体素体积为 cm*cm*cm的立方体
      // vg.setMinimumPointsNumberPerVoxel(8); // 设置构成体素所需的最小点数, 用于过滤掉数量较少的噪点
      vg.filter(*cloud_filtered);
      // sensor_msgs::PointCloud2 filter_vg_points;
      // pcl::toROSMsg(*cloud_filtered, filter_vg_points);
      // filter_vg_points.header.frame_id = output_frame_id;
      // pub_vg.publish(filter_vg_points);

      // 统计滤波器，删除离群点
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> Static;
        Static.setInputCloud(cloud_filtered);
        Static.setMeanK(8); //设置在进行统计时考虑查询点临近点数
        Static.setStddevMulThresh(0.5); //设置判断是否为离群点的阀值
        Static.filter(*cloud_filtered);
  }
  auto downsample_endTime = std::chrono::steady_clock::now();
  ROS_INFO("After voxelgrid filter, the point number: %ld", cloud_filtered->size());
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(downsample_endTime - ROITime_end);
  ROS_INFO("[test2test2test2]Voxelgrid downsampling costs time: %lu ms", elapsedTime.count());

#if POINTCLOUD_FILTER
      sensor_msgs::PointCloud2 filter_tree_points;
      pcl::toROSMsg(*cloud_filtered, filter_tree_points);
      filter_tree_points.header.frame_id = output_frame_id;
      pub_tree.publish(filter_tree_points);
#endif
  /**
   * 去地面 - 快速渐进形态学滤波(OpenMP)
   */
  // 当点云数量为0时，不参与去地面直接聚类避免bug
  if (cloud_filtered->size() == 0)
  {
      source_pointcloud = cloud_filtered;
  }
  else
  {
      ROS_INFO_STREAM("Start OpenMp");
      auto seg_startTime = std::chrono::steady_clock::now();
      pcl::PointIndicesPtr ground(new pcl::PointIndices); // 设定点云指针index

      if(lidarMF)
      {
          ROS_INFO_STREAM("Start LF OpenMp");
          fastMorFilter::FastMorFilter<pcl::PointXYZI> mf;
          mf.setInputCloud(cloud_filtered);
          mf.setMaxWindowSize(8);
          mf.setCellSize(0.25f);
          mf.setExponential(false);
          mf.setSlope(1.0f);
          mf.setInitialDistance(0.1f);
          mf.setMaxDistance(2.5f);
          mf.extract(ground->indices);

      }

      PointCloudT::Ptr cloud_obst(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(ground);
      extract.setNegative(true);
      extract.filter(*cloud_obst);

      ROS_INFO("After  remove ground, the  point number: %ld", cloud_obst->size());
      auto seg_endTime = std::chrono::steady_clock::now();
      auto seg_ground_Time = std::chrono::duration_cast<std::chrono::milliseconds>(seg_endTime - seg_startTime);
      ROS_INFO("Ground seg by MorphologicalFilter costs time: %lu ms", seg_ground_Time.count());

      source_pointcloud = cloud_obst; // 现点云替换 原(包含地面的)点云
      
      auto obst_endTime = std::chrono::steady_clock::now();
      auto obst_ego_Time = std::chrono::duration_cast<std::chrono::milliseconds>(obst_endTime - seg_endTime);
      ROS_INFO("adding the obst around ego cat costs time: %lu ms", obst_ego_Time.count());
  }
  return;
}

}  // namespace centerpoint
