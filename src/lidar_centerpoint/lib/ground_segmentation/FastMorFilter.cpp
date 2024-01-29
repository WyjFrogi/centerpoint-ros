#ifndef FASTMORFILTER_CPP_
#define FASTMORFILTER_CPP_

#include <cmath>
#include "ros/ros.h"
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include "lidar_centerpoint/ground_segmentation/FastMorFilter.h"

template <typename PointT>
fastMorFilter::FastMorFilter<PointT>::FastMorFilter() : max_window_size_(20),
                                                        slope_(1.0f),
                                                        max_distance_(1.2f),
                                                        initial_distance_(0.10f),
                                                        cell_size_(0.5f),
                                                        base_(2.0f),
                                                        exponential_(true),
                                                        threads_(0)
{
}

template <typename PointT>
fastMorFilter::FastMorFilter<PointT>::~FastMorFilter() {}

template <typename PointT>
void fastMorFilter::FastMorFilter<PointT>::extract(std::vector<int> &ground)
{
    bool segmentation_is_possible = initCompute();
    if (!segmentation_is_possible)
    {
        deinitCompute();
        return;
    }

    // 滤波超参设置，包括窗口大小、高度阈值
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;
    std::vector<int> half_sizes; // 即窗口边长的1/2, 用于滑动窗口时范围的选择
    int iteration = 0;
    int half_size = 0.0f;
    float window_size = 0.0f;
    float height_threshold = 0.0f;

    // double test1 = ros::Time::now().toSec() * 1000;
    while (window_size < max_window_size_)
    {
        // 设定窗口扩展方式
        if (exponential_)
            half_size = static_cast<int>(std::pow(static_cast<float>(base_), iteration));
        else
            half_size = (iteration + 1) * base_;

        // half_size: 2 4

        window_size = 2 * half_size + 1;

        if (window_size > max_window_size_)
        {
            window_size = max_window_size_; // 锁定最大窗口尺寸
        }
        // window_size: 5 8
        // std::cout << "window_size: " << window_size << std::endl;

        // 设定高度阈值变化方式
        if (iteration == 0)
            height_threshold = initial_distance_;
        else
            height_threshold = slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_;

        // 高度阈值不应超过设定的最大高度 -> 限制了坡道的垂直高度(对较大的坡度有影响，若超过设定高度，则需要设定足够远的避障距离)
        if (height_threshold > max_distance_)
            height_threshold = max_distance_;

        half_sizes.push_back(half_size);
        // 相较于普通PMF, 这里的窗口尺寸没有乘以cell_size_(注意cell_size默认为1.0)
        window_sizes.push_back(window_size); // 仅用于设定下一高度阈值
        height_thresholds.push_back(height_threshold);

        iteration++;
    }
    // double test2 = ros::Time::now().toSec() * 1000;
    // ROS_INFO("shedingchuangkoudaxiao: %lf ms", test2 - test1);

    // 将整体点云转为栅格图
    Eigen::Vector4f global_max, global_min;
    pcl::getMinMax3D<PointT>(*input_, global_min, global_max); // 获取传入点云的整体尺寸

    float xextent = global_max.x() - global_min.x(); // 确定栅格图范围
    float yextent = global_max.y() - global_min.y();

    int rows = static_cast<int>(std::floor(yextent / cell_size_) + 1); // 根据单格大小，确定栅格行列总数量，这里是行
    int cols = static_cast<int>(std::floor(xextent / cell_size_) + 1); // 列

    // 设定开算子中间量(窗口内点云最低高度)
    Eigen::MatrixXf A(rows, cols);
    A.setConstant(std::numeric_limits<float>::quiet_NaN());

    Eigen::MatrixXf Z(rows, cols);
    Z.setConstant(std::numeric_limits<float>::quiet_NaN());

    Eigen::MatrixXf Zf(rows, cols);
    Zf.setConstant(std::numeric_limits<float>::quiet_NaN());

    for (int i = 0; i < (int)input_->points.size(); ++i)
    {
        // 找每个格子内的最低点值
        PointT p = input_->points[i];
        int row = std::floor((p.y - global_min.y()) / cell_size_); // 确定当前的p点处于哪个格子
        int col = std::floor((p.x - global_min.x()) / cell_size_);

        if (p.z < A(row, col) || std::isnan(A(row, col)))
        {
            // 若栅格中对应点位还没赋值, 或是当前点的z值更小, 则替换为此点的值; 循环结束后, A中各个格子的值都是格子内的最低高程值
            A(row, col) = p.z;
        }
    }
    // double test3 = ros::Time::now().toSec() * 1000;
    // ROS_INFO("quedingzuididian: %lf ms", test3 - test2);

    // 地面点序初保存, 用于圈定范围
    ground = *indices_;

    // 使用形态学开操作，渐进过滤地面回波
#if 0 // 查看迭代过程中各数据情况
    for (size_t i = 0; i < window_sizes.size(); ++i)
    {
        std::cout << "The max window size: " << max_window_size_ << std::endl;
        std::cout << "window " << i << " size: " << window_sizes[i] << std::endl;
        std::cout << "height_thresholds " << i << " size: " << height_thresholds[i] << std::endl;
        std::cout << "half_sizes " << i << " size: " << half_sizes[i] << std::endl;
    }
#endif

    // double t_st = ros::Time::now().toSec() * 1000;
    for (size_t i = 0; i < window_sizes.size(); ++i)
    {
        PCL_DEBUG("      Iteration %d (height threshold = %f, window size = %f, half size = %d)...",
                  i, height_thresholds[i], window_sizes[i], half_sizes[i]);

        // 将滤波限制在当前考虑的地面回波点
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud<PointT>(*input_, ground, *cloud);

        // double test4 = ros::Time::now().toSec() * 1000;
        // 执行开运算(腐蚀+膨胀)
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
        for (int row = 0; row < rows; ++row) // 按栅格 行 进行遍历，原方法是按 点 遍历
        {
            int rs, re; // 设定窗口在行的位置，不能小于最小边界0，不能大于最大边界rows-1
            rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
            re = ((row + half_sizes[i]) > (rows - 1)) ? (rows - 1) : row + half_sizes[i];

            for (int col = 0; col < cols; ++col)
            {
                int cs, ce; // 设定窗口在列的位置
                cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
                ce = ((col + half_sizes[i]) > (cols - 1)) ? (cols - 1) : col + half_sizes[i];

                float min_coeff = std::numeric_limits<float>::max();

                // 遍历窗口，对窗口内值进行操作
                for (int j = rs; j < (re + 1); ++j) // 每行(窗口)
                {
                    for (int k = cs; k < (ce + 1); ++k) // 每列(窗口)
                    {
                        if (A(j, k) != std::numeric_limits<float>::quiet_NaN())
                        {
                            if (A(j, k) < min_coeff)
                                min_coeff = A(j, k); // 腐蚀，只保留窗口内最小的高程
                        }
                    }
                }

                if (min_coeff != std::numeric_limits<float>::max())
                    Z(row, col) = min_coeff; // 记录当前最新的最低高程值
            }
        }
        // double test5 = ros::Time::now().toSec() * 1000;
        // ROS_INFO("fushiyunsuan: %lf ms", test5 - test4);

#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
        for (int row = 0; row < rows; ++row)
        {
            int rs, re;
            rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
            re = ((row + half_sizes[i]) > (rows - 1)) ? (rows - 1) : row + half_sizes[i];

            for (int col = 0; col < cols; ++col)
            {
                int cs, ce;
                cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
                ce = ((col + half_sizes[i]) > (cols - 1)) ? (cols - 1) : col + half_sizes[i];

                float max_coeff = -std::numeric_limits<float>::max();

                for (int j = rs; j < (re + 1); ++j)
                {
                    for (int k = cs; k < (ce + 1); ++k)
                    {
                        if (Z(j, k) != std::numeric_limits<float>::quiet_NaN())
                        {
                            if (Z(j, k) > max_coeff)
                                max_coeff = Z(j, k); // 膨胀，只保留窗口内最大的高程
                        }
                    }
                }

                if (max_coeff != -std::numeric_limits<float>::max())
                    Zf(row, col) = max_coeff; //由于max来自与Z，Z来自于腐蚀，因此Zf均为区域极小值;可视作理想地面点的高度
            }
        }
        // double test6 = ros::Time::now().toSec() * 1000;
        // ROS_INFO("pengzhangyunsuan: %lf ms", test6 - test5);

        // 比较各点高程与高度阈值, 阈值以内的均设为地面点
        std::vector<int> pt_indices;
        for (size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
        {
            PointT p = cloud->points[p_idx];
            // if (p.z > 1.8)
            // {
            //     pt_indices.push_back(ground[p_idx]);
            //     continue;
            // }

            int erow = static_cast<int>(std::floor((p.y - global_min.y()) / cell_size_));
            int ecol = static_cast<int>(std::floor((p.x - global_min.x()) / cell_size_));

            float diff = abs(p.z - Zf(erow, ecol));

            if (diff < height_thresholds[i]) // 落差在高度阈值内的即设为地面点
                pt_indices.push_back(ground[p_idx]);
        }

        A.swap(Zf);

        // Ground is now limited to pt_indices
        ground.swap(pt_indices);

        // double test7 = ros::Time::now().toSec() * 1000;
        // ROS_INFO("quedingdimiandianyunsuan: %lf ms", test7 - test6);

        PCL_DEBUG("ground now has %d points\n", ground.size());
    }
    // double t_end = ros::Time::now().toSec() * 1000;
    // std::cout << "whole windows scan coats time: " << t_end - t_st << std::endl;

    deinitCompute();
}

#endif // FASTMORFILTER_CPP_
