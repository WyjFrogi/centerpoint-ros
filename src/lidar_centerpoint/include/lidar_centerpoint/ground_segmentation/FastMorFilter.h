#ifndef FASTMORFILTER_H_
#define FASTMORFILTER_H_

#include <omp.h>
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fastMorFilter
{
    template <typename PointT>
    class PCL_EXPORTS FastMorFilter : public pcl::PCLBase<PointT>
    {
    public:
        typedef pcl::PointCloud<PointT> PointCloud;

        using pcl::PCLBase<PointT>::input_;
        using pcl::PCLBase<PointT>::indices_;
        using pcl::PCLBase<PointT>::initCompute;
        using pcl::PCLBase<PointT>::deinitCompute;

    public:
        FastMorFilter();

        virtual ~FastMorFilter();

        inline int getMaxWindowSize() const { return (max_window_size_); }

        /** 设定窗口最大尺寸 */
        inline void setMaxWindowSize(int max_window_size) { max_window_size_ = max_window_size; }

        inline float getSlope() const { return (slope_); }

        /** 设定坡度阈值. */
        inline void setSlope(float slope) { slope_ = slope; }

        inline float getMaxDistance() const { return (max_distance_); }

        /** 设定最大高度 */
        inline void setMaxDistance(float max_distance) { max_distance_ = max_distance; }

        inline float getInitialDistance() const { return (initial_distance_); }

        /** 设定初始高度估计 */
        inline void setInitialDistance(float initial_distance) { initial_distance_ = initial_distance; }

        inline float getCellSize() const { return (cell_size_); }

        /** 设定单元格尺寸 */
        inline void setCellSize(float cell_size) { cell_size_ = cell_size; }

        inline float getBase() const { return (base_); }

        /** 设定增长基数 */
        inline void setBase(float base) { base_ = base; }

        inline bool getExponential() const { return (exponential_); }

        /** 是否指数增长窗口尺寸 */
        inline void setExponential(bool exponential) { exponential_ = exponential; }

        /** 设定线程数
        * \param[in] nr_threads int类型, 为0时即自动选择
        */
        inline void setNumberOfThreads(unsigned int nr_threads = 0) { threads_ = nr_threads; }

        /** \brief 执行分割算法, 最终返回地面点序indices.
        * \param[out] ground vector<int>类型, 即地面点序.
        */
        virtual void extract(std::vector<int> &ground);

    protected:
        /** 滑动窗口的最大尺寸. */
        int max_window_size_;

        /** 坡度阈值, 用于计算对应的高度值. */
        float slope_;

        /** 最高地面点高度设定. */
        float max_distance_;

        /** 初始地面点高度设定. */
        float initial_distance_;

        /** 元素单元格size. */
        float cell_size_;

        /** 滑动窗口尺寸变化基数. */
        float base_;

        /** 指数增长开关 */
        bool exponential_;

        /** 线程数. */
        unsigned int threads_;
    };
} // namespace fastMorFilter

#endif
