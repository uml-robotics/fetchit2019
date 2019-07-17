#include "common_perception/crop.h"
#include <pcl/filters/passthrough.h>

template <typename PointT>
void crop(const std::string &field_name, pcl::IndicesPtr &indices, typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out) {
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(input);
    if (indices) {
        filter.setIndices(indices);
    }
    filter.setFilterFieldName (field_name);
    filter.setFilterLimits (min, max);
    filter.filter(indices_out);
}

template<typename PointT>
void crop_x(typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out) {
    pcl::IndicesPtr _;
    crop<PointT>("x", _, input, min, max, indices_out);
}

template<typename PointT>
void crop_x(typename pcl::PointCloud<PointT>::Ptr &input, pcl::IndicesPtr indices, float min, float max, std::vector<int> &indices_out) {
    crop<PointT>("x", indices, input, min, max, indices_out);
}

template<typename PointT>
void crop_y(typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out) {
    pcl::IndicesPtr _;
    crop<PointT>("y", _, input, min, max, indices_out);
}

template<typename PointT>
void crop_y(typename pcl::PointCloud<PointT>::Ptr &input, pcl::IndicesPtr indices, float min, float max, std::vector<int> &indices_out) {
    crop<PointT>("y", indices, input, min, max, indices_out);
}

template<typename PointT>
void crop_x_y(typename pcl::PointCloud<PointT>::Ptr &input, float x_min, float x_max, float y_min, float y_max, std::vector<int> &indices_out) {
    auto x_cropped_indices = pcl::IndicesPtr(new std::vector<int>);
    crop_x<PointT>(input, x_min, x_max, *x_cropped_indices);
    crop_y<PointT>(input, x_cropped_indices, y_min, y_max, indices_out);
}

template <typename PointT>
void crop_z(typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out) {
    pcl::IndicesPtr _;
    crop<PointT>("z", _, input, min, max, indices_out);
}

template <typename PointT>
void crop_z(typename pcl::PointCloud<PointT>::Ptr &input, pcl::IndicesPtr indices, float min, float max, std::vector<int> &indices_out) {
    crop<PointT>("z", indices, input, min, max, indices_out);
}

template void crop<pcl::PointNormal>(const std::string &, pcl::IndicesPtr &, typename pcl::PointCloud<pcl::PointNormal>::Ptr &, float , float , std::vector<int> &);
template void crop<pcl::PointXYZ>(const std::string &, pcl::IndicesPtr &, typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, float , float , std::vector<int> &);

template void crop_x<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &, float , float , std::vector<int> &);
template void crop_x<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, float , float , std::vector<int> &);
template void crop_x<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &, pcl::IndicesPtr, float , float , std::vector<int> &);
template void crop_x<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, pcl::IndicesPtr, float , float , std::vector<int> &);

template void crop_y<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &, float , float , std::vector<int> &);
template void crop_y<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, float , float , std::vector<int> &);
template void crop_y<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &, pcl::IndicesPtr, float , float , std::vector<int> &);
template void crop_y<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, pcl::IndicesPtr, float , float , std::vector<int> &);

template void crop_x_y<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float x_min, float x_max, float y_min, float y_max, std::vector<int> &indices_out);
template void crop_x_y<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &input, float x_min, float x_max, float y_min, float y_max, std::vector<int> &indices_out);

template void crop_z<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &, float , float , std::vector<int> &);
template void crop_z<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, float , float , std::vector<int> &);
template void crop_z<pcl::PointNormal>(typename pcl::PointCloud<pcl::PointNormal>::Ptr &, pcl::IndicesPtr, float , float , std::vector<int> &);
template void crop_z<pcl::PointXYZ>(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &, pcl::IndicesPtr, float , float , std::vector<int> &);