#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

template<typename PointT>
void crop_x(typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out);

template<typename PointT>
void crop_x(typename pcl::PointCloud<PointT>::Ptr &input, pcl::IndicesPtr indices, float min, float max, std::vector<int> &indices_out);

template<typename PointT>
void crop_y(typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out);

template<typename PointT>
void crop_y(typename pcl::PointCloud<PointT>::Ptr &input, pcl::IndicesPtr indices, float min, float max, std::vector<int> &indices_out);

template<typename PointT>
void crop_x_y(typename pcl::PointCloud<PointT>::Ptr &input, float x_min, float x_max, float y_min, float y_max, std::vector<int> &indices_out);

template <typename PointT>
void crop_z(typename pcl::PointCloud<PointT>::Ptr &input, float min, float max, std::vector<int> &indices_out);

template <typename PointT>
void crop_z(typename pcl::PointCloud<PointT>::Ptr &input, pcl::IndicesPtr indices, float min, float max, std::vector<int> &indices_out);