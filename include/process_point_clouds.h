#ifndef PROCESS_POINT_CLOUDS_H_
#define PROCESS_POINT_CLOUDS_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "box.h"
#include "cluster.h"

template <typename PointT>
class ProcessPointClouds {
 public:
  ProcessPointClouds() = default;
  ~ProcessPointClouds() = default;

  void NumPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filter_res,
                                                    Eigen::Vector4f min_point, Eigen::Vector4f max_point);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(
      pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
      typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_threshold);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomRansacSegmentPlane(
      typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_threshold);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
                                                                int min_size, int max_size);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
                                                                      int min_size, int max_size);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  void SavePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr LoadPcd(std::string file);

  std::vector<boost::filesystem::path> StreamPcd(std::string data_path);

 private:
  std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_tolerance);
};

#endif
