#ifndef LIDAR_OBSTACLE_DETECTION_CLUSTER_H_
#define LIDAR_OBSTACLE_DETECTION_CLUSTER_H_

#include <chrono>
#include <string>
#include <vector>

#include "kd_tree.h"

template <typename PointT>
void ClusterHelper(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed,
                   KdTree<PointT>* tree, float distance_tolerance, int min_size, int max_size);

template <typename PointT>
std::vector<std::vector<int>> EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distance_tolerance,
                                               int min_size, int max_size);

#endif
