#include "../include/cluster.h"

template <typename PointT>
void ClusterHelper(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed,
                   KdTree<PointT>* tree, float distance_tolerance, int min_size, int max_size) {
  processed[index] = true;
  cluster.push_back(index);

  std::vector<int> nearest = tree->Search(cloud->points[index], distance_tolerance);

  for (int id : nearest) {
    if (!processed[id] && cluster.size() < max_size) {
      ClusterHelper(id, cloud, cluster, processed, tree, distance_tolerance, min_size, max_size);
    }
  }
}

template <typename PointT>
std::vector<std::vector<int>> EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distance_tolerance,
                                               int min_size, int max_size) {
  std::vector<std::vector<int>> clusters;

  std::vector<bool> processed(cloud->points.size(), false);

  int i = 0;
  while (i < cloud->points.size()) {
    if (processed[i]) {
      i++;
      continue;
    }

    std::vector<int> cluster;
    ClusterHelper(i, cloud, cluster, processed, tree, distance_tolerance, min_size, max_size);
    if (cluster.size() > min_size) {
      clusters.emplace_back(cluster);
    }

    i++;
  }
  return clusters;
}
