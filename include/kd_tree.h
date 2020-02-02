#ifndef LIDAR_OBSTACLE_DETECTION_KD_TREE_H_
#define LIDAR_OBSTACLE_DETECTION_KD_TREE_H_

#include <pcl/common/common.h>

#include <vector>

template <typename PointT>
struct Node {
  PointT point;
  int id;
  Node* left;
  Node* right;

  Node(PointT p, int pid) : point(p), id(pid), left(nullptr), right(nullptr) {}
};

template <typename PointT>
class KdTree {
 public:
  KdTree() : root(nullptr) {}
  ~KdTree() = default;

  void SetInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
  std::vector<int> Search(PointT target, float distance_tolerance);

 private:
  Node<PointT>* root;

  void Insert(PointT point, int id);
  void InsertHelper(Node<PointT>** node, unsigned int depth, PointT point, int id);
  void SearchHelper(PointT target, Node<PointT>* node, int depth, float distance_tolerance, std::vector<int>& ids);
};

#endif
