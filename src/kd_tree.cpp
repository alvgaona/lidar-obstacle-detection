#include "../include/kd_tree.h"

template <typename PointT>
void KdTree<PointT>::Insert(PointT point, int id) {
  InsertHelper(&root, 0, point, id);
}

template <typename PointT>
std::vector<int> KdTree<PointT>::Search(PointT target, float distance_tolerance) {
  std::vector<int> ids;
  SearchHelper(target, root, 0, distance_tolerance, ids);
  return ids;
}

template <typename PointT>
void KdTree<PointT>::InsertHelper(Node<PointT>** node, unsigned int depth, PointT point, int id) {
  if (*node == NULL)
    *node = new Node<PointT>(point, id);
  else {
    unsigned int current_dimension = depth % 3;

    // TODO: Reification needed in this piece of code.
    if (current_dimension == 0) {
      if (point.x < ((*node)->point.x)) {
        InsertHelper(&((*node)->left), depth + 1, point, id);
      } else {
        InsertHelper(&((*node)->right), depth + 1, point, id);
      }
    } else if (current_dimension == 1) {
      if (point.y < ((*node)->point.y)) {
        InsertHelper(&((*node)->left), depth + 1, point, id);
      } else {
        InsertHelper(&((*node)->right), depth + 1, point, id);
      }
    } else {
      if (point.z < ((*node)->point.z)) {
        InsertHelper(&((*node)->left), depth + 1, point, id);
      } else {
        InsertHelper(&((*node)->right), depth + 1, point, id);
      }
    }
  }
}

template <typename PointT>
void KdTree<PointT>::SearchHelper(PointT target, Node<PointT>* node, int depth, float distance_tolerance, std::vector<int>& ids) {
  if (node != NULL) {
    if ((node->point.x >= (target.x - distance_tolerance) && node->point.x <= (target.x + distance_tolerance)) &&
        (node->point.y >= (target.y - distance_tolerance) && node->point.y <= (target.y + distance_tolerance)) &&
        (node->point.z >= (target.z - distance_tolerance) && node->point.z <= (target.z + distance_tolerance))) {
      float distance =
          sqrt((node->point.x - target.x) * (node->point.x - target.x) + (node->point.y - target.y) * (node->point.y - target.y) +
               (node->point.z - target.z) * (node->point.z - target.z));
      if (distance <= distance_tolerance) ids.push_back(node->id);
    }

    unsigned int current_dimension = depth % 3;

    // TODO: Reification is needed in this piece of code.
    if (current_dimension == 0) {
      if ((target.x - distance_tolerance) < node->point.x) {
        SearchHelper(target, node->left, depth + 1, distance_tolerance, ids);
      }
      if ((target.x + distance_tolerance) > node->point.x) {
        SearchHelper(target, node->right, depth + 1, distance_tolerance, ids);
      }
    } else if (current_dimension == 1) {
      if ((target.y - distance_tolerance) < node->point.y) {
        SearchHelper(target, node->left, depth + 1, distance_tolerance, ids);
      }
      if ((target.y + distance_tolerance) > node->point.y) {
        SearchHelper(target, node->right, depth + 1, distance_tolerance, ids);
      }
    } else {
      if ((target.z - distance_tolerance) < node->point.z) {
        SearchHelper(target, node->left, depth + 1, distance_tolerance, ids);
      }
      if ((target.z + distance_tolerance) > node->point.z) {
        SearchHelper(target, node->right, depth + 1, distance_tolerance, ids);
      }
    }
  }
}

template <typename PointT>
void KdTree<PointT>::SetInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud) {
  for (int i = 0; i < cloud->points.size(); i++) Insert(cloud->points[i], i);
}
