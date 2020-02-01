#include "../../../include/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int set_id) : point(arr), id(set_id), left(NULL), right(NULL) {}
};

struct KdTree {
  Node* root;

  KdTree() : root(NULL) {}

  void InsertHelper(Node** node, uint depth, std::vector<float> point, int id) {
    if (*node == NULL)
      *node = new Node(point, id);
    else {
      uint current_dimension = depth % 2;
      if (point[current_dimension] < ((*node)->point[current_dimension])) {
        InsertHelper(&((*node)->left), depth + 1, point, id);
      } else {
        InsertHelper(&((*node)->right), depth + 1, point, id);
      }
    }
  }

  void Insert(std::vector<float> point, int id) { InsertHelper(&root, 0, point, id); }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> Search(std::vector<float> target, float distance_tolerance) {
    std::vector<int> ids;
    SearchHelper(target, root, 0, distance_tolerance, ids);
    return ids;
  }

  void SearchHelper(std::vector<float> target, Node* node, int depth, float distance_tolerance, std::vector<int>& ids) {
    if (node != NULL) {
      if ((node->point[0] >= (target[0] - distance_tolerance) && node->point[0] <= (target[0] + distance_tolerance)) &&
          (node->point[1] >= (target[1] - distance_tolerance) && node->point[1] <= (target[1] + distance_tolerance))) {
        float distance =
            sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
        if (distance <= distance_tolerance) {
          ids.push_back(node->id);
        }
      }

      if ((target[depth % 2] - distance_tolerance) < node->point[depth % 2]) {
        SearchHelper(target, node->left, depth + 1, distance_tolerance, ids);
      }
      if ((target[depth % 2] + distance_tolerance) > node->point[depth % 2]) {
        SearchHelper(target, node->right, depth + 1, distance_tolerance, ids);
      }
    }
  }
};
