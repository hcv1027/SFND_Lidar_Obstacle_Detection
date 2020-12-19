/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <list>
#include <memory>
#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
  ~Node() { std::cout << "Node " << id << " is released." << std::endl; }
};

struct KdTree {
  using Ptr = std::shared_ptr<KdTree>;
  Node* root;

  KdTree() : root(NULL) {}

  ~KdTree() {
    if (root != NULL) {
      deleteTree(root);
    }
    std::cout << "Kdtree is released." << std::endl;
  }

  void insert(const std::vector<float>& point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    Node** curr_node = &root;
    int depth = 0;
    while (*curr_node != NULL) {
      int idx = depth % 2;
      if (point[idx] < (*curr_node)->point[idx]) {
        curr_node = &((*curr_node)->left);
      } else {
        curr_node = &((*curr_node)->right);
      }
      depth++;
    }
    std::cout << "insert " << id << " to depth " << depth << std::endl;
    *curr_node = new Node(point, id);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    return ids;
  }

  void deleteTree(Node* node) {
    std::cout << "deleteTree, id: " << node->id << std::endl;
    if (node->left != NULL) {
      deleteTree(node->left);
      node->left = NULL;
    }
    if (node->right != NULL) {
      deleteTree(node->right);
      node->right = NULL;
    }
    delete node;
  }
};
