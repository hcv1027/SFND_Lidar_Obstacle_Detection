/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <list>
#include <memory>
#include <utility>
#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
  // ~Node() { std::cout << "Node " << id << " is released." << std::endl; }
  ~Node() = default;
};

struct KdTree {
  using Ptr = std::shared_ptr<KdTree>;
  Node* root;
  int dimension_;

  KdTree(int dimension) : root(NULL), dimension_(dimension) {
    // std::cout << "KdTree's dimension is " << dimension_ << std::endl;
  }

  ~KdTree() {
    if (root != NULL) {
      deleteTree(root);
    }
    // std::cout << "KdTree is released." << std::endl;
  }

  void insert(const std::vector<float>& point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    Node** curr_node = &root;
    int depth = 0;
    while (*curr_node != NULL) {
      int idx = depth % dimension_;
      if (point[idx] < (*curr_node)->point[idx]) {
        curr_node = &((*curr_node)->left);
      } else {
        curr_node = &((*curr_node)->right);
      }
      depth++;
    }
    // std::cout << "insert " << id << " to depth " << depth << std::endl;
    *curr_node = new Node(point, id);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;

    if (root != NULL) {
      // The pair in searchList is the pair of <depth, Node*>.
      std::list<std::pair<int, Node*>> searchList;
      searchList.push_front(std::make_pair(0, root));
      while (!searchList.empty()) {
        int depth = searchList.front().first;
        Node* node = searchList.front().second;
        searchList.pop_front();

        // Check if node is in the range of target's hyper-dimension box
        bool in_range = true;
        for (int i = 0; i < dimension_; ++i) {
          if (std::fabs(target[i] - node->point[i]) > 2 * distanceTol) {
            in_range = false;
            break;
          }
        }

        if (in_range) {
          float dist = 0.0;
          for (int i = 0; i < dimension_; ++i) {
            dist += std::pow(target[i] - node->point[i], 2);
          }
          dist = std::sqrt(dist);
          if (dist < distanceTol) {
            ids.push_back(node->id);
          }
        }

        // Check accross boundary
        int dim = depth % dimension_;
        if (target[dim] + distanceTol > node->point[dim] &&
            node->right != NULL) {
          searchList.push_front(std::make_pair(depth + 1, node->right));
        }
        if (target[dim] - distanceTol < node->point[dim] &&
            node->left != NULL) {
          searchList.push_front(std::make_pair(depth + 1, node->left));
        }
      }
    }

    return ids;
  }

  void deleteTree(Node* node) {
    // std::cout << "deleteTree, id: " << node->id << std::endl;
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
