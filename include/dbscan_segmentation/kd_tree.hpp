#include <tbb/tbb.h>

struct Point {
    double x, y, z;
    bool core = false;
    int cluster_id = -1; // 初期値は未分類
};

struct Node
{
    Point point;   
    Node* left;
    Node* right; 
};

Node* build_kdtree(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, int depth = 0)
{
    if (begin >= end) {
        return nullptr;
    }

    int axis = depth % 2;
    auto mid = begin + (end - begin) / 2;

    std::nth_element(begin, mid, end, [axis](const Point& a, const Point& b) {
    if (axis == 0) {
      return a.x < b.x;  // Compare 'x' coordinates if axis is 0
    } else {
      return a.y < b.y;  // Compare 'y' coordinates if axis is 1
    }
    });
  
    Node* node = new Node;
    node->point = *mid;

    tbb::task_group tg;
    tg.run([&]() { 
        node->left = build_kdtree(begin, mid, depth + 1);
    });
    node->right = build_kdtree(mid + 1, end, depth + 1);
    tg.wait();
    return node;
}

float distance(const Point &p1, const Point &p2) {
  float dx = p1.x - p2.x;
  float dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

void radius_search(Node* node, const Point& search_point, double eps, std::vector<Point>& neighbors, int depth = 0) {
  if (node == nullptr || neighbors.size() >= 3) {
    return;
  }
  double dist = distance(node->point, search_point);
  if (dist <= eps) {
    neighbors.push_back(node->point);
  }
  
  int axis = depth % 2;
  double diff;
  if (axis == 0) {
    diff = search_point.x - node->point.x;
  } else {
    diff = search_point.y - node->point.y;
  }
  double eps_sqr = eps * eps;
  Node* near;
  Node* far;
  if (diff <= 0) {
    near = node->left;
    far = node->right;
  } else {
    near = node->right;
    far = node->left;
  }
  radius_search(near, search_point, eps, neighbors, depth + 1);
  if (diff * diff < eps_sqr) {
    radius_search(far, search_point, eps, neighbors, depth + 1);
  }
}

void find_neighbors(std::vector<Point>& points, double eps, int min_points) {
  Node* root = build_kdtree(points.begin(), points.end());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()), [&](const tbb::blocked_range<size_t>& r){
    for (size_t i = r.begin(); i < r.end(); ++i) {
      Point& p = points[i];
      std::vector<Point> neighbors;
      radius_search(root, p, eps, neighbors);
      p.core =  static_cast<int>(neighbors.size()) >= min_points;
    }
  });

  // Free the KD-tree nodes
  std::function<void(Node*)> free_tree = [&](Node* node) {
    if (node != nullptr) {
      free_tree(node->left);
      free_tree(node->right);
      delete node;
    }
  };
  free_tree(root);
}


