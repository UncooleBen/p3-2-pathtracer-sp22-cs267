#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

// #define PART2_NO_BVH 1

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
#ifdef PART2_NO_BVH
  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;

  return node;
#else
  BBox bbox;
  if (end - start <= max_leaf_size) {
    // This is a leaf node
    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      bbox.expand(bb);
    }

    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;

    return node;
  }
  // This is an interior node
  // Find the axis with most biggest range
  double min_x = INF_D, max_x = -INF_D, min_y = INF_D, max_y = -INF_D, min_z = INF_D, max_z = -INF_D;
  for (auto it=start; it!=end; it++) {
    Vector3D centroid = (*it)->get_bbox().centroid();
    min_x = min(min_x, centroid.x);
    max_x = max(max_x, centroid.x);
    min_y = min(min_y, centroid.y);
    max_y = max(max_y, centroid.y);
    min_z = min(min_z, centroid.z);
    max_z = max(max_z, centroid.z);
  }
  int vector_idx;
  if (max_x - min_x > max_y - min_y) {
    vector_idx = 0;
    if (max_x - min_x < max_z - min_z) {
      vector_idx = 2;
    }
  } else {
    vector_idx = 1;
    if (max_y - min_y < max_z - min_z) {
      vector_idx = 2;
    }
  }
  std::vector<double> coords;
  for (auto it=start; it!=end; it++) {
    Vector3D centroid = (*it)->get_bbox().centroid();
    coords.emplace_back(centroid[vector_idx]);
  }
  std::sort(coords.begin(), coords.end(), std::less<double>());
  double median = coords[coords.size() / 2];
  auto mid = std::partition(start, end, 
    [vector_idx, median](const Primitive* p) {
      return p->get_bbox().centroid()[vector_idx] < median;
    }
  );
  if (mid - start == 0 || end - mid == 0) {
    mid = start + (end - start) / 2;
  }
  BVHNode* left_child = construct_bvh(start, mid, max_leaf_size);
  BVHNode* right_child = construct_bvh(mid, end, max_leaf_size);

  bbox.expand(left_child->bb);
  bbox.expand(right_child->bb);

  BVHNode *node = new BVHNode(bbox);
  node->l = left_child;
  node->r = right_child;

  return node;
#endif
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
#ifdef PART2_NO_BVH
  for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  return false;
#else
  double t0 = -INF_D, t1 = INF_D;
  bool hit = node->bb.intersect(ray, t0, t1);
  if (!hit) {
    return false;
  }

  // Hits the bbox. Still need test primitives.
  // This is a leaf node
  if (node->l == nullptr && node->r == nullptr) {
    hit = false;
    for (auto it=node->start; it!=node->end; it++) {
      hit = (*it)->has_intersection(ray);
      if (hit) {
        break;
      }
    }
    return hit;
  }
  // This is an interior node
  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
#endif
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // The ray misses the bbox
#ifdef PART2_NO_BVH
  bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;
#else
  double t0 = -INF_D, t1 = INF_D;
  bool hit = node->bb.intersect(ray, t0, t1);
  if (!hit) {
    return false;
  }

  // Hits the bbox. Still need test primitives.
  // This is a leaf node
  if (node->l == nullptr && node->r == nullptr) {
    hit = false;
    for (auto it=node->start; it!=node->end; it++) {
      total_isects++;
      hit = (*it)->intersect(ray, i) || hit;
    }
    return hit;
  }
  // This is an interior node
  bool hit_left = intersect(ray, i, node->l);
  bool hit_right = intersect(ray, i, node->r);
  return hit_left || hit_right;
#endif
}

} // namespace SceneObjects
} // namespace CGL
