#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c, alpha);
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c, alpha);
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
    // TODO (Part 2.1):
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code build a BVH aggregate with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    BBox centroid_box, bbox;

    for (Primitive *p : prims) {
      BBox bb = p->get_bbox();
      bbox.expand(bb);
      Vector3D c = bb.centroid();
      centroid_box.expand(c);
    }
    BVHNode *node = new BVHNode(bbox);
    
    if (prims.size() <= max_leaf_size){
        node->prims = new vector<Primitive *>(prims);
        return node;
    }
    
    vector<Primitive *> right = vector<Primitive *>();
    vector<Primitive *> left = vector<Primitive *>();
    
    Vector3D midpoint = centroid_box.centroid();
    int axis;
    if ((centroid_box.extent.x >= centroid_box.extent.y)&&(centroid_box.extent.x>=centroid_box.extent.z)){ //split on x axis
        axis = 0;
    }
    else if ((centroid_box.extent.y >= centroid_box.extent.x)&&(centroid_box.extent.y>=centroid_box.extent.z)){ //split on y axis
        axis = 1;
    }
    else{ //split on z axis
        axis = 2;
    }
    
    for (Primitive *p : prims){
        if (p->get_bbox().centroid()[axis] >= midpoint[axis]){
            right.push_back(p);
        }
        else{
            left.push_back(p);
        }
    }
    
    double sizehalf = prims.size() / 2;
    if (right.empty()){
        for (int i = 0; i < sizehalf; i++){
            right.push_back(left.back());
            left.pop_back();
        }
    }
    else if (left.empty()){
        for (int i = 0; i<sizehalf; i++){
            left.push_back(right.back());
            right.pop_back();
        }
    }
    
    node->r = construct_bvh(right, max_leaf_size);
    node->l = construct_bvh(left, max_leaf_size);
    return node;
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
   
    
    double t0;
    double t1;
    if (node->bb.intersect(ray, t0, t1)){  //parameters &t0/&t1 placeholders, create before to be filled in
        if (t0 > ray.max_t || t1 < ray.min_t) {
            return false;
        }

        if (node->isLeaf()){
            bool hit = false;
            for (Primitive *p : *(node->prims)) {
                total_isects++;
                if (p->intersect(ray)) {
                    hit = true;
                }
            }
            return hit;
        }

        bool left = intersect(ray, node->l);
        bool right = intersect(ray, node->r);
        return left || right;
    }
  return false;
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.

    
    double t0;
    double t1;
    if (node->bb.intersect(ray, t0, t1)){  //parameters &t0/&t1 placeholders, create before to be filled in
        if (t0 > ray.max_t || t1 < ray.min_t) {
            return false;
        }
        
        if (node->isLeaf()){
            bool hit = false;
            for (Primitive *p : *(node->prims)) {
                total_isects++;
                if (p->intersect(ray, i)) {
                    hit = true;
                }
            }
            return hit;
        }
        
        bool left = intersect(ray, i, node->l);
        bool right = intersect(ray, i, node->r);
        return left || right;
    }
    return false;
}

}  // namespace StaticScene
}  // namespace CGL
