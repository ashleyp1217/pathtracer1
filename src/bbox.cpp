#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
    
  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
    
    Vector3D origin = r.o;
    Vector3D dir = r.d;
    double tx1 = (min.x - origin.x) / dir.x;
    double tx2 = (max.x - origin.x) / dir.x;
    if (tx2 < tx1){ //t1 is min, t2 is max
        std::swap(tx1, tx2);
    }

    double ty1 = (min.y - origin.y) / dir.y;
    double ty2 = (max.y - origin.y) / dir.y;
    if (ty2 < ty1){ //t1 is min, t2 is max
        std::swap(ty1, ty2);
    }


    double tz1 = (min.z - origin.z) / dir.z;
    double tz2 = (max.z - origin.z) / dir.z;
    if (tz2 < tz1){ //t1 is min, t2 is max
        std::swap(tz1, tz2);
    }

    double tMin = std::max(tx1, std::max(ty1, tz1));
    double tMax = std::min(tx2, std::min(ty2, tz2));


    t0 = tMin;
    t1 = tMax;
    if ((tMin <= tMax) && tMax >= 0) return true;
    return false;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
