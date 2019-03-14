#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  BBox bb(p1);
  bb.expand(p2); 
  bb.expand(p3);
  return bb;

}

bool Triangle::intersect(const Ray& r) const {
    // TODO (Part 1.3):
    // implement ray-triangle intersection
    Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
    double tmin = r.min_t;
    double tmax = r.max_t;
    
    Vector3D E1 = p2 - p1;
    Vector3D E2 = p3 - p1;
    Vector3D S = r.o - p1;
    Vector3D S1 = cross(r.d, E2);
    Vector3D S2 = cross(S, E1);
    Vector3D a = Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));
    Vector3D tbb = (1.0 / dot(S1,E1)) * a; //Vector3D(t, b1, b2)
    
    double t = tbb.x;
    double b1 = tbb.y;
    double b2 = tbb.z;
    
    if (b1 >= 0 && b2 >= 0 && b1 <= 1 && b2 <= 1 && b1 + b2 <= 1
        && t >= tmin && t <= tmax) {
        return true;
    }
    
    return false;
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
  
  // TODO (Part 1.3):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

    Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
    Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n3(mesh->normals[v3]);
    
    double tmin = r.min_t;
    double tmax = r.max_t;
    Vector3D E1 = p2 - p1;
    Vector3D E2 = p3 - p1;
    Vector3D S = r.o - p1;
    Vector3D S1 = cross(r.d, E2);
    Vector3D S2 = cross(S, E1);
    
    Vector3D a = Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));
    Vector3D tbb = (1.0 / dot(S1, E1)) * a; //Vector3D(t, b1, b2)
    
    double t = tbb.x;
    double b1 = tbb.y;
    double b2 = tbb.z;
    
    if (b1 >= 0 && b2 >= 0 && b1 <= 1 && b2 <= 1 && b1 + b2 <= 1
        && t >= tmin && t <= tmax) {
        r.max_t = t;
        isect->t = t;
        isect->n = (1-b1-b2)*n1 + (b1*n2) + (b2*n3);
        isect->primitive = this;
        isect->bsdf = this->get_bsdf();
        
        return true;
    }
    
    return false;
}

void Triangle::draw(const Color& c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CGL
