#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
    double a = dot(r.d, r.d);
    double b = dot((2*(r.o - o)), r.d);
    double c = (dot(r.o - o , r.o - o)) - r2;
    if ((b*b)-(4*a*c) >= 0){
        Vector2D times = Vector2D((-b+sqrt((b*b)-(4*a*c)))/(2*a), (-b-sqrt((b*b)-(4*a*c)))/(2*a));
        t1 = min(times.x, times.y);
        t2 = max(times.x, times.y);

        bool t1in = r.min_t<=t1 && t1<=r.max_t;
        bool t2in = r.min_t<=t2 && t2<=r.max_t;
        bool t1out = !t1in;
        bool t2out = !t2in;
        if (t1out && t2out) return false;
        else {
            if (t2in && t1out) r.max_t = t2;
            else if (t1in && t2out) r.max_t = t1;
            else if (t1in && t2in) r.max_t = t1;
            return true;
        }
    }
    else{
        return false;
    }
}

bool Sphere::intersect(const Ray& r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  
    double t1;
    double t2;
    return test(r, t1, t2);
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  
    double t1;
    double t2;
    
    if (test(r, t1, t2)){
        i->t = r.max_t;
        Vector3D n = Vector3D(r.o + (r.max_t*r.d) - o);
        n.normalize();
        i->n = n;
        i->primitive = this;
        i->bsdf = this->get_bsdf();
        return true;
    }
    return false;
}

void Sphere::draw(const Color& c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c, float alpha) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
