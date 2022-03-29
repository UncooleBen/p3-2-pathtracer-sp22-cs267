#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  Vector3D n = cross(p1 - p2, p2 - p3);
  double t = dot(p1 - r.o, n) / dot(r.d, n);
  if (t < 0 || t < r.min_t || t > r.max_t) {
    return false;
  }
  Vector3D point = r.at_time(t);
  double sabc = n.norm();
  double spbc = cross(p2 - point, p3 - point).norm();
  double spac = cross(p1 - point, p3 - point).norm();
  double spab = cross(p1 - point, p2 - point).norm();
  double alpha = spbc / sabc;
  double beta = spac / sabc;
  // double gamma = 1 - alpha - beta;
  // Only works when computing gamma from area, but not from 1-alpha-beta
  // WEIRD.
  double gamma = spab / sabc;
  if (alpha >= 0 && alpha <= 1 && 
      beta >= 0 && beta <= 1 &&
      gamma >= 0 && gamma <= 1 &&
      abs(alpha + beta + gamma - 1) <= EPS_D) {
    return true;
  }
  return false;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D n = cross(p1 - p2, p2 - p3);
  double t = dot(p1 - r.o, n) / dot(r.d, n);
  if (t < 0 || t < r.min_t || t > r.max_t) {
    return false;
  }
  Vector3D point = r.at_time(t);
  double sabc = n.norm();
  double spbc = cross(p2 - point, p3 - point).norm();
  double spac = cross(p1 - point, p3 - point).norm();
  double spab = cross(p1 - point, p2 - point).norm();
  double alpha = spbc / sabc;
  double beta = spac / sabc;
  // double gamma = 1 - alpha - beta;
  // Only works when computing gamma from area, but not from 1-alpha-beta
  // WEIRD.
  double gamma = spab / sabc;
  if (alpha >= 0 && alpha <= 1 && 
      beta >= 0 && beta <= 1 &&
      gamma >= 0 && gamma <= 1 &&
      abs(alpha + beta + gamma - 1) <= EPS_D) {
    // Intersects with triangle
    // Update ray's max_t
    r.max_t = t;
    // Populate isect object
    isect->t = t;
    isect->n = alpha * n1 + beta * n2 + gamma * n3;
    isect->primitive = this;
    isect->bsdf = get_bsdf();
    return true;
  }
  return false;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
