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
  double tx_min = (min.x - r.o.x) / r.d.x;
  double tx_max = (max.x - r.o.x) / r.d.x;
  if (tx_min > tx_max) {
    std::swap(tx_min, tx_max);
  }
  double ty_min = (min.y - r.o.y) / r.d.y;
  double ty_max = (max.y - r.o.y) / r.d.y;
  if (ty_min > ty_max) {
    std::swap(ty_min, ty_max);
  }
  double tz_min = (min.z - r.o.z) / r.d.z;
  double tz_max = (max.z - r.o.z) / r.d.z;
  if (tz_min > tz_max) {
    std::swap(tz_min, tz_max);
  }
  double t_min = std::max(std::max(tx_min, ty_min), tz_min);
  double t_max = std::min(std::min(tx_max, ty_max), tz_max);

  bool hit = t_min <= t_max;
  if (t_min > t0) {
    t0 = t_min;
  }
  if (t_max < t1) {
    t1 = t_max;
  }
  return hit;
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
