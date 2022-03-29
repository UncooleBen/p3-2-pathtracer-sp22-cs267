#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  for (int sample_idx=0; sample_idx<num_samples; sample_idx++) {
    Vector3D w_in = hemisphereSampler->get_sample();
    // Surface normal may have opposite direction with w_out
    // In this case, we need to flip w_in
    if (w_out.z < 0) {
      w_in = - w_in;
    }
    Ray in_ray(hit_p, o2w * w_in);
    in_ray.min_t = EPS_F;
    // Calculate Li(p, wi)
    Intersection in_isect;
    if (bvh->intersect(in_ray, &in_isect)) {
      Vector3D L_in = in_isect.bsdf->get_emission();
      // f(p, wi->wo) * Li(p, wi) * cos(theta)
      L_out += isect.bsdf->f(w_out, w_in) * L_in * abs(w_in.z);
    }
  }
  // Normalize by hemisphere
  L_out *= 2 * PI;
  // Average out num_samples
  L_out /= num_samples;
  return L_out / 2;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  int num_samples = 0;
  Vector3D L_out;

  for (SceneLight* light : scene->lights) {
    int ns_current_light;
    if (light->is_delta_light()) {
      // We sample point light only once
      ns_current_light = 1;
    } else {
      ns_current_light = ns_area_light;
    }
    // Update total num of samples
    num_samples += ns_current_light;

    for (int sample_idx=0; sample_idx<ns_current_light; sample_idx++) {
      Vector3D world_w_in;
      double disToLight, pdf;
      // Sample Li(p, wi)
      Vector3D L_in = light->sample_L(hit_p, &world_w_in, &disToLight, &pdf);

      Ray in_ray(hit_p, world_w_in);
      in_ray.min_t = EPS_F;
      in_ray.max_t = disToLight / in_ray.d.norm() - EPS_F;
      
      // In this case, not intersecting (blocking) with the shadow ray means the light shines on p_hit
      if (!bvh->has_intersection(in_ray)) {
        Vector3D w_in = w2o * world_w_in;
        // f(p, wi->wo) * Li(p, wi) * cos(theta) / p(wi)
        L_out += isect.bsdf->f(w_out, w_in) * L_in * abs(w_in.z) / pdf;
      }
    }
  }

  L_out /= num_samples;
  return L_out / 2;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  double rr_terminate_p = 0.3;
  // first bounce OR later bounce lucky enough to pass the RR
  if ((r.depth == max_ray_depth) || (r.depth > 0 && !coin_flip(rr_terminate_p))) {
    // Get next bounce radiance
    L_out += one_bounce_radiance(r, isect);

    // Recursion with depth - 1
    Vector3D w_in;
    double pdf;
    isect.bsdf->sample_f(w_out, &w_in, &pdf);

    Vector3D world_w_in = o2w * w_in;
    Ray in_ray(hit_p, world_w_in);
    in_ray.min_t = EPS_F;
    in_ray.depth = r.depth - 1;

    Intersection in_isect;

    if (bvh->intersect(in_ray, &in_isect)) {
      L_out += pdf * at_least_one_bounce_radiance(in_ray, in_isect);
    }
  }
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
#ifdef DEBUG_INDICATOR
  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  return L_out;
#endif

  switch (max_ray_depth)
  {
  case 0:
    L_out += zero_bounce_radiance(r, isect);
    break;
  case 1:
    L_out += zero_bounce_radiance(r, isect);
    L_out += one_bounce_radiance(r, isect);
    break;
  default:
    L_out += zero_bounce_radiance(r, isect);
    L_out += at_least_one_bounce_radiance(r, isect);
    break;
  }
  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Vector3D pixel_color(0, 0, 0);
#define MY_PART_5 1
// Part 5
#ifdef MY_PART_5
  double sample_sum = 0.0, sample_squared_sum = 0.0;
#endif
  int ray_idx;
  for (ray_idx=0; ray_idx<num_samples; ray_idx++) {
    Vector2D sample = gridSampler->get_sample() + origin;
    sample.x /= sampleBuffer.w;
    sample.y /= sampleBuffer.h;
    Ray sample_ray = camera->generate_ray(sample.x, sample.y);
    sample_ray.depth = max_ray_depth;
    Vector3D pixel_radiance = est_radiance_global_illumination(sample_ray);
    pixel_color += pixel_radiance;
#ifdef MY_PART_5
    sample_sum += pixel_radiance.illum();
    sample_squared_sum += pow(pixel_radiance.illum(), 2);

    int n = ray_idx + 1;
    if ((n % samplesPerBatch) == 0) {
      double mean = sample_sum / n;
      double squared_mean = sample_squared_sum / n;
      double variance = (sample_squared_sum - pow(sample_sum, 2) / n) / (n - 1);
      double indicator = 1.96 * sqrt(variance / n);
      if (indicator <= maxTolerance * mean) {
        ray_idx ++;
        break;
      }
    }
#endif
  }
  pixel_color /= ray_idx;
  sampleBuffer.update_pixel(pixel_color, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = ray_idx;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
