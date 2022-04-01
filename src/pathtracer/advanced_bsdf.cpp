#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>
#include <math.h>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;
using std::abs;
//using namespace std;

namespace CGL {

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Project 3-2: Part 1
  // Implement MirrorBSDF
  *pdf = 1;
  reflect(wo, wi);
  return reflectance/std::fabs(wi->z);
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO Project 3-2: Part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  return 1.0;
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO Project 3-2: Part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.

  return Vector3D();
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO Project 3-2: Part 2
  // Implement microfacet model here.

  return Vector3D();
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Project 3-2: Part 2
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  *wi = cosineHemisphereSampler.get_sample(pdf);
  return MicrofacetBSDF::f(wo, *wi);
}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Project 3-2: Part 1
  // Implement RefractionBSDF
  if(refract(wo, wi, ior)){
    float eta = ior;;
    if (wo[2] > 0) {
        eta = 1/ior;
    }
    return transmittance / abs_cos_theta(*wi) / (eta*eta);

  }else{
    return Vector3D();
  }
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Project 3-2: Part 1
  // Compute Fresnel coefficient and either reflect or refract based on it.

  // compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305
    *pdf = 1;

    // Get the initial refract direction.
    bool res = refract(wo, wi, ior);
    if (!res) {
        return transmittance * (1/std::max(std::fabs((*wi)[2]),1e-8));
    }
    
    
    // Calculating Fr
    double ni = ior;
    double no = 1;
    double cos_i = std::fabs((*wi)[2]);
    double cos_o = std::fabs(wo[2]);
    if (wo[2] < 0) {
        swap(ni,no);
    }
    
    double r1 = (no*cos_i - ni*cos_o)/(no*cos_i + ni*cos_o);
    double r2 = (ni*cos_i - no*cos_o)/(ni*cos_i + no*cos_o);
    double Fr = 0.5*(r1*r1 + r2*r2);

    if (std::rand() / (double)RAND_MAX <= Fr) {    // If we choose reflection
        reflect(wo, wi);
        return reflectance * (1/std::max(std::fabs((*wi)[2]),1e-8));
    }
    else{                      
        double ratio = no/ni;
        return transmittance * ratio*ratio * (1/std::max(std::fabs((*wi)[2]),1e-8));
    }
}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // TODO Project 3-2: Part 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  Vector3D n = Vector3D(0.0, 0.0, 1.0);
 // *wi = (2 * dot(wo, n) * n) - wo;
  *wi = Vector3D(-wo[0],-wo[1],wo[2]);

}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // TODO Project 3-2: Part 1
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
      int sign = 1;
    float eta;
   // When z is positive,  we are entering the non-air material,
   //
    if (wo[2] > 0) {
        sign = -1;
        eta = 1/ior;
    }
    else{
      eta = ior;
    }
    
    float cos2_wi = 1 - eta*eta*(1 - wo[2]*wo[2]);
    if (cos2_wi < 0) {
        *wi = Vector3D(-wo[0],-wo[1],wo[2]);
        return false;
    }

    *wi = Vector3D(-wo[0]*eta,-wo[1]*eta, sign * sqrt(cos2_wi)).unit();


  return true;

}

} // namespace CGL
