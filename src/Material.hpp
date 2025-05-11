
#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "global.hpp"
#include <Eigen/Dense>
#include <algorithm>
using namespace Eigen;

enum MaterialType { DIFFUSE, SMOOTH_CONDUCTOR, ROUGH_CONDUCTOR, SMOOTH_DIELECTRIC, ROUGH_DIELECTRIC };

class Material {
  private:
    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const {
        return I - 2 * I.dot(N) * N;
    }

    // Compute refraction direction using Snell's law
    // We need to handle with care the two possible situations:
    //    - When the ray is inside the object
    //    - When the ray is outside.
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    // If the ray is inside, you need to invert the refractive indices and
    // negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N,
                     const float &ior) const {
        float cosi = clamp(-1, 1, I.dot(N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) {
            cosi = -cosi;
        } else {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return (k < 0) ? Vector3f(0, 0, 0)
                       : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    // I is the incident view direction
    // N is the normal at the intersection point
    // ior is the material refractive index
    // kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior,
                 float &kr) const {
        float cosi = clamp(-1, 1, I.dot(N));
        float etai = 1, etat = ior;
        if (cosi > 0) {
            std::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        } else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) /
                       ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) /
                       ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is
        // given by: kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N) {
        Vector3f B, C;
        if (std::fabs(N.x()) > std::fabs(N.y())) {
            float invLen = 1.0f / std::sqrt(N.x() * N.x() + N.z() * N.z());
            C = Vector3f(N.z() * invLen, 0.0f, -N.x() * invLen);
        } else {
            float invLen = 1.0f / std::sqrt(N.y() * N.y() + N.z() * N.z());
            C = Vector3f(0.0f, N.z() * invLen, -N.y() * invLen);
        }
        B = C.cross(N);
        return a.x() * B + a.y() * C + a.z() * N;
    }

    // ==============================
    // === Start of GGX Functions ===

    // GGX Normal Distribution Function (NDF)
    inline float D_GGX(const Vector3f& h, const Vector3f& n, float alpha) {
        float NoH = std::max(n.dot(h), 0.0f);
        float alpha2 = alpha * alpha;
        float denom = (NoH * NoH) * (alpha2 - 1.0f) + 1.0f;
        return alpha2 / (M_PI * denom * denom);
    }
    // Smith Geometry Term for GGX 
    // Used to account for self-shadowing & masking in incoming & outgoing vectors
    inline float G_SmithGGX_Helper(const Vector3f& v, const Vector3f& n, float alpha) {
        // cos of angle between surface normal & direction vector
        float NoV = std::max(n.dot(v), 0.0f);
        // tan of the angle between the direction vector & surface normal
        float tanTheta = std::sqrt(1.0f - NoV * NoV) / NoV;

        // when tanTheta is 0, the angle between direction vector & surface normal is perpendicular --> no masking/shadowing --> NO ATTENUATION
        if (tanTheta == 0.0f) return 1.0f;

        // Alpha: How rough the surface is. Rougher surface = more masking/shadowing
        // tanTheta: how grazing the view angle is. Large tanTheta = more masking/shadowing
        float a = 1.0f / (alpha * tanTheta);
        if (a >= 1.6f) return 1.0f;
        float a2 = a * a;

        // schlick
        // grazing angle & roughness increase -> more masking -> returns a smaller G term
        return (3.535f * a + 2.181f * a2) / (1.0f + 2.276f * a + 2.577f * a2);
    }
    inline float G_SmithGGX(const Vector3f& incoming_dir, const Vector3f& outgoing_dir, const Vector3f& n, float alpha) {
        // both incoming & outgoing direction can experience masking/shadowing
        // thus, overall geometry term G is the product of the two
        return G_SmithGGX_Helper(incoming_dir, n, alpha) * G_SmithGGX_Helper(outgoing_dir, n, alpha);
    }
    // Fresnel term using Schlick's approximation
    // used for metals, not good for refraction
    inline Vector3f FresnelSchlick(float cosTheta, const Vector3f& F0) {
        return F0 + (Vector3f(1.0f, 1.0f, 1.0f) - F0) * powf(1.0f - cosTheta, 5.0f);
    }

    // Importance sample GGX NDF
    // Given a 2D uniform random sample Xi...
    // chooses reflection directions that are likely to contribute significant energy based on statistical surface roughness
    inline Vector3f ImportanceSampleGGX(const Vector2f& Xi, float alpha, const Vector3f& n) {
        // sample random microfacet normal h from GGX NDF in tangent space
        float phi = 2.0f * M_PI * Xi.x();
        float cosTheta = std::sqrt((1.0f - Xi.y()) / (1.0f + (alpha * alpha - 1.0f) * Xi.y()));
        float sinTheta = std::sqrt(1.0f - cosTheta * cosTheta);
        Vector3f tan_space_h(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
    
        // return the world space result  
        return toWorld(tan_space_h, n).normalized();
    }
    // sample the microfacet normal h from GGX distribution  
    inline Vector3f sampleGGXMicrofacetNormal(const Vector3f& incoming_light, const Vector3f& n, float alpha) {
        Vector2f Xi(get_random_float(), get_random_float());
        return ImportanceSampleGGX(Xi, alpha, n);
    }
    // === End of GGX Functions ===
    // ============================

  public:
    MaterialType m_type;
    // Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    float roughness;
    Vector3f base_reflectance;
    // Texture tex;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    // inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &incoming_light, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &incoming_light, const Vector3f &outgoing_view, const Vector3f &N, bool isReflect);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &incoming_light, const Vector3f &outgoing_view, const Vector3f &N, bool isReflect);
};

Material::Material(MaterialType t, Vector3f e) {
    m_type = t;
    // m_color = c;
    m_emission = e;
    ior=2;
    roughness= 0.0f;
    base_reflectance=Vector3f(0,0,0);
}

MaterialType Material::getType() { return m_type; }
/// Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON)
        return true;
    else
        return false;
}

Vector3f Material::getColorAt(double u, double v) { return Vector3f::Zero(); }

Vector3f Material::sample(const Vector3f &incoming_light, const Vector3f &N) {
    switch(m_type){
        case DIFFUSE:
        {    
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);
        }
        case ROUGH_CONDUCTOR: return sampleGGXMicrofacetNormal(incoming_light, N, roughness);
        case SMOOTH_CONDUCTOR: return N;
        case ROUGH_DIELECTRIC: return sampleGGXMicrofacetNormal(incoming_light, N, roughness);
        case SMOOTH_DIELECTRIC: return N;
        default:
            return Vector3f::Zero(); // or throw an error
    }
}

// Computes the probability density of a given ray direction being generated by sample.
float Material::pdf(const Vector3f &incoming_light, const Vector3f &outgoing_view, const Vector3f &N, bool isReflect){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (outgoing_view.dot(N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
        }
        case ROUGH_CONDUCTOR:
        {
            Vector3f h_sum = incoming_light + outgoing_view;
            if (h_sum.norm() < EPSILON) return 0.0f;
            Vector3f h = (h_sum).normalized();

            float D = D_GGX(h, N, roughness);
            
            float cos_theta = outgoing_view.dot(h);
            if (cos_theta <= EPSILON) return 0.0f;
            float jacobian = 1.0f / (4.0f * cos_theta);

            return D * N.dot(h) * jacobian;
        }
        case SMOOTH_CONDUCTOR: 
            return (incoming_light.dot(reflect(-outgoing_view, N)) > 0.999f) ? 1.0f : 0.0f;
        case ROUGH_DIELECTRIC:
        {
            Vector3f h = ImportanceSampleGGX(Vector2f(get_random_float(), get_random_float()), roughness, N);
            float D = D_GGX(h, N, roughness);
            float jacobian = 1.0f / (4.0f * outgoing_view.dot(h));
            return D * N.dot(h) * jacobian;
        }
        case SMOOTH_DIELECTRIC:
        {
            float eta = incoming_light.dot(N) > 0 ? 1.0f / ior : ior;
            float kr;
            fresnel(incoming_light, N, eta, kr);
            return isReflect ? kr : (1.0f - kr);
        }
        default:
            return 0.0f;
    }
}

Vector3f Material::eval(const Vector3f &incoming_light, const Vector3f &outgoing_view, const Vector3f &N, bool isReflect){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = N.dot(outgoing_view);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            } else
                return Vector3f::Zero();
        }
        case ROUGH_CONDUCTOR:
        {
            // skip reflection calculation if below surface
            if (incoming_light.dot(N) <= 0 || outgoing_view.dot(N) <= 0) return Vector3f::Zero();
            
            // half-vector h = microfacet orientation that reflect incoming light dir into outgoing view dir
            Vector3f h_sum = incoming_light + outgoing_view;
            if (h_sum.norm() < EPSILON) return Vector3f::Zero();
            Vector3f h = h_sum.normalized();
            
            float D = D_GGX(h, N, roughness);                                              // GGX Normal Distribution Function
            float G = G_SmithGGX(incoming_light, outgoing_view, N, roughness);             // Smith Geometry term for how much the surface is visible
            Vector3f F = FresnelSchlick(outgoing_view.dot(h), base_reflectance);   // Fresnel term for view dependent reflectivity
            float denom = 4.0f * N.dot(incoming_light) * N.dot(outgoing_view) + 1e-4f;
            
            return F * D * G / denom;
        }
        case SMOOTH_CONDUCTOR: 
        {
            // Perfect mirror: Only evaluate Fresnel. Smooth surface will not have microfacets, thus no self-shadows neither
            return FresnelSchlick(N.dot(outgoing_view), base_reflectance);
        }
        case ROUGH_DIELECTRIC: 
        {
            Vector3f h = (incoming_light + outgoing_view).normalized();
            float D = D_GGX(h, N, roughness);
            float G = G_SmithGGX(incoming_light, outgoing_view, N, roughness);
            float F;
            fresnel(incoming_light, N, ior, F);

            float denom = 4.0f * std::abs(N.dot(incoming_light)) * std::abs(N.dot(outgoing_view)) + 1e-4f;
            if (isReflect) {
                return Vector3f(F, F, F) * D * G / denom;
            } else {
                float eta = incoming_light.dot(N) > 0 ? 1.0f / ior : ior;
                float factor = (1.0f - F);
                return Vector3f(factor, factor, factor) * D * G * eta * eta / denom;
            }
        }
        case SMOOTH_DIELECTRIC:
        {
            float eta = incoming_light.dot(N) > 0 ? 1.0f / ior : ior;
            float kr;
            fresnel(incoming_light, N, eta, kr);
            return isReflect ? Vector3f(kr, kr, kr) : Vector3f(1.0f - kr, 1.0f - kr, 1.0f - kr);
        }
        default:
            return Vector3f::Zero(); // or throw an error
    }
}

#endif // RAYTRACING_MATERIAL_H
