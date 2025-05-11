//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "global.hpp"
#include <Eigen/Dense>
using namespace Eigen;

enum MaterialType {
    DIFFUSE,
    SMOOTH_CONDUCTOR,
    ROUGH_CONDUCTOR,
    SMOOTH_DIELECTRIC,
    ROUGH_DIELECTRIC
};

class Material {
  private:
    // ==============================
    // === Start of GGX Functions ===

    // GGX Normal Distribution Function (NDF)
    inline float D_GGX(const Vector3f &h, const Vector3f &n, float alpha) {
        float NoH = std::abs(n.dot(h));
        float alpha2 = alpha * alpha;
        float denom = (NoH * NoH) * (alpha2 - 1.0f) + 1.0f;
        return alpha2 / (M_PI * denom * denom);
    }
    // Smith Geometry Term for GGX
    // Used to account for self-shadowing & masking in incoming & outgoing
    // vectors
    inline float G_SmithGGX_Helper(const Vector3f &v, const Vector3f &n,
                                   float alpha) {
        // cos of angle between surface normal & direction vector
        float NoV = std::abs(n.dot(v));
        if (NoV <= EPSILON && NoV >= -EPSILON)
            return 0.0f;
        // tan of the angle between the direction vector & surface normal
        float tanTheta = std::sqrt(1.0f - NoV * NoV) / NoV;

        // when tanTheta is 0, the angle between direction vector & surface
        // normal is perpendicular --> no masking/shadowing --> NO ATTENUATION
        if (tanTheta == 0.0f)
            return 1.0f;

        // Alpha: How rough the surface is. Rougher surface = more
        // masking/shadowing tanTheta: how grazing the view angle is. Large
        // tanTheta = more masking/shadowing
        float a = 1.0f / (alpha * tanTheta);
        if (a >= 1.6f)
            return 1.0f;
        float a2 = a * a;

        // schlick
        // grazing angle & roughness increase -> more masking -> returns a
        // smaller G term
        return (3.535f * a + 2.181f * a2) / (1.0f + 2.276f * a + 2.577f * a2);
    }
    inline float G_SmithGGX(const Vector3f &incoming_dir,
                            const Vector3f &outgoing_dir, const Vector3f &n,
                            float alpha) {
        // both incoming & outgoing direction can experience masking/shadowing
        // thus, overall geometry term G is the product of the two
        return G_SmithGGX_Helper(incoming_dir, n, alpha) *
               G_SmithGGX_Helper(outgoing_dir, n, alpha);
    }
    // Fresnel term using Schlick's approximation
    // used for metals, not good for refraction
    inline Vector3f FresnelSchlick(float cosTheta, const Vector3f &F0) {
        return F0 + (Vector3f::Ones() - F0) * powf(1.0f - cosTheta, 5.0f);
    }
    // Transform vector to world coordinates from tangent space
    // When sampling from GGX distribution, work is done in tangent space.
    // z-axis = surface normal n
    // x-axis = tanget vector T
    // y-axis = bitangent vector B
    // rendering algo works in world space.
    // Thus, sample from GGX distribution, then convert to world space before
    // use!
    inline Vector3f tanToWorld(const Vector3f &tan_coords, const Vector3f &n) {
        Vector3f B, T;
        if (std::fabs(n.x()) > std::fabs(n.y())) {
            float invLen = 1.0f / std::sqrt(n.x() * n.x() + n.z() * n.z());
            T = Vector3f(-n.z() * invLen, 0.0f, n.x() * invLen);
        } else {
            float invLen = 1.0f / std::sqrt(n.y() * n.y() + n.z() * n.z());
            T = Vector3f(0.0f, n.z() * invLen, -n.y() * invLen);
        }
        B = n.cross(T);
        return tan_coords.x() * T + tan_coords.y() * B + tan_coords.z() * n;
    }
    // Importance sample GGX NDF
    // Given a 2D uniform random sample Xi...
    // chooses reflection directions that are likely to contribute significant
    // energy based on statistical surface roughness
    inline Vector3f ImportanceSampleGGX(const Vector2f &Xi, float alpha,
                                        const Vector3f &n) {
        // sample random microfacet normal h from GGX NDF in tangent space
        float phi = 2.0f * M_PI * Xi.x();
        float cosTheta = std::sqrt((1.0f - Xi.y()) /
                                   (1.0f + (alpha * alpha - 1.0f) * Xi.y()));
        float sinTheta = std::sqrt(1.0f - cosTheta * cosTheta);
        Vector3f tan_space_h(sinTheta * std::cos(phi), sinTheta * std::sin(phi),
                             cosTheta);

        // return the world space result
        return tanToWorld(tan_space_h, n).normalized();
    }

    // sample the microfacet normal h from GGX distribution
    inline Vector3f sampleGGXMicrofacetNormal(const Vector3f &incoming_light,
                                              const Vector3f &n, float alpha) {
        Vector2f Xi(get_random_float(), get_random_float());
        return ImportanceSampleGGX(Xi, alpha, n);
    }
    // === End of GGX Functions ===
    // ============================

    // ==============================
    // === Start of GGX Functions ===

    // GGX Normal Distribution Function (NDF)

  public:
    MaterialType m_type;
    Vector3f m_color;
    Vector3f m_emission;
    float ior;       // index of refraction
    Vector3f Kd, Ks; // coefficient of diffuse and specular
    float specularExponent;
    bool textured;
    bool isDirac; //  Is pdf a dirac delta function
    float roughness;
    Vector3f base_reflectance;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &incoming_light, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &incoming_light,
                     const Vector3f &outgoing_view, const Vector3f &N);
    // given a ray direction and normal, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &incoming_light,
                         const Vector3f &outgoing_view, const Vector3f &N,
                         bool isReflect = false);
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const {
        return 2 * N.dot(I) * N - I;
    }
    float fresnel(const Vector3f &I, const Vector3f &N) const {
        if (this->m_type == SMOOTH_CONDUCTOR ||
            this->m_type == ROUGH_CONDUCTOR) {
            return 1;
        }
        float cosi = clamp(-1, 1, I.dot(N));
        float etai = 1, etat = ior;
        if (cosi > 0) {
            std::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            return 1;
        } else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) /
                       ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) /
                       ((etai * cosi) + (etat * cost));
            //  assume all lights are non-polarised
            return (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is
        // given by: kt = 1 - kr;
    }
    Vector3f refract(const Vector3f &I, const Vector3f &N) const {
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
};

Material::Material(MaterialType t, Vector3f e) {
    m_type = t;
    m_emission = e;
    isDirac = (t == SMOOTH_CONDUCTOR ||
               t == SMOOTH_DIELECTRIC); //  dirac delta pdf for smooth
    ior = 1.3;
    roughness = 1.0f;
    base_reflectance = Vector3f(0, 0, 0);
}

MaterialType Material::getType() { return m_type; }
Vector3f Material::getColor() { return m_color; }
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission() { return (m_emission.norm() > EPSILON); }

Vector3f Material::getColorAt(double u, double v) { return Vector3f(); }

// Generates a new ray direction for indirect lighting calculations, biased by
// the material’s properties.
Vector3f Material::sample(const Vector3f &incoming_light, const Vector3f &N) {
    switch (m_type) {
    case DIFFUSE: {
        // uniform sample on the hemisphere
        float x_1 = get_random_float(), x_2 = get_random_float();
        float z = std::fabs(1.0f - 2.0f * x_1);
        float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
        Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
        return tanToWorld(localRay, N);
    }
    case ROUGH_CONDUCTOR:
        return sampleGGXMicrofacetNormal(incoming_light, N, roughness);
    case SMOOTH_CONDUCTOR:
        return N;
    case ROUGH_DIELECTRIC:
        return sampleGGXMicrofacetNormal(incoming_light, N, roughness);
    case SMOOTH_DIELECTRIC:
        return N;
    default:
        return Vector3f();
    }
}

// Computes the probability density of a given ray direction being generated by
// sample.
float Material::pdf(const Vector3f &microfacet_normal,
                    const Vector3f &outgoing_view, const Vector3f &N) {
    switch (m_type) {
    case DIFFUSE: {
        if (outgoing_view.dot(N) > 0.0f)
            return 0.5f / M_PI;
        else
            return 0.0f;
    }
    case ROUGH_CONDUCTOR: {
        Vector3f h = microfacet_normal;

        float D = D_GGX(h, N, roughness);

        float cos_theta = std::abs(outgoing_view.dot(h));
        float jacobian = 1.0f / (4.0f * cos_theta);

        return D * N.dot(h) * jacobian;
    }
    case SMOOTH_CONDUCTOR:
        return (microfacet_normal.dot(N) > 1 - EPSILON) ? 1.0f : 0.0f;
    case ROUGH_DIELECTRIC: {
        Vector3f h = microfacet_normal;
        float D = D_GGX(h, N, roughness);
        float jacobian = 1.0f / (4.0f * outgoing_view.dot(h));
        return D * N.dot(h) * jacobian;
    }
    case SMOOTH_DIELECTRIC: {
        return (microfacet_normal.dot(N) > 1 - EPSILON) ? 1.0f : 0.0f;
    }
    default:
        return 0.;
    }
}

Vector3f Material::eval(const Vector3f &incoming_light,
                        const Vector3f &outgoing_view, const Vector3f &N,
                        bool isReflect) {
    switch (m_type) {
    case DIFFUSE: {
        float cosalpha = N.dot(incoming_light);
        if (cosalpha > 0.0f) {
            Vector3f diffuse = Kd / M_PI;
            return diffuse;
        } else
            return Vector3f::Zero();
    }
    case ROUGH_CONDUCTOR: {
        // skip reflection calculation if below surface
        if (incoming_light.dot(N) <= 0 || outgoing_view.dot(N) <= 0) {
            return Vector3f::Zero();
        }

        // half-vector h = microfacet orientation that reflect incoming
        // light dir into outgoing view dir
        Vector3f h_sum = incoming_light + outgoing_view;
        if (h_sum.norm() < EPSILON) {
            return Vector3f::Zero();
        }
        Vector3f h = h_sum.normalized();

        float D = D_GGX(h, N, roughness); // GGX Normal Distribution Function
        float G = G_SmithGGX(incoming_light, outgoing_view, h,
                             roughness); // Smith Geometry term for how
                                         // much the surface is visible
        Vector3f F = FresnelSchlick(outgoing_view.dot(h),
                                    base_reflectance); // Fresnel term for view
                                                       // dependent reflectivity
        float denom =
            4.0f * N.dot(incoming_light) * N.dot(outgoing_view) + EPSILON;

        return F * D * G / denom;
    }
    case SMOOTH_CONDUCTOR: {
        // Perfect mirror: Only evaluate Fresnel. Smooth surface will
        // not have microfacets, thus no self-shadows neither
        return FresnelSchlick(N.dot(outgoing_view), base_reflectance);
    }
    case ROUGH_DIELECTRIC: {
        if (isReflect) {
            Vector3f h = (incoming_light + outgoing_view).normalized();
            h = (h.dot(N) > 0) ? h : -h;
            float F = fresnel(-incoming_light, h);
            float D = D_GGX(h, N, roughness);
            float G = G_SmithGGX(incoming_light, outgoing_view, h, roughness);

            float denom = 4.0f * std::abs(N.dot(incoming_light)) *
                              std::abs(N.dot(outgoing_view)) +
                          EPSILON;
            return Vector3f::Constant(F) * D * G / denom;
        } else {
            float eta = (incoming_light.dot(N) > 0) ? ior : 1.0f / ior;
            Vector3f h = (incoming_light - outgoing_view * eta).normalized();
            h = (h.dot(N) > 0) ? h : -h;
            float F = fresnel(-incoming_light, h);
            float D = D_GGX(h, N, roughness);
            float G = G_SmithGGX(incoming_light, outgoing_view, h, roughness);
            float d1 = incoming_light.dot(h), d2 = outgoing_view.dot(h);
            float denom = d2 * d2 * eta * eta + d1 * d2 * eta + d1 * d1;
            return Vector3f::Constant(1.0f - F) * D * G * eta * eta / denom;
        }
    }
    case SMOOTH_DIELECTRIC: {
        float kr = fresnel(-incoming_light, N);
        return isReflect ? Vector3f::Constant(kr)
                         : Vector3f::Constant(1.0f - kr);
    }
    default:
        return Vector3f::Zero();
    }
}

#endif // RAYTRACING_MATERIAL_H
