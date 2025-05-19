//
// Created by LEI XU on 5/13/19.
//

#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H

#include "Bounds3.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include <Eigen/Dense>
using namespace Eigen;

class Sphere : public Object {
  public:
    Vector3f center;
    float radius, radius2;
    Material *m;
    float area;
    Sphere(const Vector3f &c, const float &r, Material *mt = new Material())
        : center(c), radius(r), radius2(r * r), m(mt), area(4 * M_PI * r * r) {}
    bool intersect(const Ray &ray) override;
    bool intersect(const Ray &ray, float &tnear,
                   uint32_t &index) const override;

    Intersection getIntersection(Ray ray) override {
        Intersection result;
        result.happened = false;
        Vector3f L = ray.origin - center;
        float a = ray.direction.dot(ray.direction);
        float b = 2 * ray.direction.dot(L);
        float c = L.dot(L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return result;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return result;
        result.happened = true;

        result.coords = Vector3f(ray.origin + ray.direction * t0);
        result.normal = (result.coords - center).normalized();
        result.m = this->m;
        result.obj = this;
        result.distance = t0;
        return result;
    }
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const override {
        N = (P - center).normalized();
    }

    Vector3f evalDiffuseColor(const Vector2f &st) const override {
        return m->getColor();
    }
    Bounds3 getBounds() override {
        return Bounds3(Vector3f(center.x() - radius, center.y() - radius,
                                center.z() - radius),
                       Vector3f(center.x() + radius, center.y() + radius,
                                center.z() + radius));
    }
    void Sample(Intersection &pos, float &pdf) override {
        float theta = 2.0 * M_PI * get_random_float(),
              phi = M_PI * get_random_float();
        Vector3f dir(std::cos(phi), std::sin(phi) * std::cos(theta),
                     std::sin(phi) * std::sin(theta));
        pos.coords = center + radius * dir;
        pos.normal = dir;
        pos.obj = this;
        pos.m = m;
        pdf = 1.0f / area;
    }
    float getArea() override { return area; }
    bool hasEmit() override { return m->hasEmission(); }
};
inline bool Sphere::intersect(const Ray &ray) { return true; }
inline bool Sphere::intersect(const Ray &ray, float &tnear,
                              uint32_t &index) const {
    return false;
}

#endif // RAYTRACING_SPHERE_H
