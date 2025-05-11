//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Material.hpp"
#include <Eigen/Dense>

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
    float emit_area_sum = 0;
    for (auto lightObj : lightsObjects) {
        emit_area_sum += lightObj->getArea();
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (auto lightObj : lightsObjects) {
        emit_area_sum += lightObj->getArea();
        if (p <= emit_area_sum) {
            lightObj->Sample(pos, pdf);
            break;
        }
    }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) {
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

Vector3f Scene::directLighting(const Vector3f &wo, const Vector3f &p,
                               const Vector3f &n, Material *m) const {
    Vector3f l_dir = Vector3f::Zero();
    float pdf;
    int n_dir_sample = 4;
    Intersection inter;
    for (int i = 0; i < n_dir_sample; i++) {
        sampleLight(inter, pdf);
        auto p_light = inter.coords;
        auto n_light = inter.normal;
        auto emit = inter.emit;
        auto ws = (p_light - p).normalized();
        auto dist = (p_light - p).norm();
        Ray rlight(p, ws);
        inter = intersect(rlight);
        if (inter.happened && std::abs(inter.distance - dist) < EPSILON) {
            l_dir += emit.cwiseProduct(m->eval(ws, wo, n)) * (ws.dot(n)) *
                     ((-ws).dot(n_light)) / (dist * dist) / pdf / n_dir_sample;
        }
    }
    return l_dir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
    auto inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f::Zero();
    }
    if (depth == 0 && inter.obj->hasEmit()) {
        return inter.m->getEmission();
    }
    auto p = inter.coords;
    auto n = inter.normal;
    auto m = inter.m;
    auto wo = -ray.direction;

    auto mfn = m->sample(wo, n); //  microfacet normal
    float kr = m->fresnel(ray.direction, mfn);

    Vector3f l_dir = Vector3f::Zero();
    Vector3f l_ind = Vector3f::Zero();
    float rr = get_random_float();
    float rd_flect = get_random_float();
    if (rd_flect < kr) {
        if (wo.dot(mfn) < 0) { //  inner reflection
            p -= n * EPSILON;
            mfn = -mfn;
            n = -n;
        } else { //  only calc direct lighting when ray is outside
            p += n * EPSILON;
            l_dir = directLighting(wo, p, n, m);
        }
        if (rr >= this->rrRate) {
            return l_dir;
        }
        auto wi = m->reflect(wo, mfn);
        Ray r(p, wi);
        inter = intersect(r);
        if (inter.happened && !inter.obj->hasEmit()) {
            if (m->isDirac) {
                l_ind = castRay(r, depth + 1).cwiseProduct(m->eval(wo, wi, n)) *
                        invRr;
            } else {
                l_ind = castRay(r, depth + 1).cwiseProduct(m->eval(wi, wo, n)) *
                        wi.dot(n) / m->pdf(wi, wo, n) * invRr;
            }
        }
    } else {
        auto dir_fract = m->refract(ray.direction, mfn);
        if (wo.dot(n) < 0) { //  in-out refraction
            p += n * EPSILON;
            auto wo_fake = 2 * n.dot(dir_fract) * n - dir_fract;
            auto l_dir = directLighting(wo_fake, p, n, m);
            if (rr >= this->rrRate) {
                return l_dir;
            }
            //  use BRDF as BSDF by back refraction ray as reflection ray
            //  TODO: switch to real BSDF
            auto wi = m->sample(wo_fake, n);
            Ray r(p, wi);
            inter = intersect(r);
            if (inter.happened && !inter.obj->hasEmit()) {
                l_ind = castRay(r, depth + 1)
                            .cwiseProduct(m->eval(wo_fake, wi, n)) *
                        wi.dot(n) / m->pdf(wo_fake, wi, n) * invRr;
            }
        } else {
            if (rr >= this->rrRate) {
                return {0, 0, 0};
            }
            p -= n * EPSILON;
            l_ind = castRay({p, dir_fract}, depth + 1) * invRr;
        }
    }

    //  use clamp to avoid fireflies
    float threshold_ind = 5;
    l_ind = {
        clamp(0, threshold_ind, l_ind.x()),
        clamp(0, threshold_ind, l_ind.y()),
        clamp(0, threshold_ind, l_ind.z()),
    };
    return l_dir + l_ind;
}
