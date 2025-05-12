//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Material.hpp"
#include "WaveLen.hpp"
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

float Scene::directLighting(const Vector3f &wo, const Vector3f &p,
                            const Vector3f &n, Material *m,
                            const WaveLenType &wavelen, bool isReflect) const {
    float l_dir = 0;
    float pdf;
    int n_dir_sample = 4;
    Intersection inter;
    for (int i = 0; i < n_dir_sample; i++) {
        sampleLight(inter, pdf);
        auto p_light = inter.coords;
        auto n_light = inter.normal;
        auto emit = extract(wavelen, inter.emit);
        auto ws = (p_light - p).normalized();
        auto dist = (p_light - p).norm();
        Ray rlight(p, ws);
        inter = intersect(rlight);
        if (inter.happened && std::abs(inter.distance - dist) < EPSILON) {
            l_dir += emit * m->eval(ws, wo, n, wavelen, isReflect) *
                     (ws.dot(n)) * ((-ws).dot(n_light)) / (dist * dist) / pdf /
                     n_dir_sample;
        }
    }
    return l_dir;
}

// Implementation of Path Tracing
float Scene::castRay(const Ray &ray, int depth,
                     const WaveLenType &wavelen) const {
    auto inter = intersect(ray);
    if (!inter.happened) {
        return extract(wavelen, this->backgroundColor);
    }
    auto p = inter.coords;
    auto n = inter.normal;
    auto m = inter.m;
    Vector3f wo = -ray.direction;
    float fwl = getWaveLen(wavelen);

    if (depth == 0 && inter.obj->hasEmit()) {
        return extract(wavelen, inter.emit);
    }

    auto mfn = m->sample(wo, n); //  microfacet normal
    float kr = m->fresnel(ray.direction, mfn, fwl);

    float l_dir = 0, l_ind = 0;
    float rr = get_random_float();
    float rd_flect = get_random_float();
    if (rd_flect < kr) {
        if (wo.dot(mfn) < 0) { //  inner reflection
            p -= n * EPSILON;
        } else { //  only calc direct lighting when ray is outside
            p += n * EPSILON;
            l_dir = directLighting(wo, p, n, m, wavelen, true);
        }
        if (rr >= this->rrRate) {
            return l_dir;
        }
        auto wi = m->reflect(wo, mfn);
        Ray r(p, wi);
        inter = intersect(r);
        if (inter.happened && !inter.obj->hasEmit()) {
            if (m->isDirac) {
                l_ind = castRay(r, depth + 1, wavelen) *
                        m->eval(wi, wo, n, wavelen, true) * invRr;
            } else {
                l_ind = castRay(r, depth + 1, wavelen) *
                        (m->eval(wi, wo, n, wavelen, true)) *
                        std::abs(wi.dot(n)) / m->pdf(mfn, wo, n) * invRr;
            }
        }
    } else {
        if (wo.dot(n) < 0) { //  in-out refraction
            p += n * EPSILON;
            l_dir = directLighting(wo, p, n, m, wavelen, false);
        } else {
            p -= n * EPSILON;
        }
        if (rr >= this->rrRate) {
            return l_dir;
        }
        auto wi = m->refract(ray.direction, mfn, fwl);
        Ray r(p, wi);
        inter = intersect(r);
        if (inter.happened && !inter.obj->hasEmit()) {
            if (m->isDirac) {
                l_ind = castRay(r, depth + 1, wavelen) *
                        m->eval(wi, wo, n, wavelen, false) * invRr;
            } else {
                l_ind = castRay(r, depth + 1, wavelen) *
                        m->eval(wi, wo, n, wavelen, false) *
                        std::abs(wi.dot(n)) / m->pdf(mfn, wo, n) * invRr;
            }
        }
    }

    //  use clamp to avoid fireflies
    float threshold_ind = 7;
    l_ind = clamp(0, threshold_ind, l_ind);
    return l_dir + l_ind;
}
