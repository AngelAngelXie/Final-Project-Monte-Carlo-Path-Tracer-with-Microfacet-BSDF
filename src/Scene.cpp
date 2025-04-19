//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
    // TO DO Implement Path Tracing Algorithm here
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

    float pdf;
    sampleLight(inter, pdf);
    auto p_light = inter.coords;
    auto n_light = inter.normal;
    auto emit = inter.emit;
    auto ws = (p_light - p).normalized();
    auto dist = (p_light - p).norm();
    Ray rlight(p, ws);
    inter = intersect(rlight);
    Vector3f l_dir = Vector3f::Zero();
    if (inter.happened && std::abs(inter.distance - dist) < EPSILON) {
        l_dir = emit.cwiseProduct(m->eval(wo, ws, n)) * (ws.dot(n)) *
                ((-ws).dot(n_light)) / (dist * dist) / pdf;
    }

    Vector3f l_ind = Vector3f::Zero();
    float rr = get_random_float();
    if (rr < RussianRoulette) {
        auto wi = m->sample(wo, n);
        Ray r(p, wi);
        inter = intersect(r);
        if (inter.happened && !inter.obj->hasEmit()) {
            l_ind = castRay(r, depth + 1).cwiseProduct(m->eval(wo, wi, n)) *
                    wi.dot(n) / m->pdf(wo, wi, n) * (1. / RussianRoulette);
        }
    }
    return l_dir + l_ind;
}
