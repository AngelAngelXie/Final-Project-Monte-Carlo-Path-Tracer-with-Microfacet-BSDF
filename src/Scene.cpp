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

// p: intersection coordinate of object
// N: normal of intersection point at object
// wo: direction from object intersection point to light
Vector3f Scene::calculate_direct_light(Vector3f p, Vector3f wo,
                                       Vector3f N) const {
    Intersection inter;
    float pdf;
    sampleLight(inter, pdf);
    auto p_light = inter.coords;
    auto n_light = inter.normal;
    auto emit = inter.emit;
    auto ws = (p_light - p).normalized();
    auto dist = (p_light - p).norm();
    Ray rlight(p, ws);
    inter = intersect(rlight);
    if (inter.happened && std::abs(inter.distance - dist) < EPSILON) {
        return emit.cwiseProduct(inter.m->eval(wo, ws, N, false)) *
               (ws.dot(N)) * ((-ws).dot(n_light)) / (dist * dist) / pdf;
    }
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
    auto N = inter.normal;
    auto m = inter.m;
    auto wo = -ray.direction;

    float pdf;
    Vector3f l_dir = Vector3f::Zero();
    for (int i = 0; i < 4; i++) {
        sampleLight(inter, pdf);
        auto p_light = inter.coords;
        auto n_light = inter.normal;
        auto emit = inter.emit;
        auto ws = (p_light - p).normalized();
        auto dist = (p_light - p).norm();
        Ray rlight(p, ws);
        inter = intersect(rlight);
        if (inter.happened && std::abs(inter.distance - dist) < EPSILON) {
            l_dir += emit.cwiseProduct(m->eval(wo, ws, n)) * (ws.dot(n)) *
                     ((-ws).dot(n_light)) / (dist * dist) / pdf / 4;
        }
    }
}
