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
    float kr = m->fresnel(ray.direction, n);

    float pdf;
    Vector3f l_dir = Vector3f::Zero();

    //  only calculate direct emit if the ray is not inside the object
    if (wo.dot(n) > 0) {
        p += n * EPSILON;
        int n_dir_sample = 4;
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
                l_dir += emit.cwiseProduct(m->eval(wo, ws, n)) * (ws.dot(n)) *
                         ((-ws).dot(n_light)) / (dist * dist) / pdf /
                         n_dir_sample * kr;
            }
        }
    }

    Vector3f l_ind = Vector3f::Zero();
    float rr = get_random_float();
    if (rr >= this->rrRate) {
        return l_dir;
    }
    float rd_flect = get_random_float();
    if (rd_flect < kr) {
        if (wo.dot(n) < 0) { //  inner reflection
            n = -n;
            p += n * EPSILON;
            auto wi = 2 * n.dot(wo) * n - wo;
            l_ind = castRay({p, wi}, depth + 1) * invRr;
        } else { //  use BRDF only if the ray is outside
            auto wi = m->sample(wo, n);
            Ray r(p, wi);
            inter = intersect(r);
            if (inter.happened && !inter.obj->hasEmit()) {
                l_ind = castRay(r, depth + 1).cwiseProduct(m->eval(wo, wi, n)) *
                        wi.dot(n) / m->pdf(wo, wi, n) * invRr;
            }
        }
    } else {
        auto dir_fract = m->refract(ray.direction, n);
        if (wo.dot(n) < 0) { //  in-out refraction
            p += n * EPSILON;
            //  use BRDF as BSDF by back refraction ray as reflection ray
            //  TODO: switch to real BSDF
            auto wo_fake = 2 * n.dot(dir_fract) * n - dir_fract;
            auto wi = m->sample(wo_fake, n);
            Ray r(p, wi);
            inter = intersect(r);
            if (inter.happened && !inter.obj->hasEmit()) {
                l_ind = castRay(r, depth + 1)
                            .cwiseProduct(m->eval(wo_fake, wi, n)) *
                        wi.dot(n) / m->pdf(wo_fake, wi, n) * invRr;
            }
        } else {
            p -= 2 * n * EPSILON;
            l_ind = castRay({p, dir_fract}, depth + 1) * invRr;
        }
    }
    return l_dir + l_ind;
}
