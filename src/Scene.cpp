//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Intersection.hpp"
#include "Material.hpp"
#include "WaveLen.hpp"
#include "global.hpp"
#include <Eigen/Dense>
int test_temp=0;

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

float Scene::directLighting(const Vector3f &wo, const Intersection &surf_inter,
                            const WaveLenType &wavelen, bool isReflect) const {
    Material *m = surf_inter.m;
    Vector3f p = surf_inter.coords;
    Vector3f n = surf_inter.normal;
    Vector2f uv = surf_inter.tcoords;
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
            l_dir += emit * m->eval(ws, wo, n, wavelen, uv, isReflect) *
                     (ws.dot(n)) * (-ws).dot(n_light) / (dist * dist) / pdf /
                     n_dir_sample;
        }
    }
    return l_dir;
}

// Implementation of Path Tracing
float Scene::castRay(const Ray &ray, int depth,
                     const WaveLenType &wavelen) const {
    auto inter = intersect(ray);
    // std::cout<<"inside castray"<<std::endl;
    if (!inter.happened) {
        if (useEnvMap) {
            // std::cout<<"flag1, in useEnvMap"<<std::endl;
            // return extract(wavelen, sampleEnv(ray.direction));
            Vector3f L = sampleEnv(ray.direction);
            

                // std::cout<< "colors of useEnvMap, sampleEnv result: index at " <<test_temp<<std::endl;
            // std::cout<< L.transpose()<<std::endl;
                // std::cout<<"extracted: "<<extract(wavelen, L)<<std::endl;

            if(extract(wavelen, L)<0){
                std::cout<<"extracted: "<<extract(wavelen, L)<<std::endl;
            }
            return extract(wavelen, L);
            
        }else{
            if(test_temp++<6){
                // std::cout<<"extracted: "<<extract(wavelen, this->backgroundColor)<<std::endl;
            }
            return extract(wavelen, this->backgroundColor);

        }
    }
    auto p = inter.coords;
    auto n = inter.normal;
    auto m = inter.m;
    auto uv = inter.tcoords;
    Vector3f wo = -ray.direction;

    if (depth == 0 && inter.obj->hasEmit()) {
        float dist = (p - ray.origin).norm();
        return clamp(0, 1,
                     extract(wavelen, inter.m->getEmission()) *
                         std::abs(wo.dot(n)));
    }

    auto mfn = m->sample(wo, n); //  microfacet normal
    float kr = m->fresnel(ray.direction, mfn, wavelen);

    float l_dir = 0, l_ind = 0;
    float rr = get_random_float();
    float rd_flect = get_random_float();
    if (rd_flect < kr) {
        if (wo.dot(mfn) < 0) { //  inner reflection
            p -= n * EPSILON;
        } else { //  only calc direct lighting when ray is outside
            p += n * EPSILON;
            inter.coords += n * EPSILON;
            l_dir = directLighting(wo, inter, wavelen, true);
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
                        m->eval(wi, wo, n, wavelen, uv, true) * invRr;
            } else {
                l_ind = castRay(r, depth + 1, wavelen) *
                        (m->eval(wi, wo, n, wavelen, uv, true)) *
                        std::abs(wo.dot(n)) / m->pdf(wi, wo, n, wavelen, true) *
                        invRr;
            }
        }
    } else {
        if (wo.dot(mfn) < 0) { //  in-out refraction
            p += n * EPSILON;
            inter.coords += n * EPSILON;
            l_dir = directLighting(wo, inter, wavelen, false);
        } else {
            p -= n * EPSILON;
        }
        if (rr >= this->rrRate) {
            return l_dir;
        }
        auto wi = m->refract(ray.direction, mfn, wavelen);
        Ray r(p, wi);
        inter = intersect(r);
        if (inter.happened && !inter.obj->hasEmit()) {
            if (m->isDirac) {
                l_ind = castRay(r, depth + 1, wavelen) *
                        m->eval(wi, wo, n, wavelen, uv, false) * invRr;
            } else {
                l_ind = castRay(r, depth + 1, wavelen) *
                        m->eval(wi, wo, n, wavelen, uv, false) *
                        std::abs(wo.dot(n)) /
                        m->pdf(wi, wo, n, wavelen, false) * invRr;
            }
        }
    }

    //  use clamp to avoid fireflies
    float threshold_ind = 5;
    l_ind = clamp(0, threshold_ind, l_ind);
    return l_dir + l_ind;
}
