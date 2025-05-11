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

// p: intersection coordinate of object
// N: normal of intersection point at object
// wo: direction from object intersection point to light
Vector3f Scene::calculate_direct_light(Vector3f p, Vector3f wo, Vector3f N) {
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
        return emit.cwiseProduct(m->eval(wo, ws, N)) * (ws.dot(N)) *
                ((-ws).dot(n_light)) / (dist * dist) / pdf;
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

    if (inter.m->m_type == DIFFUSE) {
        
        Vector3f l_dir = calculate_direct_light(p, wo, N);
    
        Vector3f l_ind = Vector3f::Zero();
        float rr = get_random_float();
        if (rr < RussianRoulette) {
            auto wi = m->sample(wo, N);
            Ray r(p, wi);
            inter = intersect(r);
            if (inter.happened && !inter.obj->hasEmit()) {
                l_ind = castRay(r, depth + 1).cwiseProduct(m->eval(wo, wi, N)) *
                        wi.dot(N) / m->pdf(wo, wi, N) * (1. / RussianRoulette);
            }
        }
        return l_dir + l_ind;
    } else if (inter.m->m_type == ROUGHT_CONDUCTOR || inter.material->m_type == SMOOTH_CONDUCTOR) {
        Vector3f l_ind = Vector3f::Zero();
        // sample microfacet normal from GGX if needed
        Vector3f h = inter.material->sample(wo, N);
        Vector3f wi = normalize(reflect(-wo, h));
        if (dotProduct(N, wi) <= 0) return Vector3f(0);
         
        // Compute the BRDF value
        Vector3f brdf = inter.material->eval(wo, wi, N);
        // Compute the PDF of the sampled direction
        float pdf = inter.material->pdf(wo, wi, N);
        if (pdf < EPSILON) return Vector3f(0);
        
        // Recursively trace the ray
        Vector3f adjustedp = p + N * EPSILON;
        Vector3f Li = castRay(Ray(adjustedp, wi), depth + 1);
        
        if (inter.material->m_type == ROUGH_CONDUCTOR) {
            l_ind = brdf * Li * std::max(0.0f, dotProduct(N, wi)) / pdf;
        } else if (inter.material->m_type == SMOOTH_CONDUCTOR) {
            l_ind = brdf * Li;
        }

        Vector3f l_dir = calculate_direct_light(p, wo, N)

        return l_dir+l_ind;
    } else if (inter.material->m_type == ROUGH_DIELECTRIC || inter.material->m_type == SMOOTH_DIELECTRIC) {
        // sample microfacet normal from GGX if needed
        Vector3f h = inter.material->sample(wo, N);
        // reflectance computed using the Fresnel equations
        // reflection dominates at glazing angle
        float eta = dotProduct(wo, N) > 0 ? 1.0f / inter.material->ior : inter.material->ior;
        float kr = fresnel(wo, h, eta);
        bool isReflect = get_random_float() < kr;

        // perform either reflection or refraction -- standard for Dielectric in Path Tracers 
        if (isReflect) {
            wi = reflect(wo, h).normalized();
        } else {
            wi = refract(wo, h, inter.material->ior).normalized();
        }

        float pdf = inter.material->pdf(wo, wi, N, isReflect);

        if (pdf < EPSILON) return Vector3f(0);
        // recursively castRay
        Vector3f wi_origin = dotProduct(wi, N) < 0 
                                ? p - N * EPSILON 
                                : p + N * EPSILON;
        
        Vector3f Li = castRay(Ray(wi_origin, wi), depth + 1);
        // calculate the brdf value
        Vector3f brdf = inter.material->eval(wo, wi, N, isReflect);

        return brdf * Li * std::abs(dotProduct(N, wi)) / pdf;
    }
}
