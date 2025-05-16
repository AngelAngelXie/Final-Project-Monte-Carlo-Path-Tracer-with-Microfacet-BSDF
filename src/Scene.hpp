//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Camera.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Intersection.hpp"
#include "Light.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "WaveLen.hpp"
#include <Eigen/Dense>
#include <vector>

class Scene {
    float rrRate = 0.7;
    float invRr = 1 / .7;
    bool enable_shadow = true;
    int n_dir_sample = 4;

  public:
    // setting up options
    Camera camera;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);

    Scene(Camera camera) : camera(camera) {}

    void Add(Object *object) {
        objects.push_back(object);
        if (object->hasEmit()) {
            lightsObjects.push_back(object);
        }
    }
    void setRrRate(float rr) {
        rrRate = std::min(rr, 0.99f);
        invRr = 1 / rrRate;
    }
    void setDirectLightSample(int x) {
        n_dir_sample = x;
    }
    void enableShadow(bool shadow) {
        enable_shadow = shadow;
    }
    void Add(std::unique_ptr<Light> light) {
        lights.push_back(std::move(light));
    }

    const std::vector<Object *> &get_objects() const { return objects; }
    const std::vector<std::unique_ptr<Light>> &get_lights() const {
        return lights;
    }
    Intersection intersect(const Ray &ray) const;
    BVHAccel *bvh;
    void buildBVH();
    float castRay(const Ray &ray, int depth, const WaveLenType &wavelen) const;
    void sampleLight(Intersection &pos, float &pdf) const;
    bool trace(const Ray &ray, const std::vector<Object *> &objects,
               float &tNear, uint32_t &index, Object **hitObject);
    std::tuple<Vector3f, Vector3f>
    HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint,
                    const Vector3f &N, const Vector3f &shadowPointOrig,
                    const std::vector<Object *> &objects, uint32_t &index,
                    const Vector3f &dir, float specularExponent);
    Vector3f calculate_direct_light(Vector3f p, Vector3f wo, Vector3f N) const;

    float directLighting(const Vector3f &wo, const Intersection &surf_inter,
                         const WaveLenType &wavelen,
                         bool isReflect = true) const;

    // creating the scene (adding objects and lights)
    std::vector<Object *> objects;
    std::vector<Object *> lightsObjects;
    std::vector<std::unique_ptr<Light>> lights;
};
