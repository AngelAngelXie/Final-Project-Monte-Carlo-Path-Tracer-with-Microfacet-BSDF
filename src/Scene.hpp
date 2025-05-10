//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Camera.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Light.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include <Eigen/Dense>
#include <vector>

class Scene {
  public:
    // setting up options
    Camera camera;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 1;
    float rrRate = 0.7;
    float invRr = 1 / .7;

    Scene(Camera camera) : camera(camera) {}

    void Add(Object *object) {
        objects.push_back(object);
        if (object->hasEmit()) {
            lightsObjects.push_back(object);
        }
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
    Vector3f castRay(const Ray &ray, int depth) const;
    void sampleLight(Intersection &pos, float &pdf) const;
    bool trace(const Ray &ray, const std::vector<Object *> &objects,
               float &tNear, uint32_t &index, Object **hitObject);
    std::tuple<Vector3f, Vector3f>
    HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint,
                    const Vector3f &N, const Vector3f &shadowPointOrig,
                    const std::vector<Object *> &objects, uint32_t &index,
                    const Vector3f &dir, float specularExponent);
    Vector3f calculate_direct_light(Vector3f p, Vector3f wo, Vector3f N) const;

    // creating the scene (adding objects and lights)
    std::vector<Object *> objects;
    std::vector<Object *> lightsObjects;
    std::vector<std::unique_ptr<Light>> lights;
};
