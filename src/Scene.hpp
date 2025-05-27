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
#include <string>
#include "lodepng.h" 



class Scene {
    float rrRate = 0.7;
    float invRr = 1 / .7;
    bool enable_shadow = true;
    int n_dir_sample = 4;

  public:
    // setting up options
    Camera camera;
    Vector3f backgroundColor = Vector3f(0, 0, 0);

    //environment map
    bool        useEnvMap   = false;
    unsigned    envWidth    = 0, envHeight = 0;
    std::vector<Vector3f> envPixels;
    void loadEnvMap(const std::string &path) {
        std::vector<unsigned char> png; 
        unsigned error = lodepng::decode(png, envWidth, envHeight, path);
        if (error) {
            std::cerr << "Error loading env map (" << path 
                      << "): " << lodepng_error_text(error) << std::endl;
            return;
        }
        useEnvMap = true;
        envPixels.reserve(envWidth * envHeight);
        for (unsigned i = 0; i < envWidth * envHeight; ++i) {
            unsigned idx = 4 * i;
            envPixels.push_back({
                png[idx + 0] / 255.0f,
                png[idx + 1] / 255.0f,
                png[idx + 2] / 255.0f
            });
        }
    }

    
    Vector3f sampleEnv(const Vector3f &dir) const {
        if(useEnvMap==false){
            return backgroundColor;
        }
        // 1. normalize and get spherical UV
        Vector3f d = dir.normalized();
        float phi   = std::atan2(d.z(), d.x());
        float theta = std::acos (d.y());
        float u     = (phi + M_PI) / (2.f*M_PI);
        float v     = theta      / M_PI;
        // fold & clamp
        u = u - std::floor(u);
        v = std::clamp(v, 0.f, 1.f);
    
        // 2. convert to image coords
        float x = u * envWidth  - 0.5f;
        float y = v * envHeight - 0.5f;
        int   x0 = (int)std::floor(x);
        int   y0 = (int)std::floor(y);
    
        // 3. safe wrap/clamp
        auto wrap = [&](int i, int N){
            int m = i % N;
            return m < 0 ? m + N : m;
        };
        int X0 = wrap(x0,      envWidth),  X1 = wrap(x0 + 1, envWidth);
        int Y0 = std::clamp(y0,      0, (int)envHeight-1),
            Y1 = std::clamp(y0 + 1, 0, (int)envHeight-1);
    
        float sx = x - x0, sy = y - y0;
        auto at  = [&](int ix, int iy){ return envPixels[iy*envWidth + ix]; };
    
        // 4. bilerp
        Vector3f c00 = at(X0, Y0), c10 = at(X1, Y0),
                 c01 = at(X0, Y1), c11 = at(X1, Y1);
        Vector3f c0  = c00*(1 - sx) + c10*sx;
        Vector3f c1  = c01*(1 - sx) + c11*sx;
    
        return c0*(1 - sy) + c1*sy;
    }
    

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



