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

  public:
    // setting up options
    Camera camera;
    // Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    // old solid‑color fallback
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);

    //environment map
    bool        useEnvMap   = false;
    unsigned    envWidth    = 0, envHeight = 0;
    std::vector<Vector3f> envPixels;

    // load a PNG/HDR map from disk
    void loadEnvMap(const std::string &path) {
        // std::cout<<"flag4 entered load env"<<std::endl;
        std::vector<unsigned char> png; 
        unsigned error = lodepng::decode(png, envWidth, envHeight, path);
        if (error) {
            std::cerr << "Error loading env map (" << path 
                      << "): " << lodepng_error_text(error) << std::endl;
            return;
        }
        std::cout<<"flag5 decode: width height: "<<envWidth<<" , "<<envHeight<<std::endl;
        // std::cout<<"flag6 len of png: "<<png.size()<<std::endl;
        for (unsigned i = 0; i < 4; ++i) {
            unsigned idx = 4 * i;
            // std::cout<<"color at i= "<<i<<" :"<<std::endl;
            // std::cout<< (float)png[idx + 0]<<std::endl;
            // std::cout<< (float)png[idx + 1]<<std::endl;
            // std::cout<< (float)png[idx + 2]<<std::endl;
        }
        //useEnvMap = true;
        envPixels.reserve(envWidth * envHeight);
        // png is RGBA, so stride = 4
        for (unsigned i = 0; i < envWidth * envHeight; ++i) {
            unsigned idx = 4 * i;
            envPixels.push_back({
                png[idx + 0] / 255.0f,
                png[idx + 1] / 255.0f,
                png[idx + 2] / 255.0f
            });
        }
    }

    // spherical sampling remains unchanged
    // Vector3f sampleEnv(const Vector3f &dir) const {
    //     // float theta = std::acos(clamp(dir.y(), -1.0f, 1.0f));
    //     // float phi   = std::atan2(dir.z(), dir.x());
    //     // float u     = (phi + M_PI) / (2 * M_PI);
    //     // float v     = theta / M_PI;
    //     // unsigned x = std::min(envWidth - 1u, unsigned(u * envWidth));
    //     // unsigned y = std::min(envHeight - 1u,
    //     //                       unsigned((1 - v) * envHeight));
    //     // return envPixels[y * envWidth + x];

        
    // }

    // Vector3f sampleEnv(const Vector3f &dir) const {
    //     // std::cout<<"useEnvFlag: "<< useEnvMap<<std::endl;
    //     // 1. Normalize
    //     Vector3f d = dir.normalized();
    
    //     // 2. Spherical coords
    //     float phi   = std::atan2(d.z(), d.x());
    //     float theta = std::acos(d.y());
    
    //     // 3. UV in [0,1]
    //     float u = (phi + M_PI) * 1/(2*M_PI);   // define INV_TWO_PI = 1/(2π)
    //     float v = theta * 1/(M_PI);             // define INV_PI     = 1/π
    
    //     // 4. Map to image space
    //     float x = u * envWidth  - 0.5f;
    //     float y = v * envHeight - 0.5f;
    //     int   x0 = (int)std::floor(x);
    //     int   y0 = (int)std::floor(y);
    //     int   x1 = (x0 + 1) % envWidth;
    //     int   y1 = std::min(y0 + 1, (int)envHeight - 1);
    
    //     float sx = x - x0;
    //     float sy = y - y0;
    
    //     auto at = [&](int ix, int iy) {
    //         return envPixels[iy * envWidth + ix];
    //     };
    
    //     // 5. Optional bilinear interpolation
    //     Vector3f c00 = at((x0 + envWidth) % envWidth, std::max(0,y0));
    //     Vector3f c10 = at(x1, std::max(0,y0));
    //     Vector3f c01 = at((x0 + envWidth) % envWidth, y1);
    //     Vector3f c11 = at(x1, y1);
    
    //     Vector3f c0 = c00 * (1 - sx) + c10 * sx;
    //     Vector3f c1 = c01 * (1 - sx) + c11 * sx;
    //     return c0 * (1 - sy) + c1 * sy;
    // }
    
    Vector3f sampleEnv(const Vector3f &dir) const {
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



