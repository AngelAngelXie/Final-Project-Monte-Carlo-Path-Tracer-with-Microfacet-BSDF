//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Scene.hpp"
#include "global.hpp"
#include "lodepng.h"
#include <Eigen/Dense>
#include <omp.h>

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 1e-4;
const int PARALLELISM = 8;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene) {
    Camera camera = scene.camera;
    std::vector<Vector3f> framebuffer(camera.width * camera.height, {0, 0, 0});

    float scale = tan(deg2rad(camera.fov * 0.5));
    float imageAspectRatio = camera.width / (float)camera.height;
    // Vector3f eye_pos(278, 273, -800);
    Vector3f eye_pos = camera.position;
    Matrix3f orientation = camera.getOrientation();

    // change the spp value to change sample ammount
    int spp = this->spp;
    std::cout << "SPP: " << spp << "\n";
    float prog = 0.;
    init_rngs(PARALLELISM);
#pragma omp parallel for num_threads(PARALLELISM) schedule(dynamic, 8)
    for (int m = 0; m < camera.height * camera.width; ++m) {
        // generate primary ray direction
        int i = m % camera.width, j = m / camera.width;
        for (int k = 0; k < spp; k++) {
            Vector3f pos;
            Vector3f dir;

            if (camera.useDOF) {
                // 1. compute focal point for current screen pixel
                float x =
                    (1 - 2 * (i + get_random_float()) / (float)camera.width) *
                    imageAspectRatio * scale;
                float y =
                    (1 - 2 * (j + get_random_float()) / (float)camera.height) *
                    scale;
                Vector3f focal_point =
                    Vector3f(x, y, 1) * camera.focal_distance;

                // 2. Sample point on aperture (disk in x-y plane)
                float r =
                    camera.aperture_radius * std::sqrt(get_random_float());
                float theta = 2 * M_PI * get_random_float();
                float dx = r * std::cos(theta);
                float dy = r * std::sin(theta);
                pos = eye_pos + orientation * Vector3f(dx, dy, 0);

                // 3. New direction from aperture point to focal point
                dir = (focal_point - Vector3f(dx, dy, 0)).normalized();
            } else {
                float x =
                    (1 - 2 * (i + get_random_float()) / (float)camera.width) *
                    imageAspectRatio * scale;
                float y =
                    (1 - 2 * (j + get_random_float()) / (float)camera.height) *
                    scale;
                dir = Vector3f(x, y, 1).normalized();
                pos = eye_pos;
            }

            dir = orientation * dir;
            float colorR = scene.castRay(Ray(pos, dir), 0, RED);
            float colorG = scene.castRay(Ray(pos, dir), 0, GREEN);
            float colorB = scene.castRay(Ray(pos, dir), 0, BLUE);
            framebuffer[m] += Vector3f(colorR, colorG, colorB) / spp;
            // std::cout<<"printing framebuffer at: "<<m<<std::endl;
            // for(int temppy=0;temppy<3;temppy++){
            //     std::cout<<framebuffer[m][temppy]<<std::endl;
            // }
        }
        prog += 1.f / camera.height / camera.width;
        if (i == 0) {
            UpdateProgress(prog);
        }
    }
    UpdateProgress(1.f);

    std::cout << std::endl;
    std::cout << "Writing image to " << path << std::endl;
    std::vector<unsigned char> raw(4 * camera.width * camera.height);
    float inv_gamma = 0.45;
    for (int i = 0; i < camera.width * camera.height; i++) {
        int ir = i << 2;
        raw[ir] = clamp(0, 255, 255 * pow(framebuffer[i].x(), inv_gamma));
        raw[ir + 1] = clamp(0, 255, 255 * pow(framebuffer[i].y(), inv_gamma));
        raw[ir + 2] = clamp(0, 255, 255 * pow(framebuffer[i].z(), inv_gamma));
        raw[ir + 3] = 255;
    }
    unsigned err =
        lodepng::encode(path.c_str(), raw, camera.width, camera.height);
    if (err) {
        std::cerr << "Error when writing image : " << lodepng_error_text(err)
                  << std::endl;
    }
}
