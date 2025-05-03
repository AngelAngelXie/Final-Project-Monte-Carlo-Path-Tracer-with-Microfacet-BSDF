//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Scene.hpp"
#include "global.hpp"
#include <Eigen/Dense>
#include <omp.h>

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene) {
    Camera camera = scene.camera;
    std::vector<Vector3f> framebuffer(camera.width * camera.height);

    float scale = tan(deg2rad(camera.fov * 0.5));
    float imageAspectRatio = camera.width / (float)camera.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = this->spp;
    std::cout << "SPP: " << spp << "\n";
    float prog = 0.;
#pragma omp parallel for num_threads(this->parellelism)
    for (uint32_t j = 0; j < camera.height; ++j) {
        for (uint32_t i = 0; i < camera.width; ++i) {
            // generate primary ray direction
            int m = i + j * camera.width;
            for (int k = 0; k < spp; k++) {
                float x =
                    (1 - 2 * (i + get_random_float()) / (float)camera.width) *
                    imageAspectRatio * scale;
                float y =
                    (1 - 2 * (j + get_random_float()) / (float)camera.height) *
                    scale;

                Vector3f dir = Vector3f(x, y, 1).normalized();
                auto color = scene.castRay(Ray(eye_pos, dir), 0);
                framebuffer[m] += color / spp;
            }
        }
        prog += 1.f / camera.height;
        UpdateProgress(prog);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", camera.width, camera.height);
    for (auto i = 0; i < camera.height * camera.width; ++i) {
        static unsigned char color[3];
        color[0] =
            (unsigned char)(255 *
                            std::pow(clamp(0, 1, framebuffer[i].x()), 0.6f));
        color[1] =
            (unsigned char)(255 *
                            std::pow(clamp(0, 1, framebuffer[i].y()), 0.6f));
        color[2] =
            (unsigned char)(255 *
                            std::pow(clamp(0, 1, framebuffer[i].z()), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
