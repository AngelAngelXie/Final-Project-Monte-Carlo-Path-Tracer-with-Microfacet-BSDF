//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Scene.hpp"
#include "global.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <future>
#include <thread>

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene) {
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 8;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = Vector3f(-x, y, 1);
            const float DF = 0.3;
            /*    single thread
            auto rand_with_neg = []() { return get_random_float() - .5; };
            for (int k = 0; k < spp; k++) {
                Vector3f offset =
                    Vector3f(rand_with_neg(), rand_with_neg(), 0) * DF /
                    scene.height * scale;
                auto color = scene.castRay(Ray(eye_pos, dir + offset), 0);
                framebuffer[m] += color / spp;
            }
            */
            /*    multi-thread    */
            std::vector<std::thread> threads(spp);
            std::vector<std::future<Vector3f>> futures(spp);
            auto rand_with_neg = []() { return get_random_float() - .5; };
            for (int k = 0; k < spp; k++) {
                Vector3f offset =
                    Vector3f(rand_with_neg(), rand_with_neg(), 0) * DF /
                    scene.height * scale;
                std::packaged_task<Vector3f(Vector3f, Vector3f)> task(
                    [&](Vector3f ori, Vector3f dir) {
                        return scene.castRay(Ray(ori, dir), 0);
                    });
                futures[k] = task.get_future();
                threads[k] =
                    std::thread(std::move(task), eye_pos, (dir + offset));
            }
            for (int k = 0; k < spp; k++) {
                threads[k].join();
                framebuffer[m] += futures[k].get() / spp;
            }
            /**/
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
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
