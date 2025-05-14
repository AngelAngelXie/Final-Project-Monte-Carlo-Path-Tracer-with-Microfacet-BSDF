#include "Material.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "json.hpp"
#include <chrono>

using namespace nlohmann;

bool is_v3(const json &d) {
    return d.is_array() && d.size() == 3 &&
           std::all_of(d.begin(), d.end(),
                       [](const auto &e) { return e.is_number(); });
}

int main(int argc, char **argv) {

    Camera camera;
    Scene scene(camera);
    Renderer r;

    Material *red = new Material(ROUGH_CONDUCTOR, Vector3f::Zero());
    red->base_reflectance = Vector3f(0.63f, 0.065f, 0.05f);
    Material *green = new Material(SMOOTH_CONDUCTOR, Vector3f::Zero());
    green->base_reflectance = Vector3f(0.14f, 0.45f, 0.091f);
    Material *blue = new Material(ROUGH_CONDUCTOR, Vector3f::Zero());
    blue->base_reflectance = Vector3f(0.14f, 0.091f, .45f);
    Material *white = new Material(ROUGH_CONDUCTOR, Vector3f::Zero());
    white->base_reflectance = Vector3f(0.725f, 0.71f, 0.68f);
    Material *white_plas = new Material(ROUGH_DIELECTRIC, Vector3f::Zero());
    white_plas->base_reflectance = Vector3f(0.725f, 0.71f, 0.68f);
    Material *white_glas = new Material(SMOOTH_DIELECTRIC, Vector3f::Zero());
    Material *light = new Material(
        ROUGH_CONDUCTOR,
        (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
         15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
         18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));

    int w = 384, h = 384;
    Vector3f camPos(278, 273, -800);
    Vector3f camTarget(278, 273, 0);
    Vector3f camUp(0, 1, 0);

#ifdef DEBUG
    std::string root = std::filesystem::current_path().string();
    MeshTriangle floor(root + "/models/cornellbox/floor.obj", white);
    MeshTriangle shortbox(root + "/models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox(root + "/models/cornellbox/tallbox.obj", white);
    MeshTriangle left(root + "/models/cornellbox/left.obj", red);
    MeshTriangle right(root + "/models/cornellbox/right.obj", green);
    MeshTriangle light_(root + "/models/cornellbox/light.obj", light);

    std::ifstream confJson(root + "/build/conf.json");
#else
    MeshTriangle floor("../models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white_plas);
    MeshTriangle left("../models/cornellbox/left.obj", red);
    MeshTriangle right("../models/cornellbox/right.obj", green);
    MeshTriangle light_("../models/cornellbox/light.obj", light);

    std::ifstream confJson("conf.json");
#endif

    //  Reading configuration file
    try {
        auto data = json::parse(confJson);

        auto confCam = data["camera"];
        if (!confCam.is_null()) {
            if (confCam["width"].is_number()) {
                w = confCam["width"];
            }
            if (confCam["height"].is_number()) {
                h = confCam["height"];
            }
            if (confCam["fov"].is_number()) {
                camera.fov = confCam["fov"];
            }
            if (is_v3(confCam["position"])) {
                camPos =
                    Vector3f(confCam["position"][0], confCam["position"][1],
                             confCam["position"][2]);
            }
            if (is_v3(confCam["target"])) {
                camTarget = Vector3f(confCam["target"][0], confCam["target"][1],
                                     confCam["target"][2]);
            }
            if (is_v3(confCam["up"])) {
                camUp = Vector3f(confCam["up"][0], confCam["up"][1],
                                 confCam["up"][2]);
            }
            if (confCam["useDOF"].is_boolean()) {
                camera.useDOF = confCam["useDOF"];
            }
            if (camera.useDOF && confCam["focalDistance"].is_number()) {
                camera.focal_distance = confCam["focalDistance"];
            }
            if (camera.useDOF && confCam["apertureRadius"].is_number()) {
                camera.aperture_radius = confCam["apertureRadius"];
            }
        }

        auto confRenderer = data["renderer"];
        if (!confRenderer.is_null()) {
            if (confRenderer["spp"].is_number()) {
                r.setSpp(confRenderer["spp"]);
            }
            if (confRenderer["output"].is_string()) {
                r.path = confRenderer["output"];
            }
        }

        auto confScene = data["scene"];
        if (!confScene.is_null()) {
            if (confScene["RussianRouletteRate"].is_number()) {
                scene.setRrRate(confScene["RussianRouletteRate"]);
            }
            if (is_v3(confScene["backgroundColor"])) {
                scene.backgroundColor =
                    Vector3f(confScene["backgroundColor"][0],
                             confScene["backgroundColor"][1],
                             confScene["backgroundColor"][2]);
            }
        }

    } catch (const std::exception &e) {
        std::cerr << "Error when reading json config: " << e.what()
                  << std::endl;
    }

    camera.width = w;
    camera.height = h;
    camera.position = camPos;
    camera.lookAt(camTarget, camUp);
    // Change the definition here to change resolution
    scene.camera = camera;

    //  Scene building
    Sphere sphere({400, 90, 130}, 70, white_glas);

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&sphere);

    /*
    Material *m_prism = new Material(ROUGH_DIELECTRIC, Vector3f::Zero());
    m_prism->roughness = 0.04f;
    MeshTriangle *prism =
        new MeshTriangle("../models/prism/prism.obj", white_glas);
    Material *m_floor = new Material(ROUGH_CONDUCTOR, Vector3f::Zero());
    m_floor->base_reflectance = Vector3f(0.7f, 0.7f, 0.7f);
    m_floor->roughness = 3;
    MeshTriangle *floor_pr =
        new MeshTriangle("../models/prism/floor.obj", white);
    Material *m_light = new Material(ROUGH_CONDUCTOR, Vector3f::Constant(10));
    MeshTriangle *light_pr =
        new MeshTriangle("../models/prism/light.obj", m_light);
    MeshTriangle *wall_pr = new MeshTriangle("../models/prism/wall.obj", white);
    scene.Add(floor_pr);
    scene.Add(prism);
    scene.Add(light_pr);
    scene.Add(wall_pr);
    */

    scene.buildBVH();

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    using Hour = std::chrono::hours;
    using Min = std::chrono::minutes;
    using Sec = std::chrono::seconds;
    using Milli = std::chrono::milliseconds;
    auto dur = std::chrono::duration_cast<Milli>(stop - start);
    auto hours = std::chrono::floor<Hour>(dur);
    auto mins = std::chrono::floor<Min>(dur - hours);
    auto secs = std::chrono::floor<Sec>(dur - hours - mins);
    auto millis = std::chrono::duration_cast<Milli>(dur - hours - mins - secs);

    std::cout << "Rendering finished in " << hours.count() << ":"
              << mins.count() << ":" << secs.count() << "." << millis.count()
              << std::endl;

    return 0;
}
