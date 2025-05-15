#include "Material.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "global.hpp"
#include "json.hpp"
#include <chrono>
#include <filesystem>

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
    white_glas->base_reflectance = Vector3f(0.725f, 0.71f, 0.68f);
    Material *light = new Material(
        ROUGH_CONDUCTOR,
        (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
         15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
         18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    light->Kd = Vector3f(0.65f, .65, .65);

    int w = 384, h = 384;
    Vector3f camPos(278, 50, -1600);
    Vector3f camTarget(278, 273, 0);
    Vector3f camUp(0, 1, 0);

#ifdef DEBUG
    std::string root = std::filesystem::current_path().string();
    // MeshTriangle floor(root + "/models/cornellbox/floor.obj", white);
    // MeshTriangle shortbox(root + "/models/cornellbox/shortbox.obj", white);
    // MeshTriangle tallbox(root + "/models/cornellbox/tallbox.obj", white);
    // MeshTriangle left(root + "/models/cornellbox/left.obj", red);
    // MeshTriangle right(root + "/models/cornellbox/right.obj", green);
    // MeshTriangle light_(root + "/models/cornellbox/light.obj", light);
    MeshTriangle floor("../models/backwall.obj", green, Vector3f::Zero());
    //MeshTriangle left("../models/left.obj", red, Vector3f::Zero());
    //MeshTriangle right("../models/right.obj", green, Vector3f::Zero());
    MeshTriangle light_("../models/light.obj", light, Vector3f(0, 200, 0));
    MeshTriangle bottom("../models/bottom.obj", green, Vector3f::Zero());
    MeshTriangle soldier_1("../models/soldier_zoom_12_final.obj", white, Vector3f(-559, 0, -271));
    MeshTriangle soldier_2("../models/soldier_zoom_12_final.obj", white, Vector3f(160, 0, -271));
    MeshTriangle soldier_3("../models/soldier_zoom_12_final.obj", white, Vector3f(160, 0, -50));
    MeshTriangle soldier_4("../models/soldier_zoom_12_final.obj", white, Vector3f(-546, 0, -50));
    //MeshTriangle soldier_5("../models/soldier_zoom_12.obj", white, Vector3f(-90, -62.5, 320));
    //MeshTriangle soldier_6("../models/soldier_zoom_12.obj", white, Vector3f(-90, -62.5, 280));
    MeshTriangle king("../models/king_zoom_12_final.obj", white, Vector3f::Zero());
    //MeshTriangle floor("../models/pattern_quad.obj", green, Vector3f::Zero(), 1.0f, true);

    std::ifstream confJson(root + "/build/conf.json");
#else
    std::string root = std::filesystem::current_path().string();
    // MeshTriangle floor(root + "/models/cornellbox/floor.obj", white);
    // MeshTriangle shortbox(root + "/models/cornellbox/shortbox.obj", white);
    // MeshTriangle tallbox(root + "/models/cornellbox/tallbox.obj", white);
    // MeshTriangle left(root + "/models/cornellbox/left.obj", red);
    // MeshTriangle right(root + "/models/cornellbox/right.obj", green);
    // MeshTriangle light_(root + "/models/cornellbox/light.obj", light);
    MeshTriangle floor("../models/backwall.obj", green, Vector3f::Zero());
    //MeshTriangle left("../models/left.obj", red, Vector3f::Zero());
    //MeshTriangle right("../models/right.obj", green, Vector3f::Zero());
    MeshTriangle light_("../models/light.obj", light, Vector3f(0, 200, 0));
    MeshTriangle bottom("../models/bottom.obj", green, Vector3f::Zero());
    MeshTriangle soldier_1("../models/soldier_zoom_12_final.obj", white, Vector3f(-559, 0, -271));
    MeshTriangle soldier_2("../models/soldier_zoom_12_final.obj", white, Vector3f(160, 0, -271));
    MeshTriangle soldier_3("../models/soldier_zoom_12_final.obj", white, Vector3f(160, 0, -50));
    MeshTriangle soldier_4("../models/soldier_zoom_12_final.obj", white, Vector3f(-546, 0, -50));
    //MeshTriangle soldier_5("../models/soldier_zoom_12.obj", white, Vector3f(-90, -62.5, 320));
    //MeshTriangle soldier_6("../models/soldier_zoom_12.obj", white, Vector3f(-90, -62.5, 280));
    MeshTriangle king("../models/king_zoom_12_final.obj", white, Vector3f::Zero());
    //MeshTriangle floor("../models/pattern_quad.obj", green, Vector3f::Zero(), 1.0f, true);

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
            if (confRenderer["parallelism"].is_number()) {
                r.setParallelism(confRenderer["parallelism"]);
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
    //Sphere sphere({400, 90, 130}, 70, white_glas);

    scene.Add(&floor);
    //scene.Add(&shortbox);
    //scene.Add(&tallbox);
    //scene.Add(&left);
    //scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&bottom);
    //scene.Add(&sphere);
    scene.Add(&soldier_1);
    scene.Add(&soldier_2);
    scene.Add(&soldier_3);
    scene.Add(&soldier_4);
    //scene.Add(&soldier_5);
    //scene.Add(&soldier_6);
    scene.Add(&king);
    
    //scene.Add(&floor2);
    //Material* dummy = new Material();  // Unused when applyPattern = true
    //scene.Add(new MeshTriangle("../models/pattern_floor.obj", green, Vector3f(0, 0, 0), 1.0f, true));

    //Vector3f verts[4] = {{0,0,0}, {552.8,0,0}, {549.6, 0,559.2}, {0,0,559.2}};
    //Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    //uint32_t vertIndex[6] = {0, 2, 1, 2,0,3};
    //Material* mfloor=new Material(ROUGH_CONDUCTOR, Vector3f(0));
    //mfloor->base_reflectance = Vector3f(0.725f, 0.71f, 0.68f);
    //mfloor->textured=true;
    //scene.Add(new MeshTriangle(verts, vertIndex, 2,st,mfloor));

    r.setSpp(4);

    scene.buildBVH();

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout
        << "Time taken: "
        << std::chrono::duration_cast<std::chrono::hours>(stop - start).count()
        << " hours\n";
    std::cout << "          : "
              << std::chrono::duration_cast<std::chrono::minutes>(stop - start)
                     .count()
              << " minutes\n";
    std::cout << "          : "
              << std::chrono::duration_cast<std::chrono::seconds>(stop - start)
                     .count()
              << " seconds\n";

    return 0;
}
