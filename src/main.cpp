#include "Material.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
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

    // define some materials
    std::unordered_map<std::string, Material*> materials;

    Material *blue_glass = new Material(SMOOTH_DIELECTRIC, Vector3f(0, 0, 0));
    blue_glass->m_color = Vector3f(0.2f, 0.4f, 1.0f);
    blue_glass->iorA = 1.0f;
    blue_glass->iorB = 1.5f;
    blue_glass->roughness = 0.01f;
    blue_glass->base_reflectance = Vector3f(0.04f, 0.04f, 0.04f);
    materials["blue_glass"] = blue_glass;

    // POLISHED METAL
    Material *polished_metal = new Material(ROUGH_CONDUCTOR, Vector3f::Zero());
    polished_metal->base_reflectance = Vector3f(0.725f, 0.71f, 0.68f);
    materials["polished_metal"] = polished_metal;

    int w = 384, h = 384;
    Vector3f camPos(278, 273, -800);
    Vector3f camTarget(278, 273, 0);
    Vector3f camUp(0, 1, 0);

    // default settings
    Vector3f kingPosition = Vector3f(0.0f, 0.0f, 0.0f);
    Material* kingMaterial = materials["blue_glass"];

    std::vector<std::pair<Vector3f, Material*>> soldiers = {
        {Vector3f(140, 0, -50), materials["blue_glass"]},
        {Vector3f(-456, 0, -50), materials["blue_glass"]},
        {Vector3f(-570, 0, -400), materials["blue_glass"]},
        {Vector3f(180, 0, -400), materials["blue_glass"]}
    };

    Vector3f lightPosition = Vector3f(0, 200, 0);

    Material* wallMaterial = materials["blue_glass"];
    Material* floorMaterial = materials["blue_glass"];
    float brightness_scale = 1.0f;

    //  Reading configuration file
    std::ifstream confJson("conf.json");
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

            if (is_v3(confScene["kingPosition"])) {
                kingPosition = Vector3f(confScene["kingPosition"][0],
                                        confScene["kingPosition"][1],
                                        confScene["kingPosition"][2]);
            }
            if (confScene["kingMaterial"].is_string()) {
                kingMaterial = materials[confScene["kingMaterial"]];
            }
            
            if (confScene.contains("soldierPositions") && confScene.contains("soldierMaterials")) {
                const auto& positions = confScene["soldierPositions"];
                const auto& matNames = confScene["soldierMaterials"];
                soldiers.clear();
                for (size_t i = 0; i < positions.size(); i++) {
                    const auto& pos = positions[i];
                    Vector3f position(pos[0], pos[1], pos[2]);
                    std::string matName = matNames[i];
                    Material* material = materials.count(matName) ? materials.at(matName) : materials["polished_metal"];
                    soldiers.push_back(std::make_pair(position, material));
                }
            }

            if (is_v3(confScene["lightPosition"])) {
               lightPosition = Vector3f(confScene["lightPosition"][0],
                                       confScene["lightPosition"][1],
                                       confScene["lightPosition"][2]);
            }
            if (confScene["lightBrightness"].is_number_float()) {
                brightness_scale = confScene["lightBrightness"];
            }
            if(confScene["floorMaterial"].is_string()) {
                floorMaterial = materials[confScene["floorMaterial"]];
            }
            if(confScene["wallMaterial"].is_string()) {
                wallMaterial = materials[confScene["wallMaterial"]];
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

    if (soldiers.size() < 4) {
        std::cerr << "Error: Need at least 4 soldiers in JSON config" << std::endl;
        exit(EXIT_FAILURE);
    }
    MeshTriangle soldier_1("../models/soldier_zoom_12_final.obj", soldiers[0].second, soldiers[0].first);
    MeshTriangle soldier_2("../models/soldier_zoom_12_final.obj", soldiers[1].second, soldiers[1].first);
    MeshTriangle soldier_3("../models/soldier_zoom_12_final.obj", soldiers[2].second, soldiers[2].first);
    MeshTriangle soldier_4("../models/soldier_zoom_12_final.obj", soldiers[3].second, soldiers[3].first);
    
    MeshTriangle king("../models/king_zoom_12_final.obj", kingMaterial, kingPosition);
    MeshTriangle wall("../models/backwall.obj", wallMaterial, Vector3f::Zero());
    MeshTriangle floor("../models/bottom.obj", floorMaterial, Vector3f::Zero());

    Material *light = new Material(
        ROUGH_CONDUCTOR,
        brightness_scale * (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
         15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
         18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    MeshTriangle light_("../models/light.obj", light, lightPosition);

    //  Scene building
    scene.Add(&wall);
    scene.Add(&light_);
    scene.Add(&floor);
    scene.Add(&soldier_1);
    scene.Add(&soldier_2);
    scene.Add(&soldier_3);
    scene.Add(&soldier_4);
    scene.Add(&king);

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