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

    std::string model_quality = "low";
    std::string king_model = "../models/" + model_quality + "_king.obj";
    std::string soldier_model = "../models/" + model_quality + "_soldier.obj";

    int w = 384, h = 384;
    Vector3f camPos(278, 273, -800);
    Vector3f camTarget(278, 273, 0);
    Vector3f camUp(0, 1, 0);

    // define some materials
    std::unordered_map<std::string, Material *> materials;

    // rough red metal
    Material *rough_red_conductor =
        new Material(ROUGH_CONDUCTOR, Vector3f(0, 0, 0));
    rough_red_conductor->roughness = 0.1f;
    rough_red_conductor->base_reflectance = Vector3f(1.0f, 0.0f, 0.0f);
    materials["rough_red_conductor"] = rough_red_conductor;

    // rough white metal
    Material *rough_white_conductor =
        new Material(ROUGH_CONDUCTOR, Vector3f::Zero());
    rough_white_conductor->base_reflectance = Vector3f(0.725f, 0.71f, 0.68f);
    rough_white_conductor->roughness = 0.4f;
    materials["rough_white_conductor"] = rough_white_conductor;

    // green mirror
    Material *green_mirror = new Material(ROUGH_CONDUCTOR, Vector3f(0, 0, 0));
    green_mirror->roughness = 0.01f;
    green_mirror->base_reflectance = Vector3f(0.14f, 1.0f, 0.14f);
    materials["green_mirror"] = green_mirror;

    // reflexive gold
    Material *gold_conductor =
        new Material(SMOOTH_CONDUCTOR, Vector3f(0, 0, 0));
    gold_conductor->roughness = 0.0001f;
    gold_conductor->base_reflectance = Vector3f(1.0f, 0.85f, 0.57f);
    materials["gold_conductor"] = gold_conductor;

    // silver mirror
    Material *silver_mirror = new Material(SMOOTH_CONDUCTOR, Vector3f(0, 0, 0));
    silver_mirror->roughness = 0.001f; // Very smooth
    silver_mirror->base_reflectance =
        Vector3f(0.972f, 0.960f, 0.915f); // Silver (very reflective)
    materials["silver_mirror"] = silver_mirror;

    // smooth glass
    Material *smooth_glass = new Material(SMOOTH_DIELECTRIC, Vector3f(0, 0, 0));
    smooth_glass->iorA = 1.7f;
    smooth_glass->iorB = 0.04f;
    smooth_glass->roughness = 0.01f;
    materials["smooth_glass"] = smooth_glass;

    // clear rough plastic
    Material *clear_rough_plastic =
        new Material(ROUGH_DIELECTRIC, Vector3f(0, 0, 0));
    clear_rough_plastic->iorA = 1.5f;
    clear_rough_plastic->iorB = 0.01f;
    clear_rough_plastic->roughness = 0.02f;
    materials["clear_rough_plastic"] = clear_rough_plastic;

    // rough plastic
    Material *rough_plastic = new Material(ROUGH_DIELECTRIC, Vector3f(0, 0, 0));
    rough_plastic->iorA = 1.5f;
    rough_plastic->iorB = 0.01f;
    rough_plastic->roughness = 0.4f;
    materials["rough_plastic"] = rough_plastic;

#ifdef DEMO
    Material *light = new Material(
        ROUGH_CONDUCTOR,
        (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
         15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
         18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));

    MeshTriangle light_("../models/cornellbox/light.obj", light);
    MeshTriangle back("../models/cornellbox/floor.obj", rough_white_conductor);
    MeshTriangle ground("../models/bottom.obj", rough_white_conductor);
    MeshTriangle left("../models/cornellbox/left.obj", rough_red_conductor);
    MeshTriangle right("../models/cornellbox/right.obj", gold_conductor);

    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", green_mirror);
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", rough_plastic);
    Sphere big_sphere({400, 90, 3}, 80, smooth_glass);
    Sphere mid_sphere({250, 260, 230}, 60, clear_rough_plastic);
    Sphere small_sphere({120, 390, 400}, 50, silver_mirror);

    scene.Add(&back);
    scene.Add(&ground);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&big_sphere);
    scene.Add(&mid_sphere);
    scene.Add(&small_sphere);

    camera.useDOF = true;
    camera.focal_distance = 900;
    camera.aperture_radius = 40;
#else
    // =================================================================================
    // =================================================================================
    // ====================START OF FINAL PRODUCT SCENE CONSTRUCTION==================== 
    // (controled by conf.json file located under the root directory)

    // default settings
    Vector3f kingPosition = Vector3f(0.0f, 0.0f, 0.0f);
    Material *kingMaterial = materials["rough_plastic"];

    Vector3f lightPosition = Vector3f(0, 200, 0);

    Material *wallMaterial = materials["rough_plastic"];
    Material *floorMaterial = materials["rough_plastic"];
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
            if (confScene["model_quality"].is_string()) {
                model_quality = confScene["model_quality"];
            }

            if (confScene["includeShadow"].is_boolean()) {
                scene.enableShadow(confScene["includeShadow"]);
            }
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

            if (confScene.contains("soldierLineUpPosition") &&
                confScene.contains("soldierMaterials")) {
                
                // extract starting soldier position on left & right
                const auto &left_row_positions = confScene["soldierLeftRowPosition"];
                const auto &right_row_positions = confScene["soldierRightRowPosition"];
                // extract space between sodilers
                float x_spacing = confScene["soldierXSpacing"];
                float y_spacing = confScene["soldierYSpacing"];
                float z_spacing = confScene["soldierZSpacing"];

                const auto &soldier_count = confScene["soldierCountPerRow"];
                const auto &matNames = confScene["soldierMaterials"];

                for (int i = 0; i < soldier_count; i++) {
                    // calculate poition and material types
                    float x_offset = i * x_spacing;
                    float y_offset = i * y_spacing;
                    float z_offset = i * z_spacing;
                    Vector3f leftPos(left_row_positions[0]+x_offset, left_row_positions[1]+y_offset, left_row_positions[2]+z_offset);
                    Vector3f rightPos(right_row_positions[0]+x_offset, right_row_positions[1]+y_offset, right_row_positions[2]+z_offset);
                    Material *left_material = (i < matnames.size()) ? materials[matNames[i]] : materials["rough_plastic"];
                    Material *right_material = (i + soldier_count < matnames.size()) ? materials[matName[i*2]] : materials["rough_plastic"];

                    // create soldier pair mesh & add to scene
                    auto* soldier_left = new MeshTriangle(soldier_model, left_material, leftPos);
                    auto* soldier_right = new MeshTriangle(soldier_model, right_material, rightPos);
                    scene.Add(soldier_left);
                    scene.Add(soldier_right);
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
            if (confScene["floorMaterial"].is_string()) {
                floorMaterial = materials[confScene["floorMaterial"]];
            }
            if (confScene["wallMaterial"].is_string()) {
                wallMaterial = materials[confScene["wallMaterial"]];
            }
        }

    } catch (const std::exception &e) {
        std::cerr << "Error when reading json config: " << e.what()
                  << std::endl;
    }

    if (soldiers.size() < 4) {
        std::cerr << "Error: Need at least 4 soldiers in JSON config"
                  << std::endl;
        exit(EXIT_FAILURE);
    }

    MeshTriangle king(king_model, kingMaterial,
                      kingPosition);
    MeshTriangle wall("../models/backwall.obj", wallMaterial, Vector3f::Zero());
    MeshTriangle floor("../models/bottom.obj", floorMaterial, Vector3f::Zero());

    Material *light = new Material(
        ROUGH_CONDUCTOR,
        brightness_scale *
            (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
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

    // =====================END OF FINAL PRODUCT SCENE
    // CONSTRUCTION=====================
    // =================================================================================
    // =================================================================================
#endif

    camera.width = w;
    camera.height = h;
    camera.position = camPos;
    camera.lookAt(camTarget, camUp);
    scene.camera = camera;

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
