//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"
#include "global.hpp"

#pragma once
struct hit_payload {
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object *hit_obj;
};

class Renderer {
  public:
    void Render(const Scene &scene);

    void setParallelism(int p) { parellelism = std::min(p, MAX_THRD); }
    void setSpp(int s) { spp = s; }
    std::string path = "./output.png";

  private:
    int parellelism = 8;
    int spp = 1024;
};
