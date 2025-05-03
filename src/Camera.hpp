#pragma once

class Camera {
  public:
    int width = 1280, height = 960;
    float fov = 40;
    Camera(int w = 1280, int h = 960) : width(w), height(h) {}
};
