#pragma once

#include <Eigen/Dense>
using namespace Eigen;

class Camera {
    Matrix3f orientation = Matrix3f::Identity();

  public:
    int width = 1280, height = 960;
    float fov = 40;
    bool useDOF = false;
    float focal_distance = 100;
    float aperture_radius = 5.0f;
    Camera(int w = 1280, int h = 960) : width(w), height(h) {}
    Vector3f position = {0, 0, 0};
    void lookAt(const Vector3f &target, const Vector3f &up = {0, 1, 0}) {
        Vector3f forward = (target - position).normalized();
        Vector3f left = up.cross(forward).normalized();
        Vector3f new_up = forward.cross(left).normalized();
        orientation.col(0) = left;
        orientation.col(1) = new_up;
        orientation.col(2) = forward;
    }
    Matrix3f getOrientation() const { return orientation; }
};
