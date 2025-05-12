#pragma once

#include <Eigen/Dense>
enum WaveLenType { RED, GREEN, BLUE };

class WaveLen {
    WaveLenType type;

  public:
    WaveLen(WaveLenType type) : type(type) {}
    //  following CIE 1931 standard (micrometer)
    float getWaveLen() const {
        switch (type) {
        case RED:
            return 0.700f;
        case GREEN:
            return 0.5461f;
        case BLUE:
            return 0.4358f;
        default:
            return 0.0f; // Should never happen
        }
    }
    Eigen::Vector3f getColor() const {
        switch (type) {
        case RED:
            return Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        case GREEN:
            return Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        case BLUE:
            return Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        default:
            return Eigen::Vector3f(1.0f, 1.0f, 1.0f); // Should never happen
        }
    }
};
