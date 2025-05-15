#pragma once

#include <Eigen/Dense>
enum WaveLenType { RED, GREEN, BLUE };

//  following CIE 1931 standard (micrometer)
inline constexpr float getWaveLen(const WaveLenType &type) {
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
inline Eigen::Vector3f getColor(const WaveLenType &type) {
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
inline constexpr float extract(const WaveLenType &type,
                               const Eigen::Vector3f &color) {
    switch (type) {
    case RED:
        return color.x();
    case GREEN:
        return color.y();
    case BLUE:
        return color.z();
    default:
        return 0.0f; // Should never happen
    }
}
