#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <omp.h>
#include <random>

#undef M_PI
#define M_PI 3.141592653589793f

extern const float EPSILON;
const int MAX_THRD = 16;
const float kInfinity = std::numeric_limits<float>::max();
static std::mt19937 RNGS[MAX_THRD];

inline float clamp(const float &lo, const float &hi, const float &v) {
    return std::max(lo, std::min(hi, v));
}

inline bool solveQuadratic(const float &a, const float &b, const float &c,
                           float &x0, float &x1) {
    float discr = b * b - 4 * a * c;
    if (discr < 0)
        return false;
    else if (discr == 0)
        x0 = x1 = -0.5 * b / a;
    else {
        float q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1)
        std::swap(x0, x1);
    return true;
}

inline Eigen::Vector3f lerp(const Eigen::Vector3f &a, const Eigen::Vector3f &b,
                            const float &t) {
    return a * (1 - t) + b * t;
}

inline void init_rngs(int thrd_num) {
    std::random_device dev;
    for (int i = 0; i < thrd_num; ++i) {
        RNGS[i] = std::mt19937(dev());
    }
}

inline float get_random_float() {
    std::uniform_real_distribution<float> dist(
        0.f, 1.f); // distribution in range [1, 6]

    return dist(RNGS[omp_get_thread_num()]);
}

inline void UpdateProgress(float progress) {
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};
