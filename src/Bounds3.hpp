//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "global.hpp"
#include <Eigen/Dense>
#include <array>
#include <limits>
using namespace Eigen;

class Bounds3 {
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3() {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2) {
        pMin = Vector3f(fmin(p1.x(), p2.x()), fmin(p1.y(), p2.y()),
                        fmin(p1.z(), p2.z()));
        pMax = Vector3f(fmax(p1.x(), p2.x()), fmax(p1.y(), p2.y()),
                        fmax(p1.z(), p2.z()));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const {
        Vector3f d = Diagonal();
        if (d.x() > d.y() && d.x() > d.z())
            return 0;
        else if (d.y() > d.z())
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const {
        Vector3f d = Diagonal();
        return 2 * (d.x() * d.y() + d.x() * d.z() + d.y() * d.z());
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3 &b) {
        return Bounds3(
            Vector3f(fmax(pMin.x(), b.pMin.x()), fmax(pMin.y(), b.pMin.y()),
                     fmax(pMin.z(), b.pMin.z())),
            Vector3f(fmin(pMax.x(), b.pMax.x()), fmin(pMax.y(), b.pMax.y()),
                     fmin(pMax.z(), b.pMax.z())));
    }

    Vector3f Offset(const Vector3f &p) const {
        Vector3f o = p - pMin;
        if (pMax.x() > pMin.x())
            o.x() /= pMax.x() - pMin.x();
        if (pMax.y() > pMin.y())
            o.y() /= pMax.y() - pMin.y();
        if (pMax.z() > pMin.z())
            o.z() /= pMax.z() - pMin.z();
        return o;
    }

    bool Overlaps(const Bounds3 &b1, const Bounds3 &b2) {
        bool x = (b1.pMax.x() >= b2.pMin.x()) && (b1.pMin.x() <= b2.pMax.x());
        bool y = (b1.pMax.y() >= b2.pMin.y()) && (b1.pMin.y() <= b2.pMax.y());
        bool z = (b1.pMax.z() >= b2.pMin.z()) && (b1.pMin.z() <= b2.pMax.z());
        return (x && y && z);
    }

    bool Inside(const Vector3f &p, const Bounds3 &b) {
        return (p.x() >= b.pMin.x() && p.x() <= b.pMax.x() &&
                p.y() >= b.pMin.y() && p.y() <= b.pMax.y() &&
                p.z() >= b.pMin.z() && p.z() <= b.pMax.z());
    }
    inline const Vector3f &operator[](int i) const {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray &ray, const Vector3f &invDir,
                           const std::array<int, 3> &dirisNeg) const;
};

inline Vector3f Min(const Vector3f &v1, const Vector3f &v2) {
    std::cout<<"flag53"<<std::endl;
    return Vector3f(fmin(v1.x(), v2.x()), fmin(v1.y(), v2.y()),
                    fmin(v1.z(), v2.z()));
}
inline Vector3f Max(const Vector3f &v1, const Vector3f &v2) {
    return Vector3f(fmax(v1.x(), v2.x()), fmax(v1.y(), v2.y()),
                    fmax(v1.z(), v2.z()));
}
inline bool Bounds3::IntersectP(const Ray &ray, const Vector3f &invDir,
                                const std::array<int, 3> &dirIsNeg) const {
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this
    // because Multiply is faster that Division dirIsNeg: ray direction(x,y,z),
    // dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    std::cout<<"flag50"<<std::endl;
    for(int i=0;i<3;i++){
        // std::cout<<pMin[i]<<std::endl;
        // std::cout<<pMax[i]<<std::endl;
        std::cout<<invDir[i]<<std::endl;
    }
    auto vt1 = (pMin - ray.origin).cwiseProduct(invDir);
    auto vt2 = (pMax - ray.origin).cwiseProduct(invDir);
    std::cout<<"pmin, pmax, invdir"<<std::endl;

    std::cout<<"flag51"<<std::endl;
    auto vtmin = Min(vt1, vt2);
    std::cout<<"flag51.1"<<std::endl;
    auto vtmax = Max(vt1, vt2);
    std::cout<<"flag51.11"<<std::endl;
    auto tmin = std::max({vtmin.x(), vtmin.y(), vtmin.z()}),
         tmax = std::min({vtmax.x(), vtmax.y(), vtmax.z()});
    std::cout<<"flag52"<<std::endl;
    return tmin - EPSILON <= tmax && tmax >= -EPSILON;
}

inline Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2) {
    Bounds3 ret;
    ret.pMin = Min(b1.pMin, b2.pMin);
    ret.pMax = Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3 &b, const Vector3f &p) {
    Bounds3 ret;
    ret.pMin = Min(b.pMin, p);
    ret.pMax = Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
