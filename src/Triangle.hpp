#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include <array>
#include <cassert>

inline bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1,
                                 const Vector3f &v2, const Vector3f &orig,
                                 const Vector3f &dir, float &tnear, float &u,
                                 float &v) {
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = dir.cross(edge2);
    float det = edge1.dot(pvec);
    if (det == 0 || det < 0)
        return false;

    Vector3f tvec = orig - v0;
    u = tvec.dot(pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = tvec.cross(edge1);
    v = dir.dot(qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = edge2.dot(qvec) * invDet;
    u *= invDet;
    v *= invDet;

    return true;
}

class Triangle : public Object {
  public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector2f t0, t1, t2; // texture coords
    Vector3f normal;
    float area;
    Material *m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = (e1.cross(e2)).normalized();
        area = e1.cross(e2).norm() * 0.5f;
    }

    bool intersect(const Ray &ray) override;
    bool intersect(const Ray &ray, float &tnear,
                   uint32_t &index) const override;
    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const override {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    Vector3f evalDiffuseColor(const Vector2f &) const override;
    Bounds3 getBounds() override;
    void Sample(Intersection &pos, float &pdf) {
        float x = std::sqrt(get_random_float()), y = get_random_float();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = this->normal;
        pdf = 1.0f / area;
    }
    float getArea() { return area; }
    bool hasEmit() { return m->hasEmission(); }
};

class MeshTriangle : public Object {
  public:
    MeshTriangle(const std::string &filename, Material *mt = new Material(),
                 const Vector3f &translation = Vector3f::Zero(),
                 float zoom = 1.0f) {
        objl::Loader loader;
        loader.LoadFile(filename);
        area = 0;
        m = mt;
        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};

        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z);
                face_vertices[j] = zoom * vert + translation;

                min_vert = min_vert.cwiseMin(face_vertices[j]);
                max_vert = max_vert.cwiseMax(face_vertices[j]);
            }

            Triangle tri(face_vertices[0], face_vertices[1], face_vertices[2],
                         mt);

            if (mt->textured) {
                tri.t0 = Vector2f(mesh.Vertices[i + 0].TextureCoordinate.X,
                                  mesh.Vertices[i + 0].TextureCoordinate.Y);
                tri.t1 = Vector2f(mesh.Vertices[i + 1].TextureCoordinate.X,
                                  mesh.Vertices[i + 1].TextureCoordinate.Y);
                tri.t2 = Vector2f(mesh.Vertices[i + 2].TextureCoordinate.X,
                                  mesh.Vertices[i + 2].TextureCoordinate.Y);
            }

            triangles.emplace_back(tri); // Use emplace_back with temp tri
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri : triangles) {
            ptrs.push_back(&tri);
            area += tri.area;
        }

        bvh = new BVHAccel(ptrs);
    }

    bool intersect(const Ray &ray) { return true; }

    bool intersect(const Ray &ray, float &tnear, uint32_t &index) const {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) {
            const Vector3f &v0 = vertices[vertexIndex[k * 3]];
            const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t,
                                     u, v) &&
                t < tnear) {
                tnear = t;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    Bounds3 getBounds() { return bounding_box; }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const {
        const Vector3f &v0 = vertices[vertexIndex[index * 3]];
        const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = (v1 - v0).normalized();
        Vector3f e1 = (v2 - v1).normalized();
        N = (e0.cross(e1)).normalized();
        const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x() - uv.y()) + st1 * uv.x() + st2 * uv.y();
    }

    Vector3f evalDiffuseColor(const Vector2f &st) const {
        float scale = 5;
        float pattern =
            (fmodf(st.x() * scale, 1) > 0.5) ^ (fmodf(st.y() * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray) {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }

    void Sample(Intersection &pos, float &pdf) {
        bvh->Sample(pos, pdf);
        pos.emit = m->getEmission();
    }
    float getArea() { return area; }
    bool hasEmit() { return m->hasEmission(); }

    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel *bvh;
    float area;

    Material *m;
};

inline bool Triangle::intersect(const Ray &ray) { return true; }
inline bool Triangle::intersect(const Ray &ray, float &tnear,
                                uint32_t &index) const {
    return false;
}

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray) {
    Intersection inter;

    double u, v, t_tmp = 0;
    Vector3f pvec = ray.direction.cross(e2);
    double det = e1.dot(pvec);
    if (fabs(det) < EPSILON)
        return inter;

    double det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = tvec.dot(pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = tvec.cross(e1);
    v = ray.direction.dot(qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = e2.dot(qvec) * det_inv;

    if (t_tmp < 0)
        return inter;
    inter.happened = true;
    inter.coords = ray(t_tmp);
    inter.normal = normal;
    inter.distance = t_tmp;
    inter.tcoords = (1 - u - v) * this->t0 + u * this->t1 + v * this->t2;
    inter.obj = this;
    inter.m = m;
    return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f &) const {
    return Vector3f(0.5, 0.5, 0.5);
}
