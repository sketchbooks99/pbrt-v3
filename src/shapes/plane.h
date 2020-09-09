#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPE_PLANE_H
#define PBRT_SHAPE_PLANE_H

#include "shape.h"

// The intersection algorithms in this src file are referring to following link:
// https://raytracing.github.io/books/RayTracingTheNextWeek.html#rectanglesandlights

namespace pbrt{

class Plane : public Shape {
    public:
        // Plane Axis Public Types
        enum class PlaneAxis { XY, XZ, YZ };

        Plane(const Transform *ObjectToWorld, const Transform *WorldToObject,
        bool reverseOrientation,
        Float a0, Float a1, Float b0, Float b1, Float c, 
        PlaneAxis axisType = PlaneAxis::XY)
        : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
        a0(a0), a1(a1), b0(b0), b1(b1), c(c), axisType(axisType) {}

        Bounds3f ObjectBound() const;
        bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                       bool testAlphaTexture) const;
        bool IntersectP(const Ray& ray, bool testAlphaTexture) const;
        Float Area() const;
        Interaction Sample(const Point2f &u, Float *pdf) const;
    private:
        const Float a0, a1, b0, b1, c;
        const PlaneAxis axisType;
};

std::shared_ptr<Plane> CreatePlaneShape(const Transform *o2w,
                                       const Transform *w2o,
                                       bool reverseOrientation,
                                       Float a0, Float a1, Float b0, Float b1, Float c,
                                       Plane::PlaneAxis axisType = Plane::PlaneAxis::XY);

std::vector<std::shared_ptr<Shape>> CreateAABBShape(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    const Point3f pMin, const Point3f pMax);
}

#endif