#include "shapes/plane.h"
#include "paramset.h"
#include "sampling.h"
#include "stats.h"

namespace pbrt {

Bounds3f Plane::ObjectBound() const {
    switch(axisType) {
    case PlaneAxis::XY: 
        return Bounds3f(Point3f(a0, b0, c), Point3f(a1, b1, c));
        break;
    case PlaneAxis::XZ: 
        return Bounds3f(Point3f(a0, c, b0), Point3f(a1, c, b1));
        break;
    case PlaneAxis::YZ: 
        return Bounds3f(Point3f(c, a0, b0), Point3f(c, a1, b1));
        break;
    }
}

bool Plane::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                      bool testAlphaTexture) const {
    ProfilePhase p(Prof::ShapeIntersect);

    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Ray origin vector corresponding to PlaneAxis type
    Float ro_a = 0, ro_b = 0, ro_c = 0; 
    // Ray direction vector corresponding to PlaneAxis type
    Float rd_a = 0, rd_b = 0, rd_c = 0; 
    switch(axisType) {
    case PlaneAxis::XY: {
        ro_a = ray.o.x; ro_b = ray.o.y; ro_c = ray.o.z;
        rd_a = ray.d.x; rd_b = ray.d.y; rd_c = ray.d.z;
        break;
    }
    case PlaneAxis::XZ: {
        ro_a = ray.o.x; ro_b = ray.o.z; ro_c = ray.o.y;
        rd_a = ray.d.x; rd_b = ray.d.z; rd_c = ray.d.y;
        break;
    }
    case PlaneAxis::YZ: {
        ro_a = ray.o.y; ro_b = ray.o.z; ro_c = ray.o.x;
        rd_a = ray.d.y; rd_b = ray.d.z; rd_c = ray.d.x;
        break;
    }
    }

    Float tShapeHit = (c - ro_c) / rd_c;

    if(tShapeHit <= 0 || tShapeHit >= ray.tMax) 
        return false;

    Float a = ro_a + tShapeHit * rd_a;
    Float b = ro_b + tShapeHit * rd_b;
    if(a <= a0 || a >= a1 || b <= b0 || b >= b1)
        return false;
    
    Point3f pHit = ray(tShapeHit);

    Float u = (a - a0) / (a1 - a0);
    Float v = (b - b0) / (b1 - b0);

    Vector3f dpdu(1,0,0), dpdv(0,1,0);

    Normal3f dndu(0,0,0), dndv(0,0,0);

    Vector3f pError(0,0,0);

    // Initialize _SurfaceIntersection_ from parametric information
    *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v),
                                                 -r.d, dpdu, dpdv, dndu, dndv,
                                                 r.time, this));
    
    // Update _tHit_ for quadric intersection 
    *tHit = (Float)tShapeHit;
    return true;
}

bool Plane::IntersectP(const Ray& r, bool testAlphaTexture) const {
    ProfilePhase p(Prof::ShapeIntersectP);
    // Transform _Ray_ to object space 
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Compute intersection for plane

    // Ray origin vector corresponding to PlaneAxis type
    Float ro_a, ro_b, ro_c; 
    // Ray direction vector corresponding to PlaneAxis type
    Float rd_a, rd_b, rd_c; 
    switch(axisType) {
    case PlaneAxis::XY: {
        ro_a = ray.o.x; ro_b = ray.o.y; ro_c = ray.o.z;
        rd_a = ray.d.x; rd_b = ray.d.y; rd_c = ray.d.z;
        break;
    }
    case PlaneAxis::XZ: {
        ro_a = ray.o.x; ro_b = ray.o.z; ro_c = ray.o.y;
        rd_a = ray.d.x; rd_b = ray.d.z; rd_c = ray.d.y;
        break;
    }
    case PlaneAxis::YZ: {
        ro_a = ray.o.y; ro_b = ray.o.z; ro_c = ray.o.x;
        rd_a = ray.d.y; rd_b = ray.d.z; rd_c = ray.d.x;
        break;
    }
    }

    Float tShapeHit = (c - ro_c) / rd_c;

    if(tShapeHit <= 0 || tShapeHit >= ray.tMax)
        return false;

    auto a = ro_a + tShapeHit * rd_a;
    auto b = ro_b + tShapeHit * rd_b;
    if(a < a0 || a > a1 || b < b0 || b > b1)
        return false;
    
    return true;
}

Float Plane::Area() const {
    return (a1 - a0) * (b1 - b0);
}

Interaction Plane::Sample(const Point2f &u, Float *pdf) const {
    Point3f pObj;
    Interaction it;
    switch(axisType) {
    case PlaneAxis::XY:
        it.n = Normalize((*ObjectToWorld)(Normal3f(0, 0, 1)));
        pObj = Point3f((a1 - a0) * u[0], (b1 - b0) * u[1], 0.0001);
        break;
    case PlaneAxis::XZ:
        it.n = Normalize((*ObjectToWorld)(Normal3f(0, 1, 0)));
        pObj = Point3f((a1 - a0) * u[0], 0.0001, (b1 - b0) * u[1]);
        break;
    case PlaneAxis::YZ:
        it.n = Normalize((*ObjectToWorld)(Normal3f(1, 0, 0)));
        pObj = Point3f(0.0001, (a1 - a0) * u[0], (b1 - b0) * u[1]);
        break;
    }
    if(reverseOrientation) it.n *= -1;
    it.p = (*ObjectToWorld)(pObj, Vector3f(0, 0, 0), &it.pError);
    *pdf = 1 / Area();
    return it;
}

std::shared_ptr<Plane> CreatePlaneShape(const Transform *o2w,
                                       const Transform *w2o,
                                       bool reverseOrientation,
                                       Float a0, Float a1, Float b0, Float b1, Float c,
                                       Plane::PlaneAxis axisType) {
    return std::make_shared<Plane>(o2w, w2o, reverseOrientation, 
                                   a0, a1, b0, b1, c, axisType);
}

std::vector<std::shared_ptr<Shape>> CreateAABBShape(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    Point3f pMin, Point3f pMax) {

    std::vector<std::shared_ptr<Shape>> aabb;

    // TODO: Calculate Transform for each plane.

    // XY Plane
    aabb.push_back(CreatePlaneShape(o2w, w2o, reverseOrientation,
                                    pMin.x, pMax.x, pMin.y, pMax.y, pMin.z, Plane::PlaneAxis::XY));
    aabb.push_back(CreatePlaneShape(o2w, w2o, reverseOrientation,
                                    pMin.x, pMax.x, pMin.y, pMax.y, pMin.z, Plane::PlaneAxis::XY));
    // XZ Plane
    aabb.push_back(CreatePlaneShape(o2w, w2o, reverseOrientation,
                                    pMin.x, pMax.x, pMin.z, pMax.z, pMin.y, Plane::PlaneAxis::XZ));
    aabb.push_back(CreatePlaneShape(o2w, w2o, reverseOrientation,
                                    pMin.x, pMax.x, pMin.z, pMax.z, pMin.y, Plane::PlaneAxis::XZ));
    // YZ Plane
    aabb.push_back(CreatePlaneShape(o2w, w2o, reverseOrientation,
                                    pMin.y, pMax.y, pMin.z, pMax.z, pMin.x, Plane::PlaneAxis::YZ));
    aabb.push_back(CreatePlaneShape(o2w, w2o, reverseOrientation,
                                    pMin.y, pMax.y, pMin.z, pMax.z, pMin.x, Plane::PlaneAxis::YZ));

    return aabb;
}

// ===== Intersection algorithm =====

}