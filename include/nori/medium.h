#pragma once

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/phase_function.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN
struct MediumQueryRecord
{
    // Position of the surface intersection
    Point3f p;

    // maximum t
    float tMax;

    // is sampling valid
    bool isValid;


    MediumQueryRecord() : tMax(0.0f), isValid(false){ }

    MediumQueryRecord(Point3f p, float tMax) : p(p), tMax(tMax), isValid(true){ }

};


class Medium : public NoriObject
{
public:
    Medium(const PropertyList& props);
    Color3f Tr(const Ray3f& ray, Sampler* sampler, MediumQueryRecord& mi) const;
    Color3f Tr(const Point3f& a, const Point3f& b) const;
    Color3f sample(const Ray3f& ray, Sampler* sampler, MediumQueryRecord& mi) const;

    PhaseFunction* getPhaseFunction() const { return phaseFunction; }

    void addChild(NoriObject* child) override;

    void setBoundingBox(BoundingBox3f bounds) const { bounds = bounds; }

    float getDensity(const Point3f& p) const;

    EClassType getClassType() const { return EMedium; }

    std::string Medium::toString() const;

public:
    Color3f m_sigmaA;
    Color3f m_sigmaS;
    Color3f m_sigmaT;
    Color3f m_albedo;

    float m_invDensityMax;

    float m_maxDensity;
    int m_density_function;

    BoundingBox3f bounds;
    PhaseFunction* phaseFunction = nullptr;
};

NORI_NAMESPACE_END