#pragma once
#include <nori/object.h>
#include <nori/warp.h>
NORI_NAMESPACE_BEGIN

class PhaseFunction : public NoriObject
{
public:
    // Isotropic default
    PhaseFunction(const PropertyList& props);

    virtual float p(const Vector3f& wo, const Vector3f& wi) const;
    virtual float sample_p(const Vector3f& wo, Vector3f& wi, const Point2f& sample) const;

    std::string toString() const;

    EClassType  getClassType() const { return EPhaseFunction; }
};
NORI_NAMESPACE_END