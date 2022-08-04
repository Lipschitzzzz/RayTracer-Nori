#include <nori/phase_function.h>
NORI_NAMESPACE_BEGIN


float PhaseFunction::p(const Vector3f& wo, const Vector3f& wi) const
{
    return INV_FOURPI;
}

PhaseFunction::PhaseFunction(const PropertyList& props) {}

float PhaseFunction::sample_p(const Vector3f& wo, Vector3f& wi, const Point2f& sample) const
{
    wi = Warp::squareToUniformSphere(sample);
    return INV_FOURPI;
}

std::string PhaseFunction::toString() const
{
    return tfm::format("[ Isotropic ]");
}

NORI_REGISTER_CLASS(PhaseFunction, "iso");
NORI_NAMESPACE_END