#pragma once
#include <nori/object.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class Texture : public NoriObject
{
public:

    EClassType getClassType() const { return ETexture; }

    virtual Color3f eval(const Point2f& uv) const = 0;
};

NORI_NAMESPACE_END