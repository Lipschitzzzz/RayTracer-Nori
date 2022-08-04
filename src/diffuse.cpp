/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <random>
#include <ctime>
#include <nori/texture.h>
#include <nori/bitmap.h>
#include <filesystem/resolver.h>
#include <fstream>

NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class Diffuse : public BSDF {
public:

    Color3f m_albedo;
    bool use_cosine;
    Texture* texture;
    std::string fileName;
    bool isTexture = false;
    Bitmap* bmp;
    int textureWidth = 0;
    int textureHeight = 0;

    Diffuse(const PropertyList &propList)
    {
        use_cosine = propList.getBoolean("use_cosine", false);

        m_albedo = propList.getColor("albedo", Color3f(0.5f));

        fileName = propList.getString("path", "");

        if (fileName != "")
        {
            filesystem::path filename = getFileResolver()->resolve(fileName);
            isTexture = true;
            bmp = new Bitmap(filename.str());
            textureWidth = bmp->cols();
            textureHeight = bmp->rows();
        }

        /*PropertyList p;
        p.setString("fileName", fileName);
        
        texture = static_cast<Texture*>(NoriObjectFactory::createInstance("imagetexture", p));
        */
    }

    void addChild(NoriObject* obj)
    {
        switch (obj->getClassType())
        {
        case ETexture:
            if (texture)
                throw NoriException("There can only be one texture per obj!");
            texture = static_cast<Texture*>(obj);
            break;

        default:
            throw NoriException("Diffuse::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    Color3f getAlbedo() const override { return m_albedo; }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec) const
    {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        // This if statement will make this error
        // Integrator: computed an invalid radiance value: [-nan(ind), -nan(ind), -nan(ind)]

        /*std::default_random_engine e(time(0));
        std::uniform_real_distribution<double> u(0.0, 1.0);
        Color3f albedo(0.0f);
        albedo.x() = u(e);
        albedo.y() = u(e);
        albedo.z() = u(e);*/

        Color3f albedo = m_albedo;

        //albedo = texture->eval(bRec.uv);

        if (isTexture)
        {
            int u = bRec.uv.x() * textureWidth;
            int v = (1 - bRec.uv.y()) * textureHeight;
            
            // > 1 repeat
            if (u >= textureWidth)
            {
                u -= textureWidth;
            }

            if (v >= textureHeight)
            {
                v -= textureHeight;
            }

            u = clamp(u, 0, textureWidth - 1);
            v = clamp(v, 0, textureHeight - 1);
            albedo = bmp->coeff(v, u);
        }
        if (use_cosine)
            return albedo * Frame::cosTheta(bRec.wo) * INV_PI;
            // return m_albedo * Frame::cosTheta(bRec.wo) * INV_PI;
        else
            return albedo * Frame::cosTheta(bRec.wo) * INV_PI;
            // return m_albedo * Frame::cosTheta(bRec.wo) * INV_PI;
    }
    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const
    {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside.
           
           Note that the directions in 'bRec' are in local coordinates.*/
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;
        

        if (use_cosine)
            return Frame::cosTheta(bRec.wo) * INV_PI;
        else
            return INV_TWOPI;
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const
    {
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        bRec.measure = ESolidAngle;

        /* Warp a uniformly distributed sample on [0,1]^2
           to a direction on the hemisphere */

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;

        // cosine
        // Color Li = ( cosTheta * (albedo / pi) ) / (cosTheta / pi )
        // pdf = cosTheta / pi

        // uniform
        // Color Li = ( cosTheta * (albedo / pi) ) / (1 / (2 * pi) )
        // pdf = 1 / (2 * pi)

        if (use_cosine)
        {
            bRec.wo = Warp::squareToCosineHemisphere(sample);
            // Color Li = ( cosTheta * (albedo / pi) ) / (cosTheta / pi )
            return eval(bRec) / pdf(bRec);
        }

        else
        {
            bRec.wo = Warp::squareToUniformHemisphere(sample);
            // Color Li = ( cosTheta * (albedo / pi) ) / (1 / (2 * pi) )
            return eval(bRec) / pdf(bRec);
        }
        return Color3f(0.0f);
    }

    bool isDiffuse() const {
        return true;
    }


    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Diffuse[\n"
            "  albedo = %s\n"
            "  use_cosine = %d\n"
            "  texture = %s\n"
            "]", (m_albedo.toString()), use_cosine, fileName);
    }

    EClassType getClassType() const { return EBSDF; }
};

NORI_REGISTER_CLASS(Diffuse, "diffuse");
NORI_NAMESPACE_END
