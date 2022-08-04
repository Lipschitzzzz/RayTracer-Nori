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
#include <math.h>
#include <nori/bitmap.h>
#include <filesystem/resolver.h>
#include <fstream>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF
{
public:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
    bool use_cosine;
    std::string fileName;
    bool isTexture = false;
    Bitmap* bmp;
    int textureWidth = 0;
    int textureHeight = 0;

    Microfacet(const PropertyList &propList)
    {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();

        use_cosine = propList.getBoolean("use_cosine", false);

        /*fileName = propList.getString("path", "");

        if (fileName != "")
        {
            filesystem::path filename = getFileResolver()->resolve(fileName);
            isTexture = true;
            bmp = new Bitmap(filename.str());
            textureWidth = bmp->cols();
            textureHeight = bmp->rows();
        }*/
    }

    Color3f getAlbedo() const override { return m_kd; }

    bool useCosine() const override { return use_cosine; }

    // G1 Function
    float G1(const Vector3f& wv, const Vector3f& wh, float alpha) const
    {
        float c = wv.dot(wh) / Frame::cosTheta(wv);
        if (c <= 0)
        {
            return 0;
        }
        float b = 1.0f / (alpha * Frame::tanTheta(wv));
        return b < 1.6f ? (3.535f * b + 2.181f * b * b) / (1.f + 2.276f * b + 2.577f * b * b) : 1;
    }

    // Fresnel Function
    float FresnelDielectric(float cosThetaI, float extIOR, float intIOR) const
    {
        float etaI = extIOR;
        float etaT = intIOR;
        if (abs(extIOR - intIOR) < Epsilon)
        {
            return 0.0f;
        }
        /* Swap the indices of refraction if the interaction starts
           at the inside of the object */
        if (cosThetaI < 0.0f)
        {
            float t = etaI;
            etaI = etaT;
            etaT = t;
            cosThetaI = -cosThetaI;
        }

        /* Using Snell's law, calculate the squared sine of the
           angle between the normal and the transmitted ray */
        float eta = etaI / etaT;
        float sinThetaTSqr = eta * eta * (1 - cosThetaI * cosThetaI);

        if (sinThetaTSqr > 1.0f)
        {
            return 1.0f; /* Total internal reflection! */
        }

        float cosThetaT = sqrt(1.0f - sinThetaTSqr);

        float Rs = (etaI * cosThetaI - etaT * cosThetaT) / (etaI * cosThetaI + etaT * cosThetaT);
        float Rp = (etaT * cosThetaI - etaI * cosThetaT) / (etaT * cosThetaI + etaI * cosThetaT);

        return (Rs * Rs + Rp * Rp) / 2.0f;
    }

    Color3f eval(const BSDFQueryRecord &bRec) const
    {
        if (bRec.measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float cosThetaI = Frame::cosTheta(bRec.wi);
        float cosThetaO = Frame::cosTheta(bRec.wo);
        float d = Warp::squareToBeckmannPdf(wh, m_alpha);
        float g = G1(bRec.wi, wh, m_alpha) * G1(bRec.wo, wh, m_alpha);
        float f = FresnelDielectric(wh.dot(bRec.wi), m_extIOR, m_intIOR);

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
            return bmp->coeff(v, u) / M_PI + m_ks * ((d * f * g) / (4.0f * cosThetaI * cosThetaO));
        }

        return m_kd / M_PI + m_ks * ((d * f * g) / (4.0f * cosThetaI * cosThetaO));
    }

    float pdf(const BSDFQueryRecord &bRec) const
    {
        if (bRec.wo.z() <= 0)
        {
            return 0;
        }
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float d = Warp::squareToBeckmannPdf(wh, m_alpha);
        float jacobian = 1 / (4.0f * abs(wh.dot(bRec.wo)));
        return m_ks * d * Frame::cosTheta(wh) * jacobian + (1 - m_ks) * Frame::cosTheta(bRec.wo) * INV_PI;
        
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const
    {
        if (Frame::cosTheta(bRec.wi) <= 0)
        {
            return Color3f(0.0f);
        }

        // diffuse
        if (_sample.x() > m_ks)
        {
            Point2f sample((_sample.x() - m_ks) / (1.f - m_ks), _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(sample);
            
        }
        else
        {
            Point2f sample(_sample.x() / m_ks, _sample.y());
            Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
            bRec.wo = ((2.0f * wh.dot(bRec.wi) * wh) - bRec.wi).normalized();
        }

        float cosTheta = Frame::cosTheta(bRec.wo);
        if (cosTheta <= 0.f)
            return 0;

        return eval(bRec) * cosTheta / pdf(bRec);
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
    }


    bool isDiffuse() const
    {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    EClassType getClassType() const { return EBSDF; }

    std::string toString() const
    {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
    
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
