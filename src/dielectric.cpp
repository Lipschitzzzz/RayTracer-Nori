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

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    bool useCosine() const override { return 0; }

    Vector3f refract(const Vector3f& wi, const Vector3f& n, float eta) const
    {
        float cosThetaI = wi.dot(n);
        if (cosThetaI < 0)
            eta = 1.0f / eta;
        float cosThetaTSqr = 1 - (1 - cosThetaI * cosThetaI) * (eta * eta);
        if (cosThetaTSqr <= 0.0f)
            return Vector3f(0.0f);
        float sign = cosThetaI >= 0.0f ? 1.0f : -1.0f;
        return n * (-cosThetaI * eta + sign * sqrt(cosThetaTSqr)) + wi * eta;
    }

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

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const
    {
        bRec.measure = EDiscrete;
        float cosThetaI = Frame::cosTheta(bRec.wi);
        float kr = FresnelDielectric(cosThetaI, m_extIOR, m_intIOR);
        if (sample.x() < kr)
        {
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            bRec.eta = 1.f;
            return Color3f(1.0f);
        }
        else
        {
            
            Vector3f n = Vector3f(0.0f, 0.0f, 1.0f);
            float factor = m_intIOR / m_extIOR;
            if (Frame::cosTheta(bRec.wi) < 0.f)
            {
                factor = m_extIOR / m_intIOR;
                n.z() = -1.0f;
            }

            bRec.wo = refract(-bRec.wi, n, factor);
            bRec.eta = m_intIOR / m_extIOR;
            return Color3f(1.0f);
        }
        
    }
    Color3f getAlbedo() const override { return Color3f(0.5f); }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
