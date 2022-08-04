#include <nori/emitter.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN
class AreaEmitter : public Emitter
{
    private:
    
    Color3f m_radiance;
    
    public:
    
        AreaEmitter(const PropertyList& propList)
        {
            m_radiance = propList.getColor("radiance", Color3f(0.0f));
        }

        Color3f eval(const EmitterQueryRecord& lRec) const override
        {
            return (lRec.n.dot(lRec.d) < 0.0f) ? m_radiance : 0.0f;
        }

        Color3f getRadiance() const override
        {
            return m_radiance;
        }

        Color3f sample(const Mesh* mesh, EmitterQueryRecord& lRec, Sampler* sample) const override
        {
            SampleMeshResult sRec = mesh->sampleSurfaceUniform(sample);
            lRec.p = sRec.p;
            lRec.n = sRec.n;
            lRec.d = (lRec.p - lRec.ref).normalized();
            lRec.shadowRay = Ray3f(lRec.ref, lRec.d, 5 * Epsilon, (lRec.p - lRec.ref).norm() - Epsilon);
            lRec.pdf = pdf(mesh, lRec);
            if (lRec.pdf > 0.0f && !std::isnan(lRec.pdf) && !std::isinf(lRec.pdf))
            {
                return eval(lRec) / lRec.pdf;
            }
            return Color3f(0.0f);
        }

        float pdf(const Mesh* mesh, const EmitterQueryRecord& lRec) const override
        {
            float cosTheta = lRec.n.dot(-lRec.d);
            if (cosTheta > 0.0f)
            {
                return mesh->getPdf().getNormalization() * (lRec.p - lRec.ref).squaredNorm() / cosTheta;
            }
            return 0.0f;
        }

        std::string toString() const override
        {
            return tfm::format(
                "MeshEmitter[\n"
                "m_radiance = %d\n"
                "]", m_radiance);
        }

        EClassType getClassType() { return EEmitter; }

    
};

NORI_REGISTER_CLASS(AreaEmitter, "area");
NORI_NAMESPACE_END