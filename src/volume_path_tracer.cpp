#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/medium.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <math.h>
NORI_NAMESPACE_BEGIN

class IntegratorVolumePathTracer : public Integrator
{
private:

    bool rr;
    bool nee;
    bool mis;
    float rr_prob;

public:
    IntegratorVolumePathTracer(const PropertyList& props)
    {
        rr = props.getBoolean("rr", false);
        rr_prob = props.getFloat("rr_prob", 0.7f);

    }
    
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f color(0.0f);
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return color;
        }
        Color3f t(1.0f);

        Ray3f pathRay = ray;

        float probability;
        float weightBSDR = 1.0f;

        int depth = 1;

        Medium* medium = scene->getMedias()[0];
        // medium->setBoundingBox(BoundingBox3f(Point3f(-9.0f, -9.0f, -9.0f), Point3f(-999.0f, -999.0f, -999.0f)));
        // std::cout << medium->bounds.toString() << std::endl;
        bool foundIntersection = scene->rayIntersect(pathRay, its);
        // std::cout << scene->getMeshes().size() << std::endl;

        
        // std::cout << foundIntersection << std::endl;

        while (true)
        {
            float tmax;

            if (foundIntersection)
                tmax = (its.p - pathRay.o).norm();
            else
                tmax = its.t;

            //sample mean free path
            MediumQueryRecord mi;
            mi.tMax = tmax;
            Color3f mediumColor = medium->sample(pathRay, sampler, mi);

            if (mi.isValid)
            {
                Vector3f wo;
                // std::cout << 222 << std::endl;
                float pdf_mat = medium->getPhaseFunction()->sample_p(pathRay.d, wo, sampler->next2D());

                std::vector<Mesh*> meshes = scene->getMeshes();
                std::vector<int> index;
                int i = 0;
                for (Mesh* mesh : meshes)
                {
                    if (mesh->isEmitter())
                    {
                        index.emplace_back(i);
                    }
                    i += 1;
                }
                size_t n = index.size();
                size_t indexRandom = std::min(
                    static_cast<size_t>(std::floor(n * sampler->next1D())),
                    n - 1);
                const Mesh* mesh = meshes[index[indexRandom]];
                const Emitter* light = mesh->getEmitter();

                EmitterQueryRecord lRec(mi.p);

                Color3f Li = light->sample(mesh, lRec, sampler) * n;
                // std::cout << 222 << std::endl;

                //float pdfLightSource = light->pdf(mesh, lRec);
                // std::cout << 111 << std::endl;

                if (!scene->rayIntersect(lRec.shadowRay))
                {
                    t *= mediumColor;
                    mi.tMax = lRec.shadowRay.maxt;
                    color += t * medium->Tr(lRec.shadowRay, sampler, mi) * Li * pdf_mat;
                }
                else
                {
                    t *= mediumColor;
                }

                if (rr)
                {
                    if (depth >= 3)
                    {
                        probability = std::min(t.maxCoeff(), 0.99f);
                        if (sampler->next1D() > probability)
                        {
                            return color;
                        }
                        t /= probability;
                    }
                }

                pathRay = Ray3f(mi.p, wo.normalized());
                if (scene->rayIntersect(pathRay, its))
                {
                    if (its.mesh->isEmitter())
                    {
                        EmitterQueryRecord lRec(its.mesh->getEmitter(), pathRay.o, its.p, its.shFrame.n);

                        float pdf_em = its.mesh->getEmitter()->pdf(its.mesh, lRec);
                        weightBSDR = pdf_mat + pdf_em > 0.f ? pdf_mat / (pdf_mat + pdf_em) : pdf_mat;
                    }
                }

            }

            // surface
            else
            {
                // emitter
                if (its.mesh->isEmitter())
                {
                    EmitterQueryRecord lRec(its.mesh->getEmitter(), pathRay.o, its.p, its.shFrame.n);
                    color += t * weightBSDR * its.mesh->getEmitter()->eval(lRec) * medium->Tr(pathRay, sampler, mi);
                }

                // direct light sampling
                std::vector<Mesh*> meshes = scene->getMeshes();
                std::vector<int> index;
                int i = 0;
                for (Mesh* mesh : meshes)
                {
                    if (mesh->isEmitter())
                    {
                        index.emplace_back(i);
                    }
                    i += 1;
                }
                size_t n = index.size();
                size_t indexRandom = std::min(
                    static_cast<size_t>(std::floor(n * sampler->next1D())),
                    n - 1);
                const Mesh* mesh = meshes[index[indexRandom]];
                const Emitter* light = mesh->getEmitter();
                EmitterQueryRecord lRec(its.p);
                Color3f Li = light->sample(mesh, lRec, sampler) * n;
                float pdfLightSource = light->pdf(mesh, lRec);
                if (!scene->rayIntersect(lRec.shadowRay))
                {
                    BSDFQueryRecord bRec(its.toLocal(-pathRay.d), its.toLocal(lRec.d), ESolidAngle);

                    bRec.uv = its.uv;

                    Color3f fr = its.mesh->getBSDF()->eval(bRec);
                    float pdfBSDR = its.mesh->getBSDF()->pdf(bRec);
                    float weigthLightSource = pdfBSDR + pdfLightSource > 0.0f ? pdfLightSource / (pdfBSDR + pdfLightSource) : pdfBSDR;
                    mi.tMax = lRec.shadowRay.maxt;
                    color += Li * fr * weigthLightSource * t * medium->Tr(lRec.shadowRay, sampler, mi);
                }

                // russian roulette
                if (rr)
                {
                    if (depth >= 3)
                    {
                        probability = std::min(t.maxCoeff(), 0.99f);
                        if (sampler->next1D() > probability)
                        {
                            return color;
                        }
                        t /= probability;
                    }
                }

                // bsdf
                BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
                bRec.uv = its.uv;
                Color3f fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                t *= fr;

                pathRay = Ray3f(its.p, its.toWorld(bRec.wo));

                float pdfBSDR = its.mesh->getBSDF()->pdf(bRec);

                Point3f origin = its.p;
                if (!scene->rayIntersect(pathRay, its))
                {
                    return color;
                }

                if (its.mesh->isEmitter())
                {
                    EmitterQueryRecord newLRec(its.mesh->getEmitter(), origin, its.p, its.shFrame.n);
                    float newWeigthLightSource = its.mesh->getEmitter()->pdf(its.mesh, newLRec);
                    weightBSDR = pdfBSDR + newWeigthLightSource > 0.f ? pdfBSDR / (pdfBSDR + newWeigthLightSource) : pdfBSDR;
                }
                if (bRec.measure == EDiscrete)
                {
                    weightBSDR = 1.0f;
                }
                depth++;
            }
            
        }
        return color;
    }
    


    // Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const { return Color3f(0.0f); }
    // 
    // Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return "IntegratorVolumePathTracer[]";
    }

};

NORI_REGISTER_CLASS(IntegratorVolumePathTracer, "volume_path_tracer");
NORI_NAMESPACE_END