#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <math.h>

NORI_NAMESPACE_BEGIN
 
class PathTracerRecursive: public Integrator
{
private:

    bool rr;
    bool nee;
    bool mis;
    float rr_prob;

public:

    PathTracerRecursive(const PropertyList &props)
    {
        rr = props.getBoolean("rr", false);
        nee = props.getBoolean("nee", false);
        mis = props.getBoolean("mis", false);
        rr_prob = props.getFloat("rr_prob", 0.7f);
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        if (mis)
        {
            return misSampling(scene, sampler, ray);
        }
        else if (nee)
        {
            return misSampling(scene, sampler, ray);
        }
        else
        {
            return hemisphereSampling(scene, sampler, ray);
        }
    }

    // balance heuristic
    Color3f misSampling(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f color(0.0f);
        Color3f t = 1;
        Ray3f pathRay = ray;
        float probability;

        float weightBSDR = 1.0f;

        int depth = 1;
        Intersection its;
        if (!scene->rayIntersect(pathRay, its))
        {
            return color;
        }
        while (true)
        {
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord lRec(its.mesh->getEmitter(), pathRay.o, its.p, its.shFrame.n);
                color += t * weightBSDR * its.mesh->getEmitter()->eval(lRec);
            }
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
                color += Li * fr * weigthLightSource * t;
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
            
            BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
            bRec.uv = its.uv;
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;
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
        return color;
    }
    

    Color3f neeSampling(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f color = 0;
        Color3f t = 1;
        Ray3f pathRay = ray;
        float probability;

        float weightBSDR = 1.0f;

        int depth = 1;
        Intersection its;
        if (!scene->rayIntersect(pathRay, its))
        {
            return color;
        }
        while (true)
        {
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord lRec(its.mesh->getEmitter(), pathRay.o, its.p, its.shFrame.n);
                color += t * weightBSDR * its.mesh->getEmitter()->eval(lRec);
            }
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
                Color3f fr = its.mesh->getBSDF()->eval(bRec);
                float pdfBSDR = its.mesh->getBSDF()->pdf(bRec);
                float weigthLightSource = pdfBSDR + pdfLightSource > 0.0f ? pdfLightSource / (pdfBSDR + pdfLightSource) : pdfBSDR;
                color += Li * fr * weigthLightSource * t;
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

            BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;
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
        return color;
    }

    Color3f hemisphereSampling(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {

        Color3f color = 0;
        Color3f t = 1;
        Ray3f pathRay = ray;
        float probability;
        int depth = 1;
        while (true)
        {
            Intersection its;
            if (!scene->rayIntersect(pathRay, its))
                break;

            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord lRecE(its.mesh->getEmitter(), pathRay.o, its.p, its.shFrame.n);
                color += t * its.mesh->getEmitter()->eval(lRecE);
            }

            if (depth >= 3)
            {
                probability = std::min(t.maxCoeff(), 0.99f);
                if (sampler->next1D() > probability)
                    break;
                t /= probability;
            }
            BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;

            pathRay = Ray3f(its.p, its.toWorld(bRec.wo));
            depth++;
        }
        return color;
    }


    // Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return "PathTracerRecursive[]";
    }

};
 
NORI_REGISTER_CLASS(PathTracerRecursive, "path_tracer_recursive");
NORI_NAMESPACE_END