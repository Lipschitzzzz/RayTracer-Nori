#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/area_emitter.h>
#include <nori/parallelogram_emitter.h>
#include <nori/bsdf.h>
#include <nori/bitmap.h>
#include <math.h>
NORI_NAMESPACE_BEGIN
 
class IntegratorDirectLighting : public Integrator
{
private:

    bool surface_sampling;
    bool mis_sampling;
    
public:
    IntegratorDirectLighting(const PropertyList &props)
    {
        surface_sampling = props.getBoolean("surface_sampling", false);
        mis_sampling = props.getBoolean("mis_sampling", false);
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        if (mis_sampling)
        {
            return lightSurfaceSampling(scene, sampler, ray);
        }
        else if (surface_sampling)
        {
            return lightSurfaceSampling(scene, sampler, ray);
        }
        else
        {
            return hemisphereSampling(scene, sampler, ray);
        }
    }

    Color3f misSampling(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        return Color3f(0.0f);
    }

    // light surface sampling version
    Color3f lightSurfaceSampling(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f value(0.0f);
        Intersection its;

        if (!scene->rayIntersect(ray, its))
        {
            return value;
        }

        if (its.mesh->isEmitter())
        {
            return its.mesh->getEmitter()->getRadiance();
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
        EmitterQueryRecord eqr(its.p);

        Color3f Li = light->sample(mesh, eqr, sampler);
        if (scene->rayIntersect(eqr.shadowRay))
        {
            Li = 0;
        }

        BSDFQueryRecord bqr(its.toLocal(-ray.d), its.toLocal(eqr.d), ESolidAngle);

        bqr.uv = its.uv;

        Color3f fr = its.mesh->getBSDF()->eval(bqr);

        value += Li * fr / (1.0f / n);

        return value;

    }


    // hemisphere sampling version
    Color3f hemisphereSampling(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {

        Intersection its;

        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }

        if (its.mesh->isEmitter())
        {
            return its.mesh->getEmitter()->getRadiance();
        }

        Color3f value(0.f);
        
        Frame frame = its.shFrame;
        const BSDF* bsdf = its.mesh->getBSDF();
        Point2f point_2d_square = sampler->next2D();
        Vector3f point_unit_hemisphere;
        
        if (bsdf->useCosine())
        {
            point_unit_hemisphere = Warp::squareToCosineHemisphere(point_2d_square);
            Vector3f point_global_hemisphere = frame.toWorld(point_unit_hemisphere);

            Ray3f pathRay = Ray3f(its.p + Epsilon * its.shFrame.n, point_global_hemisphere);

            Intersection its2;

            if (scene->rayIntersect(pathRay, its2) && its2.mesh->isEmitter())
            {

                value = its2.mesh->getEmitter()->getRadiance() * bsdf->getAlbedo() * INV_PI * its.shFrame.n.dot(point_global_hemisphere) / (its.shFrame.n.dot(point_global_hemisphere) * INV_PI);
            }

            return value;
        }
        else
        {

            point_unit_hemisphere = Warp::squareToUniformHemisphere(point_2d_square);
            Vector3f point_global_hemisphere = frame.toWorld(point_unit_hemisphere);

            Ray3f pathRay = Ray3f(its.p + Epsilon * its.shFrame.n, point_global_hemisphere);

            Intersection its2;

            if (scene->rayIntersect(pathRay, its2) && its2.mesh->isEmitter())
            {

                value = its2.mesh->getEmitter()->getRadiance() * bsdf->getAlbedo() * INV_PI * its.shFrame.n.dot(point_global_hemisphere) / INV_TWOPI;
            }

            return value;
        }


    }

    // Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return "IntegratorDirectLighting[]";
    }

};
 
NORI_REGISTER_CLASS(IntegratorDirectLighting, "direct_lighting");
NORI_NAMESPACE_END