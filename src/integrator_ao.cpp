#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <math.h>

NORI_NAMESPACE_BEGIN
 
class IntegratorAO : public Integrator
{
public:
    IntegratorAO(const PropertyList &props)
    {

    }
 
    // Compute the radiance value for a given ray.
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        float white_diffuse_brdf = 1.f / M_PI;
        float uniform_2d_pdf = 1.f / (2.f * M_PI);
        float visiblity = 0.f;

        Intersection its;

        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }
        
        Frame frame = its.shFrame;

        // Randomly generate point x,y
        Point2f point_2d_square = sampler->next2D();

        // transform to sphere axis(Hemisphere)
        Vector3f point_unit_hemisphere = Warp::squareToUniformHemisphere(point_2d_square);

        // unit(local) to world(global)
        Vector3f point_global_hemisphere = frame.toWorld(point_unit_hemisphere);

        Ray3f shadowRay = Ray3f(its.p + Epsilon * its.shFrame.n, point_global_hemisphere);

        if (!scene->rayIntersect(shadowRay))
        {
            visiblity = 1.f;
        }
        Color3f value = (visiblity * white_diffuse_brdf * its.shFrame.n.dot(point_global_hemisphere)) / uniform_2d_pdf;
        return value;
    }

    // Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return "IntegratorAO[]";
    }

};
 
NORI_REGISTER_CLASS(IntegratorAO, "ao");
NORI_NAMESPACE_END