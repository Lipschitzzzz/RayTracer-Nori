#include <nori/integrator.h>
#include <nori/scene.h>
 
NORI_NAMESPACE_BEGIN
 
class NormalIntegrator : public Integrator
{
public:
    NormalIntegrator(const PropertyList &props)
    {
        // first time
        // m_myProperty = props.getString("myProperty");
        // std::cout << "Parameter value was : " << m_myProperty << std::endl;

        // for tracing rays
        
        /* No parameters this time */
    }
 
    // Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        // first time
        // return Color3f(0, 1, 0);

        // for tracing rays task
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }
        
        /* Return the component-wise absolute value of the shading normal as a color */
        // Normal3f n = its.shFrame.n.cwiseAbs();
        // return Color3f(n.x(), n.y(), n.z());
        
        Normal3f n = its.shFrame.n;
        float x = n.x() / 2 + 0.5;
        float y = n.y() / 2 + 0.5;
        float z = n.z() / 2 + 0.5;
        return Color3f(x, y, z);
    }
    // Return a human-readable description for debugging purposes
    std::string toString() const
    {
        // first time
        /*
        return tfm::format(
            "NormalIntegrator[\n"
            "  myProperty = \"%s\"\n"
            "]",
            m_myProperty
        );
        */

        // for tracing rays task
        return "NormalIntegrator[]";
    }
    
// first time
/*
protected:
    std::string m_myProperty;
*/

};
 
NORI_REGISTER_CLASS(NormalIntegrator, "normals");
NORI_NAMESPACE_END