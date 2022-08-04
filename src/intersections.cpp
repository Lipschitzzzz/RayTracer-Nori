#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class IntersectionsIntegrator : public Integrator {

private:

	unsigned int m_maxTests;

public:
	IntersectionsIntegrator(const PropertyList& props) {
		/* No parameters this time */
		m_maxTests = props.getInteger("maxTests");
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		/* Find the surface that is visible in the requested direction */
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		float intensity = its.attempts / ((float)m_maxTests);
		/* Return the component-wise absolute
		   value of the shading normal as a color */
		return Color3f(intensity, 0, std::max(0.f, 1.0f - intensity));
	}

	std::string toString() const {
        return tfm::format(
                "IntersectionsIntegrator[\n"
				"maxTests = %d\n"
				"]", m_maxTests);
	}
};

NORI_REGISTER_CLASS(IntersectionsIntegrator, "intersections");
NORI_NAMESPACE_END
