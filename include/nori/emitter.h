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

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Data record for conveniently querying and sampling the
 * direct illumination technique implemented by a emitter
 */
struct EmitterQueryRecord {
    /// Pointer to the sampled emitter
    const Emitter *emitter;
    /// Reference position
    Point3f ref;
    /// Sampled position on the light source
    Point3f p;
    /// Associated surface normal
    Normal3f n;
    /// Solid angle density wrt. 'ref'
    float pdf;
    /// Direction vector from 'ref' to 'p'
    Vector3f d;
    /// Distance between 'ref' and 'p'
    float dist;

    Ray3f shadowRay;

    /// Create an unitialized query record
    EmitterQueryRecord() : emitter(nullptr) { }

    /// Create a new query record that can be used to sample a emitter
    EmitterQueryRecord(const Point3f &ref) : ref(ref) { }

    /**
     * \brief Create a query record that can be used to query the
     * sampling density after having intersected an area emitter
     */
    EmitterQueryRecord(const Emitter *emitter,
            const Point3f &ref, const Point3f &p,
            const Normal3f &n) : emitter(emitter), ref(ref), p(p), n(n) {
        //d = p - ref;
        //dist = d.norm();
        //d /= dist;
        d = (p - ref).normalized();
    }

    /**
     * \brief Create a query record that can be used to query the
     * sampling density after having intersected an environment emitter
     */
    EmitterQueryRecord(const Emitter *emitter, const Ray3f &ray) :
        emitter(emitter), ref(ray.o), p(ray(1)), n(-ray.d), d(ray.d),
        dist(std::numeric_limits<float>::infinity()) {
    }


    /// Return a human-readable string summary
    std::string toString() const;
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    Mesh* mesh;
    /**
     * \brief Direct illumination sampling: given a reference point in the
     * scene, sample an emitter position that contributes towards it
     * or fail.
     *
     * Given an arbitrary reference point in the scene, this method
     * samples a position on the emitter that has a nonzero contribution
     * towards that point. Note that this does not yet account for visibility.
     *
     * Ideally, the implementation should importance sample the product of
     * the emission profile and the geometry term between the reference point
     * and the position on the emitter.
     *
     * \param lRec
     *    A lumaire query record that specifies the reference point.
     *    After the function terminates, it will be populated with the
     *    position sample and related information
     *
     * \param sample
     *    A uniformly distributed 2D vector
     *
     * \return
     *    An importance weight associated with the sample. Includes
     *    any geometric terms between the emitter and the reference point.
     */
    virtual Color3f sample(const Mesh* mesh, EmitterQueryRecord& lRec, Sampler* sample) const = 0;

    /**
     * \brief Compute the sampling density of the direct illumination technique
     * implemented by \ref sample() with respect to the solid angle measure
     */
    virtual float pdf(const Mesh* mesh, const EmitterQueryRecord& lRec) const = 0;

    /// Evaluate the emitted radiance
    virtual Color3f eval(const EmitterQueryRecord &lRec) const = 0;

    virtual Color3f getRadiance() const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
};

inline std::string EmitterQueryRecord::toString() const {
    return tfm::format(
        "EmitterQueryRecord[\n"
        "  emitter = \"%s\",\n"
        "  ref = %s,\n"
        "  p = %s,\n"
        "  n = %s,\n"
        "  pdf = %f,\n"
        "  d = %s,\n"
        "  dist = %f\n"
        "]",
        indent(emitter->toString()),
        ref.toString(), p.toString(),
        n.toString(), pdf, d.toString(), dist
    );
}

NORI_NAMESPACE_END
