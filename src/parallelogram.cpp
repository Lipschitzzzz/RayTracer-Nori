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


#include <nori/mesh.h>
#include <nori/timer.h>
#include <filesystem/resolver.h>
#include <unordered_map>
#include <fstream>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

/**
 * \brief Loader for Wavefront OBJ triangle meshes
 */
class ParallelogramMesh : public Mesh {
public:
    ParallelogramMesh(const PropertyList &propList) {
        std::string name = propList.getString("name");
        m_v0 = propList.getPoint("origin");
        m_u_vector = propList.getVector("u");
        m_v_vector = propList.getVector("v");

        Vector3f normal = m_u_vector.cross(m_v_vector);
        Point3f v1 = m_v0 + m_u_vector;
        Point3f v2 = m_v0 + m_v_vector;
        Point3f v3 = v1 + m_v_vector;


        m_V.resize(3, 4);
        m_V.col(0) = m_v0;
        m_V.col(1) = v1;
        m_V.col(2) = v2;
        m_V.col(3) = v3;

        m_N.resize(3, 4);
        m_N.col(0) = normal;
        m_N.col(1) = normal;
        m_N.col(2) = normal;
        m_N.col(3) = normal;

        m_UV.resize(2, 4);
        m_UV.col(0) = Vector2f(0, 0);
        m_UV.col(1) = Vector2f(1, 0);
        m_UV.col(2) = Vector2f(0, 1);
        m_UV.col(3) = Vector2f(1, 1);

        m_F.resize(3, 2);
        m_F.col(0) = TVector<unsigned int, 3>(0, 1, 2);
        m_F.col(1) = TVector<unsigned int, 3>(1, 3, 2);

        m_bbox.expandBy(m_v0);
        m_bbox.expandBy(v1);
        m_bbox.expandBy(v2);
        m_bbox.expandBy(v3);

        m_name = tfm::format(
                "Parallelogram[name = %s, \n"
                "  v0 = %s,\n"
                "  v1 = %f,\n"
                "  v2 = %s,\n"
                "  v3 = %s,\n"
                "  n  = %s,\n"
                "]",
                name,
                m_v0.toString(),
                v1.toString(),
                v2.toString(),
                v3.toString(),
                normal.toString()
            );
    }

private:
    Point3f m_v0;
    Vector3f m_u_vector;
    Vector3f m_v_vector;
};

NORI_REGISTER_CLASS(ParallelogramMesh, "parallelogram");
NORI_NAMESPACE_END
