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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample)
{
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample)
{
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tent(float x)
{
	return x < 0.5f ? sqrt(2.0f * x) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * x);
}

Point2f Warp::squareToTent(const Point2f &sample)
{
	/*
	Assignment 3: Complete this function.
	*/
	return Point2f(tent(sample.x()), tent(sample.y()));
}

float tentPdf(float t)
{
	return t >= -1 && t <= 1 ? 1 - abs(t) : 0;
}

float Warp::squareToTentPdf(const Point2f &p)
{
	/*
	Assignment 3: Complete this function.
	*/
	return tentPdf(p.x()) * tentPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f &sample)
{
	/*
	Assignment 3: Complete this function.
	*/
	float radius = std::sqrt(sample.x());
	float angle = sample.y() * (float)M_PI * 2;
	return Point2f(radius * cos(angle), radius * sin(angle));
}

float Warp::squareToUniformDiskPdf(const Point2f &p)
{
	/*
	Assignment 3: Complete this function.
	*/
	return std::sqrt(p.x() * p.x() + p.y() * p.y()) <= 1.0f ? INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample)
{
	/*
	Assignment 3: Complete this function.
	*/    
	float phi = sample.x() * M_PI * 2;
	float theta = acos(1 - 2 * sample.y());
	float sinTheta = sin(theta);
	float cosTheta = cos(theta);
	float sinPhi = sin(phi);
	float cosPhi = cos(phi);
	return Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v)
{
	/*
	Assignment 3: Complete this function.
	*/
	return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample)
{
    float cosTheta = sample.y();
    float sinTheta = std::sqrt(std::max((float) 0, 1-cosTheta*cosTheta));

    float sinPhi, cosPhi;
    sincosf(2.0f * M_PI * sample.x(), &sinPhi, &cosPhi);

    return Vector3f(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v)
{
	return v.z() < 0 ? 0 : INV_TWOPI;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample)
{
	/*

	Assignment 3: Complete this function.
	*/
	Point2f bottom = squareToUniformDisk(sample);
	float x = bottom.x();
	float y = bottom.y();
	return Vector3f(x, y, sqrt(1 - x * x - y * y));
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v)
{
	/*
	Assignment 3: Complete this function.
	*/
	return v.z() < 0 ? 0 : v.z() * INV_PI;
}

Vector3f Warp::squareToPhongSpecular(Point2f sample, float exponent)
{
	/*
	Assignment 3 (optional): Complete this function.
	*/
    throw NoriException("Warp::squareToPhongSpecular() is not yet implemented!");
}

float Warp::squareToPhongSpecularPdf(const Vector3f &v, float exponent)
{
	/*
	Assignment 3 (optional): Complete this function.
	*/
    throw NoriException("Warp::squareToPhongSpecularPdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha)
{
	/*
	Assignment 3: Complete this function.
	*/
	float phi = M_PI * 2 * sample.x();
	float theta = atan(sqrt(-alpha * alpha * log(1 - sample.y())));
	float cosPhi = cos(phi);
	float sinPhi = sin(phi);
	float cosTheta = cos(theta);
	float sinTheta = sin(theta);
	float x = sinTheta * cosPhi;
	float y = sinTheta * sinPhi;
	float z = cosTheta;
	return Vector3f(x, y, z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha)
{
	/*
	Assignment 3: Complete this function.
	*/
	if (m.z() <= 0) 
	{
		return 0;
	}
	float alpha2 = alpha * alpha;
	float cosTheta = m.z();
	float tanTheta2 = (m.x() * m.x() + m.y() * m.y()) / (cosTheta * cosTheta);
	float cosTheta3 = cosTheta * cosTheta * cosTheta;
	float azimuthal = INV_PI;
	float longitudinal = exp(-tanTheta2 / alpha2) / (alpha2 * cosTheta3);
	return azimuthal * longitudinal;
}

NORI_NAMESPACE_END
