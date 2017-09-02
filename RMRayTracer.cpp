// RMRayTracer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

using Ray = ParametrizedLine<float, 3>;
using Colori = Vector3i;

namespace rm
{
	class IHitableObject;
}

using HitableObjectList = std::vector<rm::IHitableObject>;

struct Sphere
{
	Vector3f center;
	float radius;
};

struct HitResult
{
	float t;
	Vector3f hitPostion;
	bool isHit;
};

class Camera
{
public:
	Camera(const Vector2i& lScreenSize, const Vector2f& lScreenVirtualSize, const Vector3f& lCameraPosition, const Vector3f& lLeftRightCorner)
		: screenSize(lScreenSize)
		, screenVirtualSize(lScreenVirtualSize)
		, screenVirtualHalfSize(lScreenVirtualSize / 2.0f)
		, cameraPosition(lCameraPosition)
		, leftRightCorner(lLeftRightCorner)
	{

	}

	const Vector2i& getScreenSize() const { return screenSize; }
	const Vector2f& getScreenVirtualSize() const { return screenVirtualSize; }
	const Vector2f& getScreenVirtualHalfSize() const { return screenVirtualHalfSize; }
	const Vector3f& getCameraPosition() const { return cameraPosition; }
	const Vector3f& getLeftRightCorner() const { return leftRightCorner; }

private:
	Vector2i screenSize;
	Vector2f screenVirtualSize;
	Vector2f screenVirtualHalfSize;
	Vector3f cameraPosition;
	Vector3f leftRightCorner;
};

Colori getColor(Ray& ray)
{
	float sqrt = std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y());
	float lerpValue = std::min(1.0f, sqrt);
	int value = (int)(255.0f * (1.0f - lerpValue));
	return Colori(value, value, 255);
}

bool isHitSphere(Sphere& sphere, Ray& ray)
{
	const Vector3f& vectorA = ray.origin();
	const Vector3f& vectorB = ray.direction();
	const Vector3f& vectorC = sphere.center;

	float a = vectorB.dot(vectorB);
	float b = 2 * vectorB.dot(vectorA - vectorC);
	Vector3f vecDiffBetweenAAndC = vectorA - vectorC;
	float c = vecDiffBetweenAAndC.dot(vecDiffBetweenAAndC) - sphere.radius * sphere.radius;

	return (b * b - 4 * a * c) >= 0;
}

namespace rm
{
	class IHitableObject
	{
	public:
		virtual HitResult IsHit(const Ray& ray) const = 0;
	};

	class Sphere : public IHitableObject
	{
	public:
		Sphere(const Vector3f& lCenter, float lRadius)
			: center(lCenter)
			, radius(lRadius)
		{}

		virtual HitResult IsHit(const Ray& ray, float maxT, float minT) const
		{
			HitResult result = { 0.0f, Vector3f::Zero(), false };

			const Vector3f& vectorA = ray.origin();
			const Vector3f& vectorB = ray.direction();
			const Vector3f& vectorC = center;

			float a = vectorB.dot(vectorB);
			float b = 2 * vectorB.dot(vectorA - vectorC);
			Vector3f vecDiffBetweenAAndC = vectorA - vectorC;
			float c = vecDiffBetweenAAndC.dot(vecDiffBetweenAAndC) - radius * radius;

			float condition = b * b - 4 * a * c;
			if (condition > 0.0f)
			{
				float t = (-b - std::sqrt(condition)) / (2 * a);
				if (t < maxT && t >= minT)
				{
					result.t = t;
					//result.hitPostion
					result.isHit = true;
				}
				else
				{
					t = (-b + std::sqrt(condition)) / (2 * a);
					if (t < maxT && t >= minT)
					{
						result.t = t;
						result.isHit = true;
					}
				}
			}

			return result;
		}

	private:
		Vector3f center;
		float radius;
	};
}

void RayTrace(ofstream& fileHandle, const Camera& cam, const HitableObjectList& list)
{
	/*for (int y = cam.getScreenSize.y(); y >= 0; --y)
	{
		for (int x = 0; x < cam.getScreenSize.x(); ++x)
		{
			Vector3f offset((float)x / (float)cam.getScreenSize.x() * cam.getScreenVirtualSize.x(),
							(float)y / (float)cam.getScreenSize.y() * cam.getScreenVirtualSize.y(), 0.0f);
			offset = offset + cam.getLeftRightCorner;
			offset.normalize();
			Ray ray(cam.getCameraPosition(), offset);

			Colori color = getColor(ray);

			for (size_t i = 0; i < list.size(); ++i)
			{
				IHitableObject& obj = list[i];

				HitResult result = obj.IsHit(ray);
				if (result.isHit)
				{
					color = Colori(255, 0, 0);
				}
			}

			if (isHitSphere(sphere, ray))
			{
				color = Colori(255, 0, 0);
			}

			fileHandle << color.x() << " " << color.y() << " " << color.z() << "\n";
		}
	}*/
}

int main()
{
	ofstream ppmFile;
	ppmFile.open("buffer.ppm");

	Vector2i screenSize(200, 200);
	Vector2f screenVirtualSize(2, 2);
	Vector2f screenVirtualHalfSize(screenVirtualSize / 2.0f);
	Vector3f cameraPosition(0.0f, 0.0f, 1.0f);
	Vector3f leftRightCorner(-screenVirtualHalfSize.x(), -screenVirtualHalfSize.y(), -1.0f);

	Camera cam(screenSize, screenVirtualSize, cameraPosition, leftRightCorner);

	ppmFile << "P3\n" << screenSize.x() << " " << screenSize.y() << "\n255\n";

	Sphere sphere{ Vector3f(0.0, 0.0, -1.0), 1.0f };

	for (int y = screenSize.y(); y >= 0; --y)
	{
		for (int x = 0; x < screenSize.x(); ++x)
		{
			Vector3f offset((float)x / (float)screenSize.x() * screenVirtualSize.x(), 
							(float)y / (float)screenSize.y() * screenVirtualSize.y(), 
							0.0f);
			offset = offset + leftRightCorner;
			offset = offset - cameraPosition;
			offset.normalize();
			Ray ray(cameraPosition, offset);

			Colori color = getColor(ray);

			if (isHitSphere(sphere, ray))
			{
				color = Colori(255, 0, 0);
			}
			
			ppmFile << color.x() << " " << color.y() << " " << color.z() << "\n";
		}
	}
	
	ppmFile.close();

    return 0;
}

