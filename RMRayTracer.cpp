// RMRayTracer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

using Ray = ParametrizedLine<float, 3>;
using Colori = Vector3i;
using Colorf = Vector3f;

namespace rm
{
	class HitResult
	{
	public:
		HitResult()
			: t(0.0f)
			, hitPostion(Vector3f::Zero())
			, isHit(false)
		{}

		void Reset()
		{
			t = 0.0f;
			hitPostion = Vector3f::Zero();
			isHit = false;
		}

		float t;
		Vector3f hitPostion;
		bool isHit;
	};

	class Camera
	{
	public:
		Camera(const Vector2i& lScreenSize, const Vector2f& lScreenVirtualSize, const Vector3f& lCameraPosition, const Vector3f& lLeftRightCorner, int lSampleCount)
			: screenSize(lScreenSize)
			, screenVirtualSize(lScreenVirtualSize)
			, screenVirtualHalfSize(lScreenVirtualSize / 2.0f)
			, cameraPosition(lCameraPosition)
			, leftRightCorner(lLeftRightCorner)
			, sampleCount(lSampleCount)
		{

		}

		const Vector2i& GetScreenSize() const { return screenSize; }
		const Vector2f& GetScreenVirtualSize() const { return screenVirtualSize; }
		const Vector2f& GetScreenVirtualHalfSize() const { return screenVirtualHalfSize; }
		const Vector3f& GetCameraPosition() const { return cameraPosition; }
		const Vector3f& GetLeftRightCorner() const { return leftRightCorner; }
		int GetSampleCount() const { return sampleCount; }

	private:
		Vector2i screenSize;
		Vector2f screenVirtualSize;
		Vector2f screenVirtualHalfSize;
		Vector3f cameraPosition;
		Vector3f leftRightCorner;
		int sampleCount;
	};

	class IHitableObject
	{
	public:
		virtual bool		IsHit(const Ray& ray, float minT, float maxT, HitResult& hitResult) const = 0;
		virtual Vector3f	GetNormal(const Vector3f& surfacePosition) const = 0;
		virtual Colorf		GetColor() const = 0;
	};

	class Sphere : public IHitableObject
	{
	public:
		Sphere(const Vector3f& lCenter, float lRadius, const Colorf lColor)
			: center(lCenter)
			, radius(lRadius)
			, color(lColor)
		{}

		virtual bool IsHit(const Ray& ray, float minT, float maxT, HitResult& hitResult) const
		{
			hitResult.Reset();

			const Vector3f& vectorA = ray.origin();
			const Vector3f& vectorB = ray.direction();
			const Vector3f& vectorC = center;

			float a = vectorB.dot(vectorB);
			Vector3f vectorAC = vectorA - vectorC;
			float b = 2 * vectorB.dot(vectorAC);
			float c = vectorAC.dot(vectorAC) - radius * radius;

			float discriminant = b * b - 4 * a * c;

			if (discriminant > 0.0f)
			{
				float t1 = (-b + std::sqrt(discriminant)) / (2.0f * a);
				float t2 = (-b - std::sqrt(discriminant)) / (2.0f * a);
				
				float t = std::min(t1, t2);

				if (t >= minT && t < maxT)
				{
					hitResult.t = t;
					hitResult.hitPostion = ray.pointAt(t);
					hitResult.isHit = true;
				}
			}

			return hitResult.isHit;
		}

		virtual Vector3f GetNormal(const Vector3f& surfacePosition) const
		{
			Vector3f normal = surfacePosition - center;
			normal.normalize();
			return normal;
		}

		virtual Colorf GetColor() const
		{
			return color;
		}

	private:
		Vector3f center;
		float radius;
		Colorf color;
	};

	class Scene
	{
	public:
		Scene()
		{
			InitScene();
		}

		~Scene()
		{
			DestoryScene();
		}

		void AddHitableObject(IHitableObject* obj)
		{
			objectList.push_back(obj);
		}

		void RayTrace(ofstream& fileHandle, const Camera& cam);

	protected:
		void InitScene()
		{

		}

		void DestoryScene()
		{
			for each (auto obj in objectList)
			{
				delete obj;
			}

			objectList.clear();
		}

	private:
		Colori GetBackgroundColor(Ray& ray)
		{
			float sqrt = std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y());
			float lerpValue = std::min(1.0f, sqrt);
			int value = (int)(255.0f * (1.0f - lerpValue));
			return Colori(value, value, 255);
		}

		using HitableObjectList = std::vector<rm::IHitableObject*>;

		HitableObjectList objectList;
	};

	void Scene::RayTrace(ofstream& fileHandle, const Camera& cam)
	{
		std::random_device rd; 
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> dis(0.0f, 1.0f);

		for (int y = cam.GetScreenSize().y(); y >= 0; --y)
		{
			for (int x = 0; x < cam.GetScreenSize().x(); ++x)
			{
				Colori color(0, 0, 0);

				for (int index = 0; index < cam.GetSampleCount(); ++index)
				{
					float sampleX = float(x) + dis(gen);
					float sampleY = float(y) + dis(gen);
					
					Vector3f offset(sampleX / (float)cam.GetScreenSize().x() * cam.GetScreenVirtualSize().x(),
						sampleY / (float)cam.GetScreenSize().y() * cam.GetScreenVirtualSize().y(), 0.0f);
					offset = offset + cam.GetLeftRightCorner();
					offset.normalize();

					Ray ray(cam.GetCameraPosition(), offset);

					HitResult hitResult;
					float minT = 0.0f;
					float maxT = FLT_MAX;
					bool atLeastHitOnce = false;

					for each (auto obj in objectList)
					{
						if (obj->IsHit(ray, minT, maxT, hitResult))
						{
							atLeastHitOnce = true;
							maxT = hitResult.t;
							Vector3f normal = obj->GetNormal(hitResult.hitPostion);
							float blend = 1.0f - (1.0f + ray.direction().dot(normal)) / 2.0f;
							Colorf objColor = blend * 256.0f * obj->GetColor();
							color += Colori((int)objColor.x(), (int)objColor.y(), (int)objColor.z());
						}
					}

					if (!atLeastHitOnce)
					{
						color += GetBackgroundColor(ray);
					}
				}

				color /= cam.GetSampleCount();

				fileHandle << color.x() << " " << color.y() << " " << color.z() << "\n";
			}
		}
	} // end of Scene::RayTrace

} // end of namespace

int main()
{
	ofstream ppmFile;
	ppmFile.open("buffer.ppm");

	Vector2i screenSize(200, 200);
	Vector2f screenVirtualSize(2, 2);
	Vector2f screenVirtualHalfSize(screenVirtualSize / 2.0f);
	Vector3f cameraPosition(0.0f, 0.0f, 1.0f);
	Vector3f leftRightCorner(-screenVirtualHalfSize.x(), -screenVirtualHalfSize.y(), -1.0f);

	rm::Camera cam(screenSize, screenVirtualSize, cameraPosition, leftRightCorner, 8);

	ppmFile << "P3\n" << screenSize.x() << " " << screenSize.y() << "\n255\n";

	rm::Sphere* sphere1 = new rm::Sphere { Vector3f(0.0f, 0.0f, -1.0f), 0.8f, Colorf(1.0f, 0.0f, 0.0f) };
	rm::Sphere* sphere2 = new rm::Sphere { Vector3f(0.0f, 0.0f, -2.5f), 1.8f, Colorf(0.0f, 0.0f, 1.0f) };

	rm::Scene scene;
	scene.AddHitableObject(sphere1);
	scene.AddHitableObject(sphere2);
	scene.RayTrace(ppmFile, cam);

	ppmFile.close();

    return 0;
}

