#pragma once
#include "eShapeType.h"
#include <glm/glm.hpp>
namespace nPhysics
{
	class iShape
	{
	public: 
		virtual ~iShape() {};

		inline eShapeType GetShapeType() const { return mShapeType; }
		virtual bool GetAABB(const glm::mat4& transform, glm::vec3& minBoundsOut, glm::vec3& maxBoundsOut) { return false; } /*=0*/
		virtual bool GetSphereRadius(float& radiusOut) { return false; }
		virtual bool GetPlaneConstant(float& planeConstantOut) { return false; }
		virtual bool GetPlaneNormal(glm::vec3& planeNormalOut) { return false; }
	protected: 
		iShape(eShapeType shapeType) : mShapeType(shapeType){}
		iShape(const iShape& other){}
		iShape& operator=(const iShape& other) { return *this; }
	private:
		eShapeType mShapeType;
	};

}