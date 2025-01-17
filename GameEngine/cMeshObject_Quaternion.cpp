#include "cGameObject.h"

void cGameObject::setMeshOrientationEulerAngles(glm::vec3 newAnglesEuler, bool bIsDegrees /*=false*/)
{
	if (bIsDegrees)
	{
		newAnglesEuler = glm::vec3(glm::radians(newAnglesEuler.x),
			glm::radians(newAnglesEuler.y),
			glm::radians(newAnglesEuler.z));
	}

	this->m_meshQOrientation = glm::quat(glm::vec3(newAnglesEuler.x, newAnglesEuler.y, newAnglesEuler.z));
	return;
}

void cGameObject::setMeshOrientationEulerAngles(float x, float y, float z, bool bIsDegrees /*=false*/)
{
	return this->setMeshOrientationEulerAngles(glm::vec3(x, y, z), bIsDegrees);
}



glm::vec3 cGameObject::getMeshOrientationEulerAngles(bool bIsDegrees /*=false*/)
{
	glm::vec3 rot = glm::eulerAngles(this->m_meshQOrientation);
	if (bIsDegrees)
	{
		
		rot = glm::vec3(glm::degrees(rot.x),
			glm::degrees(rot.y),
			glm::degrees(rot.z));
		
	}
	return rot;

}


void cGameObject::adjMeshOrientationEulerAngles(glm::vec3 adjAngleEuler, bool bIsDegrees /*=false*/)
{
	if (bIsDegrees)
	{
		adjAngleEuler = glm::vec3(glm::radians(adjAngleEuler.x),
			glm::radians(adjAngleEuler.y),
			glm::radians(adjAngleEuler.z));
	}

	// Step 1: make a quaternion that represents the angle we want to rotate
	glm::quat rotationAdjust(adjAngleEuler);

	// Step 2: Multiply this quaternion by the existing quaternion. This "adds" the angle we want.
	this->m_meshQOrientation *= rotationAdjust;
	this->m_meshQOrientation = this->m_meshQOrientation * rotationAdjust;

	return;
}

void cGameObject::adjMeshOrientationEulerAngles(float x, float y, float z, bool bIsDegrees /*=false*/)
{
	return this->adjMeshOrientationEulerAngles(glm::vec3(x, y, z), bIsDegrees);
}

void cGameObject::adjMeshOrientationQ(glm::quat adjOrientQ)
{
	this->m_meshQOrientation *= adjOrientQ;
	return;
}

