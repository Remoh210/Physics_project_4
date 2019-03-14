#pragma once
#include "cGameObject.h"
#include "globalOpenGLStuff.h"
class cSimpleDebugRenderer
{
public:
	cSimpleDebugRenderer(cGameObject* mUnitSphere, cGameObject* mUnitCube, GLuint IDprogram);

	void drawSphere(glm::vec3 pos, float rad);
	void drawCube(glm::vec3 pos, float side);
	

private:
	cGameObject* mUnitSphere;
	cGameObject* mUnitPlane;
	cGameObject* mUnitCube;
	GLuint mShaderProgram;
};