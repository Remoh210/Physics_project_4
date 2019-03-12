#include "cSimpleSoftBody.h"
#include <algorithm>
namespace nPhysics 
{
	cSimpleSoftBody::cSimpleSoftBody(const sSoftBodyDef& def)
	{
		mNodes.resize(def.Nodes.size());
		for (size_t i = 0; i < def.Nodes.size(); i++)
		{
			mNodes[i] = new cNode(def.Nodes[i].Position, def.Nodes[i].Mass);
		}

		for (size_t i = 0; i < def.Springs.size(); i++)
		{
			cNode* nodeA = mNodes[def.Springs[i].first];
			cNode* nodeB = mNodes[def.Springs[i].second];
			if (!nodeA->HasNeighbor(nodeB))
			{
				cSpring* spring = new cSpring(nodeA, nodeB, def.SpringConstant);
				mSprings.push_back(spring);
				nodeA->Springs.push_back(spring);
				nodeB->Springs.push_back(spring);
			}

		}
	}

	cSimpleSoftBody::~cSimpleSoftBody()
	{
		for (size_t i = 0; i < mNodes.size(); i++)
		{
			delete mNodes[i];
		}
		mNodes.clear();
		for (size_t i = 0; i < mSprings.size(); i++)
		{
			delete mSprings[i];
		}
		mSprings.clear();
	}

	void cSimpleSoftBody::GetNodePostion(size_t index, glm::vec3 & positionOut)
	{
		positionOut = mNodes[index]->Position;
	}

	size_t cSimpleSoftBody::NumNodes()
	{
		return mNodes.size();
	}

	void cSimpleSoftBody::UpdateInternal(float dt, const glm::vec3& gravity)
	{
		for (size_t i = 0; i < mNodes.size(); i++)
		{
			mNodes[i]->SpringForce = gravity;
		}
		for (size_t i = 0; i < mSprings.size(); i++)
		{
			mSprings[i]->ApplyForceToNodes();
		}
		for (size_t i = 0; i < mNodes.size(); i++)
		{
			mNodes[i]->Intgrate(dt);
		}
	}

	cSimpleSoftBody::cSpring::cSpring(cNode * nodeA, cNode * nodeB, float springConstant)
		: NodeA(nodeA)
		, NodeB(nodeB)
		, SpringConstant(springConstant)
		, RestingDistance(glm::length(NodeA->Position - NodeB->Position))
	{
	}

	void cSimpleSoftBody::cSpring::ApplyForceToNodes()
	{
		glm::vec3 separationAtoB = NodeB->Position - NodeA->Position;
		float CurrentDistance = glm::length(separationAtoB);
		glm::vec3 force = (separationAtoB / CurrentDistance) * (-SpringConstant * (CurrentDistance - RestingDistance));
		NodeB->SpringForce += force;
		NodeA->SpringForce -= force;

	}

	cSimpleSoftBody::cNode::cNode(const glm::vec3 & position, float mass)
		: Position(position)
		, Velocity(0.f, 0.f, 0.f)
		, SpringForce(0.f, 0.f, 0.f)
		, Mass(mass)
	{
	}

	bool cSimpleSoftBody::cNode::HasNeighbor(cNode * other)
	{
		return Springs.end() != std::find_if(Springs.begin(), Springs.end(), [other, this](cSpring* spring){
			return other == spring->GetOther(this);
		});
		
	}

	void cSimpleSoftBody::cNode::Intgrate(float dt)
	{
		if(IsFixed())
		{
			return;
		}
		Velocity += SpringForce * (dt / Mass);
		Position += Velocity * dt;
		float dampingFactor = 0.2f;
		//Velocity *= 0.999f;
		Velocity *= glm::pow(1.f - dampingFactor, dt);
	}
}

