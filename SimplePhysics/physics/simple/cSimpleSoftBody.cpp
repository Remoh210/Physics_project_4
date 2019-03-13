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
			for (size_t i = 0; i < def.Nodes.size(); i++)
			{
				mNodes[i]->ComputeRadius();
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

	void cSimpleSoftBody::GetNodeRadius(size_t index, float & nodeRadiusOut)
	{
		nodeRadiusOut = mNodes[index]->Radius;
	}

	size_t cSimpleSoftBody::NumNodes()
	{
		return mNodes.size();
	}

	void cSimpleSoftBody::GetAABB(glm::vec3 & minBoundsOut, glm::vec3 & maxBoundsOut)
	{
		minBoundsOut = mMinBounds;
		maxBoundsOut = mMaxBounds;
	}

	void cSimpleSoftBody::UpdateInternal(float dt, const glm::vec3& gravity)
	{
		for (size_t i = 0; i < mNodes.size(); i++)
		{
			mNodes[i]->SpringForce = gravity * mNodes[i]->Mass;
		}
		for (size_t i = 0; i < mSprings.size(); i++)
		{
			mSprings[i]->ApplyForceToNodes();
		}
		for (size_t i = 0; i < mNodes.size(); i++)
		{
			mNodes[i]->Intgrate(dt);
		}
		for (size_t idxA = 0; idxA < mNodes.size(); idxA++)
		{
			for (size_t idxB = 0; idxB < mNodes.size(); idxB++)
			{
				CollideNodes(mNodes[idxA], mNodes[idxB]);
			}
		}
		UpdateAABB();
	}

	void cSimpleSoftBody::CollideNodes(cNode * nodeA, cNode * nodeB)
	{
		if (nodeA->HasNeighbor(nodeB))
		{
			if (nodeA != nodeB) {
				if (!nodeA->IsFixed() && !nodeB->IsFixed())
				{
					float radiusA = nodeA->Radius;
					float radiusB = nodeB->Radius;

					float distance = glm::distance(nodeA->Position, nodeB->Position);
					if (distance <= radiusA + radiusB)
					{
						//nodeA->Position = nodeA->PreviousPosition;
						//nodeB->Position = nodeB->PreviousPosition;


						////////Collison response

						float massA = nodeA->Mass;
						float massB = nodeB->Mass;
						float totalMass = massA + massB;



						float fac1 = massA / totalMass;
						float fac2 = massB / totalMass;
						
						
						glm::vec3 x = nodeA->Position - nodeB->Position;
						x = glm::normalize(x);


						glm::vec3 v1 = nodeA->Velocity;
						float x1 = glm::dot(x, v1);
						glm::vec3 v1x = x * x1;
						glm::vec3 v1y = v1 - v1x;

						x = x * -1.0f;
						glm::vec3 v2 = nodeB->Velocity;
						float x2 = glm::dot(x, v2);
						glm::vec3 v2x = x * x2;
						glm::vec3 v2y = v2 - v2x;

						nodeA->Velocity = v1x * (massA - massB) / (massA + massB) + v2x * (2 * massB) / (massA + massB) + v1y;
						nodeB->Velocity = v1x * (2 * massA) / (massA + massB) + v2x * (massB - massA) / (massA + massB) + v2y;
						nodeA->Velocity = -nodeA->Velocity;
						nodeB->Velocity = -nodeB->Velocity;

						nodeA->Velocity *= 0.9999f;
						nodeB->Velocity *= 0.9999f;


					}
				}
			}
		}

	}

	void cSimpleSoftBody::UpdateAABB()
	{
		mMinBounds.x = std::numeric_limits<float>::max();
		mMinBounds.y = std::numeric_limits<float>::max();
		mMinBounds.z = std::numeric_limits<float>::max();

		mMaxBounds.x = std::numeric_limits<float>::min();
		mMaxBounds.y = std::numeric_limits<float>::min();
		mMaxBounds.z = std::numeric_limits<float>::min();

		for (size_t i = 0; i < mNodes.size(); i++)
		{
			cNode* node = mNodes[i];

			mMinBounds.x = glm::min(mMinBounds.x, node->Position.x - node->Radius);
			mMinBounds.y = glm::min(mMinBounds.y, node->Position.y - node->Radius);
			mMinBounds.z = glm::min(mMinBounds.z, node->Position.z - node->Radius);

			mMaxBounds.x = glm::max(mMaxBounds.x, node->Position.x + node->Radius);
			mMaxBounds.y = glm::max(mMaxBounds.y, node->Position.y + node->Radius);
			mMaxBounds.z = glm::max(mMaxBounds.z, node->Position.z + node->Radius);

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
		, PreviousPosition(position)
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
		PreviousPosition = Position;
		Velocity += SpringForce * (dt / Mass);
		Position += Velocity * dt;
		float dampingFactor = 0.9f;
		//Velocity *= 0.999f;
		Velocity *= glm::pow(1.f - dampingFactor, dt);
	}
	void cSimpleSoftBody::cNode::ComputeRadius()
	{
		float closestNeighbor = std::numeric_limits<float>::max();
		for (size_t i = 0; i < Springs.size(); i++)
		{
			float distance = glm::length(Springs[i]->GetOther(this)->Position - Position);
			closestNeighbor = glm::min(closestNeighbor, distance);
		}
		
		Radius = 0.5f * closestNeighbor;
	}
}

