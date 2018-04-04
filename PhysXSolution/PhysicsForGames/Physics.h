#ifndef SOUND_PROGRAMMING_H_
#define SOUND_PROGRAMMING_H_

#include "Application.h"
#include "Camera.h"
#include "Render.h"

#include "ParticleEmitter.h"
#include "ParticleFluidEmitter.h"

#include <PxPhysicsAPI.h>
#include <PxScene.h>

#include <atomic>

using namespace physx;


enum RagDollParts
{
	NO_PARENT = -1,
	LOWER_SPINE,
	LEFT_PELVIS,
	RIGHT_PELVIS,
	LEFT_UPPER_LEG,
	RIGHT_UPPER_LEG,
	LEFT_LOWER_LEG,
	RIGHT_LOWER_LEG,
	UPPER_SPINE,
	LEFT_CLAVICLE,
	RIGHT_CLAVICLE,
	NECK,
	HEAD,
	LEFT_UPPER_ARM,
	RIGHT_UPPER_ARM,
	LEFT_LOWER_ARM,
	RIGHT_LOWER_ARM,
};

struct FilterGroup
{
	enum Enum
	{
		ePLAYER = (1 << 0),
		ePLATFORM = (1 << 1),
		eGROUND = (1 << 2)
	};
};

//create some constants for axis of rotation to make definition of quaternions a bit neater
const PxVec3 X_AXIS = PxVec3(1, 0, 0);
const PxVec3 Y_AXIS = PxVec3(0, 1, 0);
const PxVec3 Z_AXIS = PxVec3(0, 0, 1);

struct RagdollNode
{
	PxQuat globalRotation;
	PxVec3 scaledGobalPos;

	int parentNodeIdx;
	float halfLength;
	float radius;
	float parentLinkPos;
	float childLinkPos;
	char* name;
	PxArticulationLink* linkPtr;
	//constructor
	RagdollNode(PxQuat _globalRotation, int _parentNodeIdx, float _halfLength, float
		_radius, float _parentLinkPos, float _childLinkPos, char* _name)
	{
		globalRotation =
			_globalRotation, parentNodeIdx = _parentNodeIdx; halfLength = _halfLength; radius = _radius;
		parentLinkPos = _parentLinkPos; childLinkPos = _childLinkPos; name = _name;
	};
};


class MyControllerHitReport : public PxUserControllerHitReport
{
public:
	virtual void onShapeHit(const PxControllerShapeHit &hit);
	virtual void onControllerHit(const PxControllersHit &hit) {};
	virtual void onObstacleHit(const PxControllerObstacleHit &hit) {};

	MyControllerHitReport() :PxUserControllerHitReport() {};
	PxVec3 getPlayerContactNormal() { return _playerContactNormal; };
	void clearPlayerContactNormal() { _playerContactNormal = PxVec3(0, 0, 0); };
	PxVec3 _playerContactNormal;
};


class MyCollisionCallback : public PxSimulationEventCallback
{
public:
	std::atomic_int SafeTrigger;
	MyCollisionCallback() { SafeTrigger = 0; }

	virtual void onContact(const PxContactPairHeader& pairHeader,
		const PxContactPair* pairs, PxU32 nbPairs);
	virtual void onTrigger(PxTriggerPair* pairs, PxU32 nbPairs);
	virtual void onConstraintBreak(PxConstraintInfo*, PxU32) {};
	virtual void onWake(PxActor**, PxU32) {};
	virtual void onSleep(PxActor**, PxU32) {};
};



class Physics : public Application
{
public:
	virtual bool startup();
	virtual void shutdown();
    virtual bool update();
    virtual void draw();

	void renderGizmos(PxScene* physics_scene);
	
	PxScene* createDefaultScene();
	PxScene* m_physics_scene;
	
	void setUpPhysX();
	void Physics::setUpIntroductionToPhysX();

	float _characterYVelocity;
	float _characterRotation;
	float _playerGravity;

    Renderer* m_renderer;
    FlyCamera m_camera;
    float deltaTime;

	PxFoundation* m_physics_foundation;

	PxPhysics* m_physics;

	PxDefaultErrorCallback m_default_error_callback;
	PxDefaultAllocator m_default_allocator;
	PxSimulationFilterShader m_default_filter_shader;

	PxMaterial* m_physics_material;
	PxMaterial* m_box_material;
	PxCooking* m_physics_cooker;

	MyControllerHitReport* myHitReport;
	PxControllerManager* m_controller_manager;
	PxController* m_player_controller;

	MyCollisionCallback* myCollisionCallback;

	std::vector<PxRigidActor*> m_PhysXActors;

	//ParticleEmitter* m_particleEmitter;
	ParticleFluidEmitter* m_particleFluidEmitter;


	unsigned int m_shader;
	unsigned int m_texture;

	unsigned int m_clothIndexCount;
	unsigned int m_clothVertexCount;
	glm::vec3* m_clothPositions;

	unsigned int m_clothVAO, m_clothVBO, m_clothTextureVBO, m_clothIBO;


	PxArticulation* makeRagdoll(PxPhysics* g_Physics, RagdollNode** nodeArray,
		PxTransform worldPos, float scaleFactor, PxMaterial* ragdollMaterial);

	bool waterfall;
};


#endif //CAM_PROJ_H_
