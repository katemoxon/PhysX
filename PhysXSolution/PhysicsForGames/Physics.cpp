#include "Physics.h"

#include "gl_core_4_4.h"
#include "GLFW/glfw3.h"
#include "Gizmos.h"

#include "glm/ext.hpp"
#include "glm/gtc/quaternion.hpp"

#include <iostream>

#define Assert(val) if (val){}else{ *((char*)0) = 0;}
#define ArrayCount(val) (sizeof(val)/sizeof(val[0]))



RagdollNode* ragdollData[] =
{
	new RagdollNode(PxQuat(PxPi / 2.0f, Z_AXIS), NO_PARENT,1,3,1,1,"lower spine"),
	new RagdollNode(PxQuat(PxPi, Z_AXIS), LOWER_SPINE, 1,1,-1,1,"left pelvis"),
	new RagdollNode(PxQuat(0, Z_AXIS), LOWER_SPINE, 1,1,-1, 1,"right pelvis"),
	new RagdollNode(PxQuat(PxPi / 2.0f + 0.2f, Z_AXIS),LEFT_PELVIS,5,2,-1,1,"L upper leg"),
	new RagdollNode(PxQuat(PxPi / 2.0f - 0.2f, Z_AXIS),RIGHT_PELVIS,5,2,-1,1,"R upper leg"),
	new RagdollNode(PxQuat(PxPi / 2.0f + 0.2f, Z_AXIS),LEFT_UPPER_LEG,5,1.75,-1,1,"L lower leg"),
	new RagdollNode(PxQuat(PxPi / 2.0f - 0.2f, Z_AXIS),RIGHT_UPPER_LEG,5,1.75,-1,1,"R lowerleg"),
	new RagdollNode(PxQuat(PxPi / 2.0f, Z_AXIS), LOWER_SPINE, 1, 3, 1, -1, "upper spine"),
	new RagdollNode(PxQuat(PxPi, Z_AXIS), UPPER_SPINE, 1, 1.5, 1, 1, "left clavicle"),
	new RagdollNode(PxQuat(0, Z_AXIS), UPPER_SPINE, 1, 1.5, 1, 1, "right clavicle"),
	new RagdollNode(PxQuat(PxPi / 2.0f, Z_AXIS), UPPER_SPINE, 1, 1, 1, -1, "neck"),
	new RagdollNode(PxQuat(PxPi / 2.0f, Z_AXIS), NECK, 1, 3, 1, -1, "HEAD"),
	new RagdollNode(PxQuat(PxPi - .3, Z_AXIS), LEFT_CLAVICLE, 3, 1.5, -1, 1, "left upper arm"),
	new RagdollNode(PxQuat(0.3, Z_AXIS), RIGHT_CLAVICLE, 3, 1.5, -1, 1, "right upper arm"),
	new RagdollNode(PxQuat(PxPi - .3, Z_AXIS), LEFT_UPPER_ARM, 3, 1, -1, 1, "left lower arm"),
	new RagdollNode(PxQuat(0.3, Z_AXIS), RIGHT_UPPER_ARM, 3, 1, -1, 1, "right lower arm"),
	NULL
};


PxArticulation* Physics::makeRagdoll(PxPhysics* g_Physics, RagdollNode** nodeArray,
	PxTransform worldPos, float scaleFactor, PxMaterial* ragdollMaterial)
{
	//create the articulation for our ragdoll
	PxArticulation *articulation = g_Physics->createArticulation();
	RagdollNode** currentNode = nodeArray;
	//while there are more nodes to process...
	while (*currentNode != NULL)
	{
		//get a pointer to the current node
		RagdollNode* currentNodePtr = *currentNode;
		//create a pointer ready to hold the parent node pointer if there is one
		RagdollNode* parentNode = nullptr;
		//get scaled values for capsule
		float radius = currentNodePtr->radius * scaleFactor;
		float halfLength = currentNodePtr->halfLength * scaleFactor;
		float childHalfLength = radius + halfLength;
		float parentHalfLength = 0; //will be set later if there is a parent
									//get a pointer to the parent
		PxArticulationLink* parentLinkPtr = NULL;
		currentNodePtr->scaledGobalPos = worldPos.p;

		if (currentNodePtr->parentNodeIdx != -1)
		{
			//if there is a parent then we need to work out our local position for the link
			//get a pointer to the parent node
			parentNode = *(nodeArray + currentNodePtr->parentNodeIdx);
			//get a pointer to the link for the parent
			parentLinkPtr = parentNode->linkPtr;
			parentHalfLength = (parentNode->radius + parentNode->halfLength) *scaleFactor;
			//work out the local position of the node
			PxVec3 currentRelative = currentNodePtr->childLinkPos * currentNodePtr->globalRotation.rotate(PxVec3(childHalfLength, 0, 0));
			PxVec3 parentRelative = -currentNodePtr->parentLinkPos * parentNode->globalRotation.rotate(PxVec3(parentHalfLength, 0, 0));
			currentNodePtr->scaledGobalPos = parentNode->scaledGobalPos - (parentRelative +
				currentRelative);
		}

		//build the transform for the link
		PxTransform linkTransform = PxTransform(currentNodePtr->scaledGobalPos, currentNodePtr->globalRotation);
		//create the link in the articulation
		PxArticulationLink* link = articulation->createLink(parentLinkPtr, linkTransform);
		//add the pointer to this link into the ragdoll data so we have it for later when we want to link to it
		currentNodePtr->linkPtr = link;
		float jointSpace = .015f; //gap between joints
		float capsuleHalfLength = (halfLength > jointSpace ? halfLength - jointSpace : 0) + .01f;
		PxCapsuleGeometry capsule(radius, capsuleHalfLength);
		link->createShape(capsule, *ragdollMaterial); //adds a capsule collider to the link
		PxRigidBodyExt::updateMassAndInertia(*link, 50.0f); //adds some mass, mass should really be part of the data!

		if (currentNodePtr->parentNodeIdx != -1)
		{
			//get the pointer to the joint from the link
			PxArticulationJoint *joint = link->getInboundJoint();
			//get the relative rotation of this link
			PxQuat frameRotation = parentNode->globalRotation.getConjugate() *
				currentNodePtr->globalRotation;
			//set the parent contraint frame
			PxTransform parentConstraintFrame = PxTransform(PxVec3(currentNodePtr->parentLinkPos * parentHalfLength, 0, 0), frameRotation);
			//set the child constraint frame (this the constraint frame of the newly added link)
			PxTransform thisConstraintFrame = PxTransform(PxVec3(currentNodePtr->childLinkPos * childHalfLength, 0, 0));
			//set up the poses for the joint so it is in the correct place
			joint->setParentPose(parentConstraintFrame);
			joint->setChildPose(thisConstraintFrame);
			//set up some constraints to stop it flopping around
			joint->setStiffness(20);
			joint->setDamping(20);
			joint->setSwingLimit(0.4f, 0.4f);
			joint->setSwingLimitEnabled(true);
			joint->setTwistLimit(-0.1f, 0.1f);
			joint->setTwistLimitEnabled(true);
		}
		currentNode++;
	}
	return articulation;
}


void MyControllerHitReport::onShapeHit(const PxControllerShapeHit &hit)
{
	//gets a reference to a structure which tells us what has been
	//hit and where the acter from the shape we hit
	PxRigidActor* actor = hit.shape->getActor();

	//get the normal of the thing we hit and store it so the player
	//controller can respond correctly
	_playerContactNormal = hit.worldNormal;

	//try to cast to a dynamic actor
	PxRigidDynamic* myActor = actor->is<PxRigidDynamic>();
	if (myActor)
	{
		//this is where we can apply forces to things we hit
	}
}


void MyCollisionCallback::onContact(const PxContactPairHeader& pairHeader, const
	PxContactPair* pairs, PxU32 nbPairs)
{
	for (PxU32 i = 0; i < nbPairs; i++)
	{
		const PxContactPair& cp = pairs[i];
		//only interested in touches found and lost
		if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
		{
			cout << "Collision Detected between: ";
			cout << pairHeader.actors[0]->getName();
			cout << pairHeader.actors[1]->getName() << endl;
		}
	}
}

void MyCollisionCallback::onTrigger(PxTriggerPair* pairs, PxU32 nbPairs)
{
	if (SafeTrigger == 0)
	{
		SafeTrigger = 1;
	}
	else
	{
		SafeTrigger = 0;
	}
	

	for (PxU32 i = 0; i < nbPairs; i++)
	{
		PxTriggerPair* pair = pairs + i;
		PxActor* triggerActor = pair->triggerActor;
		PxActor* otherActor = pair->otherActor;
	}
};

PxFilterFlags myFliterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock,
	PxU32 constantBlockSize)
{
	// let triggers through
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}

	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where
	// the filtermask of A contains the ID of B and vice versa.
	if ((filterData0.word0 & filterData1.word1) &&
		(filterData1.word0 & filterData0.word1))
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND |
		PxPairFlag::eNOTIFY_TOUCH_LOST;

	return PxFilterFlag::eDEFAULT;
}


void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask)
{
	PxFilterData filterData;
	filterData.word0 = filterGroup; // word0 = own ID
	filterData.word1 = filterMask; // word1 = ID mask to filter pairs that
								   //trigger a contact callback;
	const PxU32 numShapes = actor->getNbShapes();
	PxShape** shapes = (PxShape**)_aligned_malloc(sizeof(PxShape*)*numShapes, 16);
	actor->getShapes(shapes, numShapes);
	for (PxU32 i = 0; i < numShapes; i++)
	{
		PxShape* shape = shapes[i];
		shape->setSimulationFilterData(filterData);
	}
	_aligned_free(shapes);
}

void setShapeAsTrigger(PxRigidActor* actorIn)
{
	PxRigidStatic* staticActor = actorIn->is<PxRigidStatic>();
	assert(staticActor);
	const PxU32 numShapes = staticActor->getNbShapes();
	PxShape** shapes = (PxShape**)_aligned_malloc(sizeof(PxShape*)*numShapes, 16);
	staticActor->getShapes(shapes, numShapes);
	for (PxU32 i = 0; i < numShapes; i++)
	{
		shapes[i]->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
		shapes[i]->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
	}
}

bool Physics::startup()
{
	if (Application::startup() == false)
	{
		return false;
	}

	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	Gizmos::create();

	m_camera = FlyCamera(1280.0f / 720.0f, 10.0f);
	m_camera.setLookAt(vec3(15, 10, 5), vec3(0), vec3(0, 1, 0));
	m_camera.sensitivity = 3;

	m_renderer = new Renderer();

	setUpPhysX();
	setUpIntroductionToPhysX();

	waterfall = false;

	myHitReport = new MyControllerHitReport();
	m_controller_manager = PxCreateControllerManager(*m_physics_scene);
	//describe our controller...
	PxCapsuleControllerDesc desc;
	desc.height = 1.6f;
	desc.radius = 0.4f;
	desc.position.set(10, 0, 10);
	desc.material = m_physics_material; //playerPhysicsMaterial?
	desc.reportCallback = myHitReport; //connect it to our collision detection routine
	desc.density = 10;
	//create the layer controller
	m_player_controller = m_controller_manager->createController(desc);
	m_player_controller->setPosition(PxExtendedVec3(5, 0, 5));
	//set up some variables to control our player with
	_characterYVelocity = 0; //initialize character velocity
	_characterRotation = 0; //and rotation
	_playerGravity = -9.8f; //set up the player gravity
	myHitReport->clearPlayerContactNormal(); //initialize the contact normal (what we
												//are in contact with)
	//m_physics_scene->addActor(*m_player_controller->getActor()); //so we can draw its gizmo




	PxArticulation* ragDollArticulation;
	ragDollArticulation = makeRagdoll(m_physics, ragdollData, PxTransform(PxVec3(10, 1, 0)), .1f, m_physics_material);
	m_physics_scene->addArticulation(*ragDollArticulation);

	myCollisionCallback = new MyCollisionCallback();
	m_physics_scene->setSimulationEventCallback(myCollisionCallback);

	PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f), PxQuat(PxHalfPi,
		PxVec3(0.0f, 0.0f, 1.0f)));

	PxRigidStatic* plane = PxCreateStatic(*m_physics, pose, PxPlaneGeometry(),
		*m_physics_material);

	const PxU32 numShapes = plane->getNbShapes();
	m_physics_scene->addActor(*plane);

	//~~~~~centre box~~~~~//
	PxBoxGeometry side1(4.5, 1, .5);
	PxBoxGeometry side2(.5, 1, 4.5);
	pose = PxTransform(PxVec3(0.0f, 0.5, 4.0f));
	PxRigidStatic* box = PxCreateStatic(*m_physics, pose, side1,
		*m_physics_material);

	m_physics_scene->addActor(*box);
	m_PhysXActors.push_back(box);

	pose = PxTransform(PxVec3(0.0f, 0.5, -4.0f));
	box = PxCreateStatic(*m_physics, pose, side1, *m_physics_material);
	m_physics_scene->addActor(*box);
	m_PhysXActors.push_back(box);

	pose = PxTransform(PxVec3(4.0f, 0.5, 0));
	box = PxCreateStatic(*m_physics, pose, side2, *m_physics_material);
	m_physics_scene->addActor(*box);
	m_PhysXActors.push_back(box);

	pose = PxTransform(PxVec3(-4.0f, 0.5, 0));
	box = PxCreateStatic(*m_physics, pose, side2, *m_physics_material);
	m_physics_scene->addActor(*box);
	m_PhysXActors.push_back(box);


	//~~~~~trigger box~~~~~//
	PxBoxGeometry triggerBox_side(1, 0.1f, 1);
	pose = PxTransform(PxVec3(-9.0f, 1, -9.0f));
	PxRigidStatic* triggerBox = PxCreateStatic(*m_physics, pose, triggerBox_side,
		*m_physics_material);

	m_physics_scene->addActor(*triggerBox);
	m_PhysXActors.push_back(triggerBox);

	setShapeAsTrigger(triggerBox);
	//~~~~~trigger box~~~~~//

	setupFiltering(m_player_controller->getActor(), FilterGroup::ePLAYER, FilterGroup::ePLATFORM);
	setupFiltering(triggerBox, FilterGroup::ePLATFORM, FilterGroup::ePLAYER);



	//PxParticleSystem* pf;
	//// create particle system in PhysX SDK
	//// set immutable properties.
	//PxU32 maxParticles = 4000;
	//bool perParticleRestOffset = false;
	//pf = m_physics->createParticleSystem(maxParticles, perParticleRestOffset);
	//
	//pf->setDamping(0.1);
	//pf->setParticleMass(.1);
	//pf->setRestitution(0);
	//pf->setParticleBaseFlag(PxParticleBaseFlag::eCOLLISION_TWOWAY, true);
	//
	//if (pf)
	//{
	//	m_physics_scene->addActor(*pf);
	//	m_particleEmitter = new ParticleEmitter(maxParticles,
	//		PxVec3(0, 10, 0), pf, .01);
	//	m_particleEmitter->setStartVelocityRange(-2.0f, 0, -2.0f,
	//		2.0f, 0.0f, 2.0f);
	//}


	/*PARTICLE EMITTER*/
	//create our particle system
	PxParticleFluid* pf;
	// create particle system in PhysX SDK
	// set immutable properties.
	PxU32 maxParticles = 4000;
	bool perParticleRestOffset = false;
	pf = m_physics->createParticleFluid(maxParticles, perParticleRestOffset);

	pf->setRestParticleDistance(.3f);
	pf->setDynamicFriction(0.1);
	pf->setStaticFriction(0.1);
	pf->setDamping(0.1);
	pf->setParticleMass(.1);
	pf->setRestitution(0);
	pf->setParticleReadDataFlag(PxParticleReadDataFlag::eDENSITY_BUFFER, true);
	pf->setParticleBaseFlag(PxParticleBaseFlag::eCOLLISION_TWOWAY, true);
	pf->setStiffness(100);

	if (pf)
	{
		m_physics_scene->addActor(*pf);
		m_particleFluidEmitter = new ParticleFluidEmitter(maxParticles,
			PxVec3(0, 10, 0), pf, .1);
		m_particleFluidEmitter->setStartVelocityRange(-0.001f, -250.0f, -0.001f,
			0.001f, -250.0f, 0.001f);
	}
	/*PARTICLE EMITTER*/

	
	return true;
}

void Physics::shutdown()
{
	delete m_renderer;
	Gizmos::destroy();
	Application::shutdown();
}

bool Physics::update()
{
	if (Application::update() == false)
	{
		return false;
	}

	Gizmos::clear();

	float dt = (float)glfwGetTime();
	deltaTime = dt;
	glfwSetTime(0.0);

	if (dt > 0)
	{
		m_physics_scene->simulate(dt > 0.033f ? 0.033f : dt);
		while (m_physics_scene->fetchResults() == false);
	}

	vec4 white(1);
	vec4 black(0, 0, 0, 1);

	for (int i = 0; i <= 20; ++i)
	{
		Gizmos::addLine(vec3(-10 + i, -0.01, -10), vec3(-10 + i, -0.01, 10),
			i == 10 ? white : black);
		Gizmos::addLine(vec3(-10, -0.01, -10 + i), vec3(10, -0.01, -10 + i),
			i == 10 ? white : black);
	}

	m_camera.update(1.0f / 60.0f);

	//if (m_particleEmitter)
	//{
	//	m_particleEmitter->update(deltaTime);
	//	//render all our particles
	//	m_particleEmitter->renderParticles();
	//}

	if (myCollisionCallback->SafeTrigger == 1)
	{
		waterfall = true;
	}
	else
	{
		waterfall = false;
	}
	
	if (m_particleFluidEmitter)
	{
		if (waterfall)
			m_particleFluidEmitter->update(deltaTime);
			//render all our particles
		m_particleFluidEmitter->renderParticles();
	}

	bool onGround; //set to true if we are on the ground
	float movementSpeed = 10.0f; //forward and back movement speed
	float rotationSpeed = 1.0f; //turn speed
	//check if we have a contact normal. if y is greater than .3 we assume this is
	//solid ground.This is a rather primitive way to do this.Can you do better ?

	if (myHitReport->getPlayerContactNormal().y > 0.3f)
	{
		_characterYVelocity = -0.1f;
		onGround = true;
	}
	else
	{
		_characterYVelocity += _playerGravity * deltaTime;
		onGround = false;
	}
	myHitReport->clearPlayerContactNormal();
	const PxVec3 up(0, 1, 0);
	//scan the keys and set up our intended velocity based on a global transform
	PxVec3 velocity(0, _characterYVelocity, 0);
	if (glfwGetKey(m_window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		velocity.x -= movementSpeed * deltaTime;
	}
	if (glfwGetKey(m_window, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		velocity.x += movementSpeed * deltaTime;
	}
	if (glfwGetKey(m_window, GLFW_KEY_LEFT) == GLFW_PRESS)
	{
		velocity.z += movementSpeed * deltaTime;
	}
	if (glfwGetKey(m_window, GLFW_KEY_RIGHT) == GLFW_PRESS)
	{
		velocity.z -= movementSpeed * deltaTime;
	}
	if (glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS)
	{
		velocity.y += movementSpeed * deltaTime * 5;
	}
	//To do.. add code to control z movement and jumping
	float minDistance = 0.001f;
	PxControllerFilters filter;
	//make controls relative to player facing
	PxQuat rotation(_characterRotation, PxVec3(0, 1, 0));
	//move the controller
	m_player_controller->move(rotation.rotate(velocity), minDistance, deltaTime,
		filter);

	return true;
}

void Physics::draw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_CULL_FACE);


	//m_renderer->RenderAndClear(m_camera.view_proj);
	renderGizmos(m_physics_scene);

	Gizmos::draw(m_camera.proj, m_camera.view);

	glfwSwapBuffers(m_window);
	glfwPollEvents();
}

void AddWidget(PxShape* shape, PxRigidActor* actor, vec4 geo_color)
{
	PxTransform full_transform = PxShapeExt::getGlobalPose(*shape, *actor);
	vec3 actor_position(full_transform.p.x, full_transform.p.y, full_transform.p.z);
	glm::quat actor_rotation(full_transform.q.w,
		full_transform.q.x,
		full_transform.q.y,
		full_transform.q.z);
	glm::mat4 rot(actor_rotation);

	mat4 rotate_matrix = glm::rotate(10.f, glm::vec3(7, 7, 7));

	PxGeometryType::Enum geo_type = shape->getGeometryType();

	switch (geo_type)
	{
	case (PxGeometryType::eBOX):
	{
		PxBoxGeometry boxGeo;
		shape->getBoxGeometry(boxGeo);
		vec3 extents(boxGeo.halfExtents.x, boxGeo.halfExtents.y, boxGeo.halfExtents.z);
		Gizmos::addAABBFilled(actor_position, extents, geo_color, &rot);
	} break;
	case (PxGeometryType::eCAPSULE):
	{
		PxCapsuleGeometry capsuleGeo;
		shape->getCapsuleGeometry(capsuleGeo);
		Gizmos::addCapsule(actor_position, capsuleGeo.halfHeight * 2, capsuleGeo.radius, 16, 16, geo_color, &rot);

		//float radius;
		//float halfHeight;
		//bool status = shape->getCapsuleGeometry(capsuleGeo);
		//if (status)
		//{
		//	radius = capsuleGeo.radius; //copy out capsule radius
		//	halfHeight = capsuleGeo.halfHeight; //copy out capsule half length
		//}
		//PxTransform transform = PxShapeExt::getGlobalPose(*shape, *actor);

	} break;
	case (PxGeometryType::eSPHERE):
	{
		PxSphereGeometry sphereGeo;
		shape->getSphereGeometry(sphereGeo);
		Gizmos::addSphereFilled(actor_position, sphereGeo.radius, 16, 16, geo_color, &rot);

		//float radius;
		//bool status = shape->getSphereGeometry(sphereGeo);
		//if (status)
		//{
		//	radius = sphereGeo.radius;
		//}

	} break;
	case (PxGeometryType::ePLANE):
	{

	} break;
	}
}

void Physics::renderGizmos(PxScene* physics_scene)
{
	PxActorTypeFlags desiredTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
	PxU32 actor_count = physics_scene->getNbActors(desiredTypes);
	PxActor** actor_list = new PxActor*[actor_count];
	physics_scene->getActors(desiredTypes, actor_list, actor_count);

	vec4 geo_color(0, 1, 0, 1);
	for (int actor_index = 0;
		actor_index < (int)actor_count;
		++actor_index)
	{
		PxActor* curr_actor = actor_list[actor_index];
		if (curr_actor->isRigidActor())
		{
			PxRigidActor* rigid_actor = (PxRigidActor*)curr_actor;
			PxU32 shape_count = rigid_actor->getNbShapes();
			PxShape** shapes = new PxShape*[shape_count];
			rigid_actor->getShapes(shapes, shape_count);

			for (int shape_index = 0;
				shape_index < (int)shape_count;
				++shape_index)
			{
				PxShape* curr_shape = shapes[shape_index];
				AddWidget(curr_shape, rigid_actor, geo_color);
			}

			delete[]shapes;
		}
	}

	delete[] actor_list;

	int articulation_count = physics_scene->getNbArticulations();

	vec4 geo_colour(0, 0, 1, 1);
	for (int a = 0; a < articulation_count; ++a)
	{
		PxArticulation* articulation;
		physics_scene->getArticulations(&articulation, 1, a);

		int link_count = articulation->getNbLinks();

		PxArticulationLink** links = new PxArticulationLink*[link_count];
		articulation->getLinks(links, link_count);

		for (int l = 0; l < link_count; ++l)
		{
			PxArticulationLink* link = links[l];
			int shape_count = link->getNbShapes();

			for (int s = 0; s < shape_count; ++s)
			{
				PxShape* shape;
				link->getShapes(&shape, 1, s);
				AddWidget(shape, link, geo_colour);
			}
		}
		delete[] links;
	}
}


PxScene* Physics::createDefaultScene()
{
	PxSceneDesc scene_desc(m_physics->getTolerancesScale());
	scene_desc.gravity = PxVec3(0, -9.807f, 0);

	scene_desc.filterShader = myFliterShader;
	//scene_desc.filterShader = &PxDefaultSimulationFilterShader;

	scene_desc.cpuDispatcher = PxDefaultCpuDispatcherCreate(8);
	PxScene* result = m_physics->createScene(scene_desc);

	return result;
}


void Physics::setUpPhysX()
{
	m_default_filter_shader = PxDefaultSimulationFilterShader;
	m_physics_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_default_allocator,
		m_default_error_callback);
	m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_physics_foundation,
		PxTolerancesScale());
	PxInitExtensions(*m_physics);

	m_physics_material = m_physics->createMaterial(1, 1, 0);
	m_physics_cooker = PxCreateCooking(PX_PHYSICS_VERSION, *m_physics_foundation,
		PxCookingParams(PxTolerancesScale()));
}

void Physics::setUpIntroductionToPhysX()
{
	m_physics_scene = createDefaultScene();
	//add a plane
	PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f), PxQuat(PxHalfPi*1.0f,
		PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic* plane = PxCreateStatic(*m_physics, pose, PxPlaneGeometry(),
		*m_physics_material);
	//add it to the physX scene
	m_physics_scene->addActor(*plane);

	m_physics_scene->addActor(*PxCreateDynamic(*m_physics, PxTransform(1, 0, 0), PxSphereGeometry(2), *m_physics_material, 10));
}