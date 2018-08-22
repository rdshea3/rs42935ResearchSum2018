#include <OgreTextureManager.h>
#include <OgreMath.h>
#include <OgreMeshManager.h>
#include <OgreRoot.h>
#include <OgreEntity.h>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <assert.h>
namespace ObjectShapes
{
	enum Shape
	{
		BOX, BALL, OTHER
	};
}

class OBObj
{
public:
	static const float defaultGravity = 980 / 2;
	ObjectShapes::Shape sh;
	float width;
	float height;
	float restitution;
	float mass;
	int worldIndex;
	std::string identifier;

	btRigidBody* bulletBody;
	Ogre::SceneNode* ogreBody;

	OBObj()
	{

	}

	OBObj(Ogre::SceneManager* mSceneMgr, std::string materialName, 
		std::string name, ObjectShapes::Shape s, btVector3 startPos,
		float rest, float w, float h, float m,
		std::string optionalMeshName = "")
	{
		restitution = rest;
		width = w;
		height = h; // NOTE: HEIGHT NOT YET EVER ACTUALLY USED!
		mass = m;
		sh = s;
		identifier = name;
		
		btCollisionShape* colShape;
		ogreBody = mSceneMgr->getRootSceneNode()->createChildSceneNode(name + "_ogre_node");
		switch(sh)
		{
			case ObjectShapes::BALL: // Width = height, use width
			{
				Ogre::Entity* e = mSceneMgr->createEntity(name + "_ogre_ent", Ogre::SceneManager::PT_SPHERE);
				ogreBody->setScale(width / 100.0, width / 100.0, width / 100.0); // pt_sphere is 100 units diameter
				e->setMaterialName(materialName);
				ogreBody->attachObject(e);

				colShape = new btSphereShape(width/2);
			}
			break;
			case ObjectShapes::BOX:
			{
				Ogre::Entity* e = mSceneMgr->createEntity(name + "_ogre_ent", Ogre::SceneManager::PT_CUBE); // ASSUME CUBE IS 100x100x100
				ogreBody->setScale(width / 100.0, height / 100.0, width / 100.0);
				e->setMaterialName(materialName);
				ogreBody->attachObject(e);

				colShape = new btBoxShape(btVector3(width/2, height/2, width/2)); // says half-extents in documentation, I assume that's 'radius'
			}
			break;
			default:
				Ogre::Entity* e;
				e = mSceneMgr->createEntity(name + "_ogre_ent", optionalMeshName);
				e->setMaterialName(materialName);
				ogreBody->attachObject(e);
		}
		
		
		ogreBody->setPosition(startPos.getX(), startPos.getY(), startPos.getZ());	

		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), startPos));
		btVector3 inertia(0,0,0);
		colShape->calculateLocalInertia(mass, inertia);
		btRigidBody::btRigidBodyConstructionInfo rbCI(mass, motionState, colShape, inertia);
		bulletBody = new btRigidBody(rbCI);
		bulletBody->setRestitution(restitution); // double up on storage here? depends on if there's get(restitution)
	}



};
/*
class Player : public OBObj
{
public:
	btKinematicCharacterController* controller;
	btPairCachingGhostObject* ghostObj;

	Player(Ogre::SceneManager* mSceneMgr, std::string materialName, 
		std::string name, ObjectShapes::Shape s, btVector3 startPos,
		float rest, float w, float h, float m,
		btDiscreteDynamicsWorld* physWorld, int stepHeight,
		std::string optionalMeshName = "")
	{
		restitution = rest;
		width = w;
		height = h; // NOTE: HEIGHT USED FOR PLAYERS!
		mass = m;
		sh = s;
		identifier = name;

		//NOTE: Can't find good documentation on KCC, so mostly copying some lines from another set-up/question from another person
		btTransform start;
		start.setIdentity();
		start.setOrigin(startPos);

		//Apparently KCC starts with X as up. This quaternion apparently rotates that so Y is up as intended.
		//NOTE: Other code says this makes Z up, examine for issues. Could just be miscom
		double sq05 = sqrt(0.5); 
		start.setRotation(btQuaternion(sq05, 0, 0, sq05));

		ghostObj = NULL;
		ghostObj = new btPairCachingGhostObject();
		ghostObj->setWorldTransform(start);
		physWorld->getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

		btConvexShape* playerShape = new btCapsuleShape(width, height);

		ghostObj->setCollisionShape(playerShape);
		ghostObj->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

		controller = new btKinematicCharacterController(ghostObj, playerShape, btScalar(stepHeight), btVector3(0,1,0));
		// 2 here says Y is up. This is why i think them writing Z beforehand is a miscom. I could be misreading the docs though.
		controller->setFallSpeed(10); // How is this different from gravity?
		controller->setGravity(btVector3(0, defaultGravity, 0));

		//physWorld->addCollisionObject(ghostObj, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
		//physWorld->addCharacter(controller);
		//idk what the last two arguments are for. Deciding what characters collide with I guess..



		ogreBody = mSceneMgr->getRootSceneNode()->createChildSceneNode(name + "_ogre_node");
		
		Ogre::Entity* head = mSceneMgr->createEntity(name + "_head", Ogre::SceneManager::PT_SPHERE);
		Ogre::Entity* tail = mSceneMgr->createEntity(name + "_tail", Ogre::SceneManager::PT_SPHERE);
		head->setMaterialName(materialName);
		tail->setMaterialName(materialName);
		Ogre::SceneNode* headMount = ogreBody->createChildSceneNode(name + "hmount", Ogre::Vector3(0,50,0));
		Ogre::SceneNode* tailMount = ogreBody->createChildSceneNode(name + "tmount", Ogre::Vector3(0,-50,0));
		headMount->attachObject(head);
		tailMount->attachObject(tail);
		ogreBody->setScale(width / 100.0, width / 100.0, width / 100.0); // Scale of 1 is 200 high 100 wide
		ogreBody->setPosition(startPos.getX(), startPos.getY(), startPos.getZ());	

	}


};*/