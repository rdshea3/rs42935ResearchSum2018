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

	OBObj()																									// Pretty sure things freak out without default constructor
	{																										// Could just be remnant of when still using Player as OBObj sub-class

	}

	OBObj(Ogre::SceneManager* mSceneMgr, std::string materialName, 
		std::string name, ObjectShapes::Shape s, btVector3 startPos,
		float rest, float w, float h, float m,
		std::string optionalMeshName = "")
	{
		restitution = rest;
		width = w;
		height = h;
		mass = m;
		sh = s;
		identifier = name;
		
		btCollisionShape* colShape;																			// Ogre init & collision shape derivation
		ogreBody = mSceneMgr->getRootSceneNode()->createChildSceneNode(name + "_ogre_node");
		switch(sh)
		{
			case ObjectShapes::BALL: // Width = height, use width
			{
				Ogre::Entity* e = mSceneMgr->createEntity(name + "_ogre_ent", Ogre::SceneManager::PT_SPHERE);
				ogreBody->setScale(width / 100.0, width / 100.0, width / 100.0); 							// pt_sphere is 100 units diameter
				e->setMaterialName(materialName);
				ogreBody->attachObject(e);

				colShape = new btSphereShape(width/2);
			}
			break;
			case ObjectShapes::BOX:
			{
				Ogre::Entity* e = mSceneMgr->createEntity(name + "_ogre_ent", Ogre::SceneManager::PT_CUBE); // Ogre Cube model is 100x100x100
				ogreBody->setScale(width / 100.0, height / 100.0, width / 100.0);
				e->setMaterialName(materialName);
				ogreBody->attachObject(e);

				colShape = new btBoxShape(btVector3(width/2, height/2, width/2)); // Half extents
			}
			break;
			default:																						// Never tested OTHER shape. Bet need to adjust scale
				Ogre::Entity* e;
				e = mSceneMgr->createEntity(name + "_ogre_ent", optionalMeshName);
				e->setMaterialName(materialName);
				ogreBody->attachObject(e);
		}
		
		ogreBody->setPosition(startPos.getX(), startPos.getY(), startPos.getZ());							
																											// shape-independent bullet init
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), startPos));
		btVector3 inertia(0,0,0);
		colShape->calculateLocalInertia(mass, inertia);
		btRigidBody::btRigidBodyConstructionInfo rbCI(mass, motionState, colShape, inertia);
		bulletBody = new btRigidBody(rbCI);
		bulletBody->setRestitution(restitution);
	}



};