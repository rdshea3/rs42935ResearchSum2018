
#include <OgreTextureManager.h>
#include <OgreMath.h>
#include <OgreMeshManager.h>
#include <OgreRoot.h>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include <assert.h>
#include "OBObj.cpp"

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

struct ContactSensorCallback : public btCollisionWorld::ContactResultCallback {

    //! Constructor, pass whatever context you want to have available when processing contacts
    /*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
     *  (supplied by the superclass) for needsCollision() */
    ContactSensorCallback(btRigidBody& tgtBody)
        : btCollisionWorld::ContactResultCallback(), body(tgtBody) { }

    btRigidBody& body; //!< The body the sensor is monitoring



    //! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
    virtual btScalar addSingleResult(btManifoldPoint& cp,
        const btCollisionObjectWrapper* colObj0,int partId0,int index0,
        const btCollisionObjectWrapper* colObj1,int partId1,int index1)
    {
        btVector3 pt; // will be set to point of collision relative to body
        if(colObj0->m_collisionObject==&body) 
        {
            pt = cp.m_localPointA;
            if(!(colObj1->m_collisionObject->isStaticObject()))
            {
            	//printf("Collision with non-static rigidbody.");
            }

        } else 
        {
            assert(colObj1->m_collisionObject==&body && "body does not match either collision object");
            pt = cp.m_localPointB;
            if(!(colObj0->m_collisionObject->isStaticObject()))
            {
            	//printf("Collision with non-static rigidbody.");
            }
        }
        // do stuff with the collision point
        return 0; // There was a planned purpose for the return value of addSingleResult, but it is not used so you can ignore it.
    }
};

class Wrapper
{
public:
	btDiscreteDynamicsWorld* bulletWorld;
	Ogre::SceneManager* mSceneMgr;
	std::vector<OBObj>* objList;
	int numObjects;

	Ogre::SceneNode* playerNode;
	btRigidBody* playerBody;
	//KinematicMotionState* playerMS;
	btDefaultMotionState* playerMS;
	float playerWidth;
	bool playerGrounded;
	btVector3 playerVelocity;
	float playerKneeHeight;
	float playerSpeed;
	float playerJumpStrength;

	OIS::Mouse* mouseIn;
	OIS::Keyboard* keyIn;

	Wrapper(btDiscreteDynamicsWorld* bWorld, Ogre::SceneManager* oManager, OIS::Mouse* m, OIS::Keyboard* k)
	{
		numObjects = 0;
		objList = new std::vector<OBObj>();
		bulletWorld = bWorld;
		mSceneMgr = oManager;
		mouseIn = m;
		keyIn = k;
	}

	int placeInWorld(OBObj newObject)
	{
		bulletWorld->addRigidBody(newObject.bulletBody);
		objList->push_back(newObject); // if we remove elements, we'll be leaving empty indices
		numObjects++; // this is going to help resolve empty elements if we get to needing that
		return objList->size() - 1;
	}

	void placePlayer(Ogre::SceneNode* sn, btRigidBody* rb, btDefaultMotionState* kms, float pw)
	{
		playerMS = kms;
		playerNode = sn;
		playerBody = rb;
		playerWidth = pw;
		playerGrounded = false;
		playerVelocity = btVector3();
		playerKneeHeight = playerWidth / 2;
		playerSpeed = playerWidth * 1.5;
		playerJumpStrength = playerWidth * 10;
		bulletWorld->addRigidBody(playerBody);
	}

	void stepWorld(double timeSinceLastFrame, int substeps = 10)
	{
		ContactSensorCallback callback(*playerBody);
		bulletWorld->contactTest(playerBody, callback);


		btTransform playerTrans;
		playerMS->getWorldTransform(playerTrans);
		btVector3 pos = playerTrans.getOrigin();

		btVector3 base = pos - btVector3(0, playerWidth, 0); // width = height / 3, 1 part ray, 2 parts model
		btVector3 target = base - btVector3(0, playerKneeHeight + 1, 0);
		btCollisionWorld::ClosestRayResultCallback closest(base, target);

		bulletWorld->rayTest(base, target, closest);

		float dist;
		if(closest.hasHit())
		{
			dist = base.distance(closest.m_hitPointWorld) - playerKneeHeight;
		}
		else
		{
			dist = playerKneeHeight + 1;
			playerGrounded = false;
		}

		btVector3 toMove = btVector3();

		if(!playerGrounded)
		{
			playerVelocity -= btVector3(0, OBObj::defaultGravity * timeSinceLastFrame, 0);
			//std::printf("not grounded\n");
		}
		else if(closest.hasHit())
		{
			//std::printf("grounded\n");
			btVector3 inputVector = btVector3(0, 0, 0);
			inputVector.setZ(keyIn->isKeyDown(OIS::KC_W) - keyIn->isKeyDown(OIS::KC_S));
			inputVector.setX(keyIn->isKeyDown(OIS::KC_A) - keyIn->isKeyDown(OIS::KC_D));
			inputVector *= playerSpeed;
			//printf("%f %f %f\n", inputVector.getX(), inputVector.getY(), inputVector.getZ());
			inputVector = inputVector - closest.m_hitNormalWorld * closest.m_hitNormalWorld.dot(inputVector); // project onto plane
			//printf("%f %f %f\n", inputVector.getX(), inputVector.getY(), inputVector.getZ());
			if(keyIn->isKeyDown(OIS::KC_SPACE))
			{
				inputVector += btVector3(0, playerJumpStrength, 0);
				playerGrounded = false;
			}

			playerVelocity *= 0.1;
			playerVelocity += inputVector * 0.9;
			//playerVelocity = inputVector;
			//printf("%f %f %f\n", playerVelocity.getX(), playerVelocity.getY(), playerVelocity.getZ());
			btVector3 playerFacing = playerVelocity;
			playerFacing.setY(0);
			//playerTrans.setRotation(btQuaternion::LookRotation(playerFacing));
			playerFacing = pos + playerFacing;
			playerNode->lookAt(Ogre::Vector3(playerFacing.getX(), playerFacing.getY(), playerFacing.getZ()), Ogre::Node::TS_WORLD);
		}

		float distToFall = playerVelocity.getY() * timeSinceLastFrame;
		distToFall = playerVelocity.getY()<0 ? distToFall : 0;
		if(distToFall>=dist && !playerGrounded && closest.hasHit())
		{
			playerGrounded = true;
			playerVelocity.setY(0);
			toMove = playerVelocity * timeSinceLastFrame + btVector3(0, distToFall-dist, 0);
		}
		else
		{
			toMove = playerVelocity * timeSinceLastFrame;
		}
		playerTrans.setOrigin(pos + toMove);
		playerNode->translate(toMove.getX(), toMove.getY(), toMove.getZ());
		//playerMS->setKinematicPos(playerTrans);
		playerMS->setWorldTransform(playerTrans);

		bulletWorld->stepSimulation(timeSinceLastFrame, substeps);

		for (int i = 0; i < objList->size(); i++)
		{   // I have no idea what this is gonna do if we start taking objects out of the world. Were this a vector of pointers it'd be easy.
			OBObj o = (*objList)[i];
			btTransform newTrans;
			o.bulletBody->getMotionState()->getWorldTransform(newTrans);
			btVector3 newPos = newTrans.getOrigin();
			o.ogreBody->setPosition(newPos.getX(), newPos.getY(), newPos.getZ());
		}
	}
};