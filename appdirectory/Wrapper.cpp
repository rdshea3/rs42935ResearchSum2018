
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
            	/*btTransform playerTrans;
				((btRigidBody*)colObj0->m_collisionObject)->getMotionState()->getWorldTransform(playerTrans);
				btVector3 pos = playerTrans.getOrigin();
            	btDefaultMotionState* ms = (btDefaultMotionState*)(body.getMotionState());
            	playerTrans.setOrigin(pos - cp.m_normalWorldOnB * cp.getDistance());
            	ms->setWorldTransform(playerTrans);*/
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

	Ogre::SceneNode* playerNode;											//Should be in player type object.
	btRigidBody* playerBody;
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

	int placeInWorld(OBObj newObject)										//Add generic physics object to bullet world, physics object is already in ogre world from constructor
	{
		bulletWorld->addRigidBody(newObject.bulletBody);
		objList->push_back(newObject); // if we remove elements, we'll be leaving empty indices
		numObjects++; // this is going to help resolve empty elements if we get to needing that
		return objList->size() - 1;
	}

	void placePlayer(Ogre::SceneNode* sn, btRigidBody* rb, btDefaultMotionState* kms, float pw)
	{																		//Build player in Wrapper space
		playerMS = kms;														//Later iterations would have this pushed out of Wrapper.cpp into a player class that the wrapper would hold
		playerNode = sn;
		playerBody = rb;
		playerWidth = pw;
		playerGrounded = false;
		playerVelocity = btVector3();
		playerKneeHeight = playerWidth / 2;									//Trying to keep these things in terms of player width where possible
		playerSpeed = playerWidth * 1.5;
		playerJumpStrength = playerWidth * 10;
		//bulletWorld->addRigidBody(playerBody);
	}

	void stepWorld(double timeSinceLastFrame, int substeps = 10)
	{
																			//Contact test - WIP - non functional
		ContactSensorCallback callback(*playerBody);
		bulletWorld->contactTest(playerBody, callback);
																			//Get current kinematic position
		btTransform playerTrans;
		playerMS->getWorldTransform(playerTrans);
		btVector3 pos = playerTrans.getOrigin();
																			//Perform feet raycast
		btVector3 base = pos - btVector3(0, playerWidth, 0); 				// Knees start at bottom of model, knee height 1/4 player model height
		btVector3 target = base - btVector3(0, playerKneeHeight + 1, 0);
		btCollisionWorld::ClosestRayResultCallback closest(base, target);
		bulletWorld->rayTest(base, target, closest);

																			//set dist to length from bottom of player feet to ray hit, no hit = max length & player is not grounded
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
		if(!playerGrounded) 												// Gravity for falling player
		{
			playerVelocity -= btVector3(0, OBObj::defaultGravity * timeSinceLastFrame, 0);
		}
		else if(closest.hasHit()) 											// If grounded & feet making contact, establish movement vector according to inputs
		{
			btVector3 inputVector = btVector3(0, 0, 0);
			inputVector.setZ(keyIn->isKeyDown(OIS::KC_W) - keyIn->isKeyDown(OIS::KC_S));
			inputVector.setX(keyIn->isKeyDown(OIS::KC_A) - keyIn->isKeyDown(OIS::KC_D));
			inputVector *= playerSpeed;
			inputVector = inputVector - closest.m_hitNormalWorld * closest.m_hitNormalWorld.dot(inputVector); // project onto plane
			// MISSING, ENSURING MAXIMUM POSSIBLE ANGLE TO CLIMB

			if(keyIn->isKeyDown(OIS::KC_SPACE)) 							// Adding jump to movement vector
			{
				inputVector += btVector3(0, playerJumpStrength, 0);
				playerGrounded = false;
			}
																			// Modify player velocity, extra complications to allow for messing with friction
			playerVelocity *= 0.1;
			playerVelocity += inputVector * 0.9;
			
																			// set facing of ogre model, bullet model is symmetrical across all valid facings so unncessary to rotate
			btVector3 playerFacing = playerVelocity;
			playerFacing.setY(0);
			playerFacing = pos + playerFacing;
			playerNode->lookAt(Ogre::Vector3(playerFacing.getX(), playerFacing.getY(), playerFacing.getZ()), Ogre::Node::TS_WORLD);
		}

		float distToFall = playerVelocity.getY() * timeSinceLastFrame; 		// Determine if player is going to fall through object below them, adjust movement and velocity if so
		distToFall = playerVelocity.getY()<0 ? distToFall : 0;
		if(distToFall>=dist && !playerGrounded && closest.hasHit())
		{
			playerGrounded = true;
			playerVelocity.setY(0); 										// This will produce strange behavior, if landing on a steep incline.
			toMove = playerVelocity * timeSinceLastFrame + btVector3(0, distToFall-dist, 0);
		}
		else
		{
			toMove = playerVelocity * timeSinceLastFrame;
		}

		playerTrans.setOrigin(pos + toMove);
		playerNode->translate(toMove.getX(), toMove.getY(), toMove.getZ()); // Update OgrePos
		playerMS->setWorldTransform(playerTrans);							// Update bulletPos

		bulletWorld->stepSimulation(timeSinceLastFrame, substeps);			// Step bullet for all generic physics objects.

		for (int i = 0; i < objList->size(); i++)							// Sync Ogre to new bullet state for all generic physics objects
		{   
			OBObj o = (*objList)[i];
			btTransform newTrans;
			o.bulletBody->getMotionState()->getWorldTransform(newTrans);
			btVector3 newPos = newTrans.getOrigin();
			o.ogreBody->setPosition(newPos.getX(), newPos.getY(), newPos.getZ());
		}
	}
};