/*
-----------------------------------------------------------------------------
Filename:    BaseApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
Tutorial Framework (for Ogre 1.9)
http://www.ogre3d.org/wiki/
-----------------------------------------------------------------------------
*/

#include "BaseApplication.h"

#include <OgreTextureManager.h>
#include <OgreMath.h>
#include <OgreMeshManager.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <macUtils.h>
#endif

//---------------------------------------------------------------------------
BaseApplication::BaseApplication(void)
    : mRoot(0),
    mCamera(0),
    mSceneMgr(0),
    mWindow(0),
    mResourcesCfg(Ogre::StringUtil::BLANK),
    mPluginsCfg(Ogre::StringUtil::BLANK),
    mCursorWasVisible(false),
    mShutDown(false),
    mInputManager(0),
    mMouse(0),
    mKeyboard(0),
    mOverlaySystem(0)
{
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
    m_ResourcePath = Ogre::macBundlePath() + "/Contents/Resources/";
#else
    m_ResourcePath = "";
#endif
}
//---------------------------------------------------------------------------
BaseApplication::~BaseApplication(void)
{
  if (mOverlaySystem) delete mOverlaySystem;

  // Remove ourself as a Window listener
  Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
  windowClosed(mWindow);
  delete mRoot;
}

//---------------------------------------------------------------------------
bool BaseApplication::configure(void)
{
    // Show the configuration dialog and initialise the system.
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg.
    if(mRoot->showConfigDialog())
    {
        // If returned true, user clicked OK so initialise.
        // Here we choose to let the system create a default rendering window by passing 'true'.
        mWindow = mRoot->initialise(true, "TutorialApplication Render Window");

        return true;
    }
    else
    {
        return false;
    }
}
//---------------------------------------------------------------------------
void BaseApplication::chooseSceneManager(void)
{
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

    // Initialize the OverlaySystem (changed for Ogre 1.9)
    mOverlaySystem = new Ogre::OverlaySystem();
    mSceneMgr->addRenderQueueListener(mOverlaySystem);
}
//---------------------------------------------------------------------------
void BaseApplication::createCamera(void)
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

    // Position it at 500 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,0,80));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(5);
}
//---------------------------------------------------------------------------
void BaseApplication::createFrameListener(void)
{
    Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;

    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

    //mInputManager = NULL;

    mInputManager = OIS::InputManager::createInputSystem(pl);
    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));


    mMouse->setEventCallback(this);
    mKeyboard->setEventCallback(this);

    // Set initial mouse clipping size
    windowResized(mWindow);

    // Register as a Window listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    mRoot->addFrameListener(this);
}
//---------------------------------------------------------------------------
void BaseApplication::destroyScene(void)
{
}
//---------------------------------------------------------------------------
void BaseApplication::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//---------------------------------------------------------------------------
void BaseApplication::setupResources(void)
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
            // OS X does not set the working directory relative to the app.
            // In order to make things portable on OS X we need to provide
            // the loading with it's own bundle path location.
            if (!Ogre::StringUtil::startsWith(archName, "/", false)) // only adjust relative directories
                archName = Ogre::String(Ogre::macBundlePath() + "/" + archName);
#endif

            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }
}
//---------------------------------------------------------------------------
void BaseApplication::createResourceListener(void)
{
}
//---------------------------------------------------------------------------
void BaseApplication::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

//---------------------------------------------------------------------------
void BaseApplication::go(void)
{
#ifdef _DEBUG
#ifndef OGRE_STATIC_LIB
    mResourcesCfg = m_ResourcePath + "resources_d.cfg";
    mPluginsCfg = m_ResourcePath + "plugins_d.cfg";
#else
    mResourcesCfg = "resources_d.cfg";
    mPluginsCfg = "plugins_d.cfg";
#endif
#else
#ifndef OGRE_STATIC_LIB
    mResourcesCfg = m_ResourcePath + "resources.cfg";
    mPluginsCfg = m_ResourcePath + "plugins.cfg";
#else
    mResourcesCfg = "resources.cfg";
    mPluginsCfg = "plugins.cfg";
#endif
#endif

    if (!setup())
        return;

   

    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

	mCamera->setPosition(100,300,-400);
	mCamera->lookAt(100,20,0);

                                                                                                            //physics world init
    broadphase = new btDbvtBroadphase();
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    solver = new btSequentialImpulseConstraintSolver;
    physicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    physicsWorld->setGravity(btVector3(0, -1 * OBObj::defaultGravity, 0));
    
                                                                                                            //Wrapper world init
    comboWorld = new Wrapper(physicsWorld, mSceneMgr, mMouse, mKeyboard);



    OBObj theball(mSceneMgr, "Examples/SphereMappedRustySteel",                                             //Generic physics objects.
                                "ball", ObjectShapes::BALL, btVector3(100, 300, 0),
                                1.0, 40, 40, 1);
    comboWorld->placeInWorld(theball); // NOTE: OBJECT ALREADY IN OGRE WORLD!
    
    OBObj boxone(mSceneMgr, "Examples/OgreLogo", 
                                "boxone", ObjectShapes::BOX, btVector3(0, 100, 0),
                                0.1, 100, 60, 10);
    comboWorld->placeInWorld(boxone); // NOTE: OBJECT ALREADY IN OGRE WORLD!
                                                                                                            //Ground not present in Wrapper world, static in ogre + bullet
                                                                                                            //Ground ogre init
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
    
    Ogre::Entity* groundEnt = mSceneMgr->createEntity("ground");
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEnt);
    groundEnt->setCastShadows(false);
    groundEnt->setMaterialName("Examples/Rockwall");
    groundEnt->getParentSceneNode()->setPosition(0,0,0);
    groundEnt->getParentSceneNode()->setOrientation(Ogre::Quaternion( 1, 0, 0.1, 0));//Ogre Quaternions are WXYZ instead of XYZW
                                                                                                            //Ground bullet init
	horizontalPlane = new btStaticPlaneShape(btVector3(0, 1, 0) , 1);
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0.1, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo 
                groundRigidBodyCI(0, groundMotionState, horizontalPlane, btVector3(0,0,0));
                // mass 0=inf, motion state, shape, inertia
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    groundRigidBody->setRestitution(0.9);
    physicsWorld->addRigidBody(groundRigidBody);

                                                                                                            //Player init, should push to constructor.
    btVector3 playerStartPos(105,100,0);
    btScalar playerWidth = 50;
    std::string materialName = "Examples/SphereMappedRustySteel";
    std::string name = "player";
                                                                                                            //Bullet
    capsuleShape = new btCapsuleShape(playerWidth / 2, playerWidth*2 / 2); // Half extents.
    btDefaultMotionState* playerMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), playerStartPos));
    btVector3 playerInertia(0,0,0);
    capsuleShape->calculateLocalInertia(1, playerInertia);
    btRigidBody::btRigidBodyConstructionInfo playerCI(1, playerMotionState, capsuleShape, playerInertia);
    btRigidBody* playerRigidBody = new btRigidBody(playerCI);
    playerRigidBody->setRestitution(1);
    playerRigidBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
                                                                                                            //Ogre
    Ogre::SceneNode* ogreBody = mSceneMgr->getRootSceneNode()->createChildSceneNode(name + "_ogre_node");
    Ogre::Entity* head = mSceneMgr->createEntity(name + "_head", Ogre::SceneManager::PT_SPHERE);
    Ogre::Entity* tail = mSceneMgr->createEntity(name + "_tail", Ogre::SceneManager::PT_SPHERE);
    head->setMaterialName(materialName);
    tail->setMaterialName(materialName);
    Ogre::SceneNode* headMount = ogreBody->createChildSceneNode(name + "hmount", Ogre::Vector3(0,50,0));
    Ogre::SceneNode* tailMount = ogreBody->createChildSceneNode(name + "tmount", Ogre::Vector3(0,-50,0));
    headMount->attachObject(head);
    tailMount->attachObject(tail);
    ogreBody->setScale(playerWidth / 100.0, playerWidth / 100.0, playerWidth / 100.0); // Scale of 1 is 200 high 100 wide
    ogreBody->setPosition(playerStartPos.getX(), playerStartPos.getY(), playerStartPos.getZ());
    physicsWorld->addRigidBody(playerRigidBody);
                                                                                                            //Wrapper
    comboWorld->placePlayer(ogreBody, playerRigidBody, playerMotionState, playerWidth);

    mRoot->startRendering();

    // Clean up
    destroyScene();
}
//---------------------------------------------------------------------------
bool BaseApplication::setup(void)
{
    mRoot = new Ogre::Root(mPluginsCfg);

    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    chooseSceneManager();
    createCamera();
    createViewports();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Create any resource listeners (for loading screens)
    createResourceListener();
    // Load resources
    loadResources();

    // Create the scene
    createScene();

    createFrameListener();

    return true;
};
static Ogre::Vector3 btToOgreVec3(btVector3 in)
{
    Ogre::Vector3 out(in.getX(), in.getY(), in.getZ());
    // so new creates a pointer, this strat just sticks it on stack or w/e
    return out;
}
//---------------------------------------------------------------------------
bool BaseApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed() || mShutDown)
    {
    	delete physicsWorld;
    	delete solver;
    	delete dispatcher;
    	delete collisionConfiguration;
    	delete broadphase;
        return false;
    }

    // Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

    comboWorld->stepWorld(evt.timeSinceLastFrame);                                                          // So clean :D

    return true;
}
//---------------------------------------------------------------------------
bool BaseApplication::keyPressed( const OIS::KeyEvent &arg )
{
  if (arg.key == OIS::KC_ESCAPE) {
    mShutDown = true;
  }

  return true;
}
//---------------------------------------------------------------------------
bool BaseApplication::keyReleased(const OIS::KeyEvent &arg)
{
    return true;
}
//---------------------------------------------------------------------------
bool BaseApplication::mouseMoved(const OIS::MouseEvent &arg)
{
    return true;
}
//---------------------------------------------------------------------------
bool BaseApplication::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
    return true;
}
//---------------------------------------------------------------------------
bool BaseApplication::mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
    return true;
}
//---------------------------------------------------------------------------
// Adjust mouse clipping area
void BaseApplication::windowResized(Ogre::RenderWindow* rw)
{
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}
//---------------------------------------------------------------------------
// Unattach OIS before window shutdown (very important under Linux)
void BaseApplication::windowClosed(Ogre::RenderWindow* rw)
{
    // Only close for window that created OIS (the main window in these demos)
    if(rw == mWindow)
    {
        if(mInputManager)
        {
            mInputManager->destroyInputObject(mMouse);
            mInputManager->destroyInputObject(mKeyboard);

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}
//---------------------------------------------------------------------------
