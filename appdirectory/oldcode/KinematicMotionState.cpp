// Taken from http://www.bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates#Kinematic_Bodies

#  include <btBulletDynamicsCommon.h>
#  include <btBulletCollisionCommon.h>

class KinematicMotionState : public btMotionState {
public:
    KinematicMotionState(const btTransform &initialpos) { mPos1 = initialpos; }
    virtual ~ KinematicMotionState() { }
    //void setNode(Ogre::SceneNode *node) { mVisibleobj = node; }
    virtual void getWorldTransform(btTransform &worldTrans) const { worldTrans = mPos1; }
    void setKinematicPos(btTransform &currentPos) { mPos1 = currentPos; }
    virtual void setWorldTransform(const btTransform &worldTrans) { }

protected:
    btTransform mPos1;
};