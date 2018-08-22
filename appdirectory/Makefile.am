ACLOCAL_AMFLAGS= -I m4
noinst_HEADERS= BaseApplication.h TutorialApplication.h

bin_PROGRAMS= OgreApp
OgreApp_CPPFLAGS= -I$(top_srcdir) 
OgreApp_SOURCES= BaseApplication.cpp TutorialApplication.cpp OBObj.cpp Wrapper.cpp
OgreApp_CXXFLAGS= -I/usr/include/OGRE -pthread $(OIS_CFLAGS) -I/lusr/opt/bullet-2.86/include/bullet -I/lusr/opt/bullet-2.86/include
OgreApp_LDADD= -L/usr/lib/x86_64-linux-gnu/OGRE-1.9.0 -lOgreMain -lpthread $(OIS_LIBS) -L/lusr/opt/bullet-2.86/lib -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath
OgreApp_LDFLAGS= -lOgreOverlay -lboost_system -L${PWD}/src

EXTRA_DIST= buildit makeit
AUTOMAKE_OPTIONS= foreign
