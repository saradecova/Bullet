BTPATH=/home/sara/bullet3/bullet-build/src
all:
	g++ -std=c++11 -g -pg -DBT_USE_DOUBLE_PRECISION -DEBUG_TRIANGLE_MESH short_chain.cpp -I /home/sara/bullet3/src $(BTPATH)/BulletDynamics/libBulletDynamics.a  $(BTPATH)/BulletCollision/libBulletCollision.a $(BTPATH)/LinearMath/libLinearMath.a constants.h

#g++ -std=c++11 -g -pg -DBT_USE_DOUBLE_PRECISION -DEBUG_TRIANGLE_MESH test_sim.cpp -I /home/sara/bullet3/src $(BTPATH)/BulletDynamics/libBulletDynamics.a  $(BTPATH)/BulletCollision/libBulletCollision.a $(BTPATH)/LinearMath/libLinearMath.a constants.h
#g++ -std=c++11 -g -DBT_USE_DOUBLE_PRECISION -DEBUG_TRIANGLE_MESH short_chain.cpp -I ../../bullet3/src ${BTPATH}/BulletDynamics/libBulletDynamics.a  ${BTPATH}/BulletCollision/libBulletCollision.a ${BTPATH}/LinearMath/libLinearMath.a constants.h -o short
