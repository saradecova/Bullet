#include <btBulletDynamicsCommon.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>


const double kChromRadiusMono = 1e-8;
const double kChromLength = 1.25e-8;
const double kChromMass = 13e6*1.7e-27/5;
const double kCylLength = 15e-8;

const int kNoSegments = 18;

void loadSegmentData(std::vector<std::vector<double> >* segments,
		     std::vector<std::vector<double> >* poles) {
  std::string line;
  std::ifstream init_file("/home/sara/Bullet/ShortSim/InitCoord.txt");
  if (init_file) {
    std::stringstream ss;
    ss << init_file.rdbuf();
    init_file.close();
    const int no_fields = 19;
    for (int i = 0; i < kNoSegments; i++) {
      segments->at(i) = std::vector<double>(no_fields);
      std::string tmp;
      ss >> tmp >> tmp; // discard first two values
      for (int j = 0; j < no_fields; j++) {
	ss >> segments->at(i)[j];
      }
    }

    poles->at(0) = std::vector<double>(no_fields);
    std::string tmp;
    ss >> tmp >> tmp;
    for (int i = 0; i < no_fields; i++) {
      ss >> poles->at(0)[i];
    }
  }
  else std::cout << "Unable to open file InitCoord25e-9_18seg.txt" << std::endl;
}

int main () {
    std::cout << "Hello World!" << std::endl;

    // Build the broadphase
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    // Set up the collision configuration and dispatcher
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // The actual physics solver
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    // The world.
    btDiscreteDynamicsWorld* dynamicsWorld =
      new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    dynamicsWorld->getSolverInfo().m_erp = btScalar(0.8);
    dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(0.01);

    // load chromosome data
    std::vector<std::vector<double> > segments(kNoSegments);
    std::vector<std::vector<double> > poles(1);
    loadSegmentData(&segments, &poles);

    // Create Chromosome mono shape
    double len = kChromLength-0.9e-8;
    btCollisionShape* capsuleShape = new btCapsuleShapeZ(kChromRadiusMono,len);
    btScalar mass(kChromMass);
    btVector3 inertia(1., 1., 1.);
    capsuleShape->calculateLocalInertia(mass, inertia);
    
    btTransform startTransform;
    startTransform.setIdentity();
    for(int i=0;i<kNoSegments;++i) {
      startTransform.setOrigin(btVector3(btScalar(segments[i][1]),
					 btScalar(segments[i][2]),
					 btScalar(segments[i][3])));
      btDefaultMotionState* motion_state = new btDefaultMotionState(startTransform);
      btRigidBody::btRigidBodyConstructionInfo body_ci(mass, motion_state, capsuleShape, inertia);
      btRigidBody* capsule_body = new btRigidBody(body_ci);
      capsule_body->setUserIndex2(i);
      dynamicsWorld->addRigidBody(capsule_body);
    }

    // Create Pole
    btCollisionShape* poleShape =
      new btCylinderShape(btVector3(kChromRadiusMono, kCylLength, kCylLength));
    
    btVector3 pole_localInertia(1,0,0);
    poleShape->calculateLocalInertia(mass, pole_localInertia);
    startTransform.setOrigin(btVector3(btScalar(poles[0][1]),
				       btScalar(poles[0][2]),
				       btScalar(poles[0][3])));
    btDefaultMotionState* pole_ms = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo pole_ci(mass, pole_ms, poleShape, inertia);
    btRigidBody* pole_body = new btRigidBody(pole_ci);
    pole_body->setUserIndex2(20);
    dynamicsWorld->addRigidBody(pole_body);

    // Set up constraints:
    
    
    // Clean up behind ourselves like good little programmers
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;

    return 0;
}
