#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include <iostream>
#include <vector>

#include "constants.h"
int step = 0;
const double kMass= 13e6*1.7e-27/5;
const double kChromRadiusMono = 1e-8;
const double kChromLength = 2.5e-8;
const double kViscosity = 6.6e-3;
const double kB = 1.38e-23;
const double kTemp = 310.;
const double kFriction=6.0*M_PI*kViscosity*20e-9;
const double kDT = kMass/kFriction;

void loggingCallback(btDynamicsWorld* world, btScalar timeStep) {
  int numManifolds = world->getDispatcher()->getNumManifolds();
  //std::cout << step++ << std::endl;
	for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = contactManifold->getBody0();
    const btCollisionObject* obB = contactManifold->getBody1();

    // if (obA->isStaticObject() || obB->isStaticObject()) continue;
    // contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();

    // For each contact point in that manifold
    // for (int j = 0; j < numContacts; j++) {
    //   //Get the contact information
    //   std::cout << "dist: " << contactManifold->getContactPoint(j).getDistance() << std::endl;
    //   std::cout << obA->getUserIndex2() <<", " << obB->getUserIndex2() << std::endl;
    //   const btVector3& normalOnB = contactManifold->getContactPoint(j).m_normalWorldOnB;
    //   std::cout << normalOnB[0] << " " << normalOnB[1] << " " << normalOnB[2] << std::endl;
    // }
  }
}

int main() {
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
  dynamicsWorld->getSolverInfo().m_erp = btScalar(0.8);
  dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(10);
  dynamicsWorld->setGravity(btVector3(0, 0, 0));
  dynamicsWorld->setInternalTickCallback(loggingCallback, &dynamicsWorld, kDT);

  // create free moving capsule
  btTransform startTransform;
  startTransform.setOrigin(btVector3(0,0,0));
  btVector3 localInertia(0,0,0);
  btScalar mass(kMass);
  double len = kChromLength-2*kChromRadiusMono;
  btCollisionShape* capsuleShape = new btCapsuleShapeZ(btScalar(kChromRadiusMono),4e-8);
  capsuleShape->calculateLocalInertia(mass,localInertia);

  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
  btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,capsuleShape,localInertia);
  btRigidBody* capsule_body = new btRigidBody(rbInfo);
  capsule_body->applyCentralForce(btVector3(1e-10,-1.5e-6,-2e-6));
  dynamicsWorld->addRigidBody(capsule_body);

  for (int i = 0; i < 5000; i++) {
    dynamicsWorld->stepSimulation(kDT,1,kDT);
    btVector3 capsule = capsule_body->getCenterOfMassPosition();
    std::cout << capsule.getX() << " " << capsule.getY() << " " << capsule.getZ() << std::endl;
  }

 // CProfileManager::dumpAll();

  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;
  return 0;
}
