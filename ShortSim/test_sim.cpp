#include <btBulletDynamicsCommon.h>
#include <iostream>
#include <vector>

#include "constants.h"
int step = 0;
const double kMass= 13e6*1.7e-27/5;
const double kChromRadiusMono = 1e-8;
const double kChromLength = 1.25e-8;

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

  // Create mesh
  btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
  std::vector<btVector3>* vertices = new std::vector<btVector3>(Trimesh::kNucleusVertices);
  for (int i = 0; i < Trimesh::kNucleusVertices; i++) {
    vertices->at(i) = btVector3(btScalar(Trimesh::nuc1_vertices[3*i]),
                                btScalar(Trimesh::nuc1_vertices[3*i+1]),
                                btScalar(Trimesh::nuc1_vertices[3*i+2]));
  }

  btIndexedMesh mesh;
  mesh.m_numTriangles = Trimesh::kNucleusFaces;
  mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(Trimesh::nuc_indices);
  mesh.m_triangleIndexStride = sizeof (short)*3;
  mesh.m_numVertices = vertices->size();
  mesh.m_vertexStride = sizeof(btVector3);
  mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertices->data());

  meshInterface->addIndexedMesh(mesh, PHY_SHORT);
  bool useQuantizedAabbCompression = false;
  btBvhTriangleMeshShape* trimeshShape =
    new btBvhTriangleMeshShape(meshInterface,useQuantizedAabbCompression);
  btRigidBody::btRigidBodyConstructionInfo nucleus_ci(0, NULL, trimeshShape, btVector3(0,0,0));
  btRigidBody* body = new btRigidBody(nucleus_ci);
  body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CollisionFlags::CF_STATIC_OBJECT);
  body->setUserIndex2(2);
  dynamicsWorld->addRigidBody(body);


  // create free moving capsule
  double len = kChromLength-2*kChromRadiusMono;
  btCollisionShape* capsuleShape = new btCapsuleShapeZ(kChromRadiusMono,len);
  btScalar mass(kMass);
  btVector3 inertia(1., 1., 1.);
  capsuleShape->calculateLocalInertia(mass, inertia);

  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0,0,0));
  btDefaultMotionState* motion_state = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo body_ci(mass, motion_state, capsuleShape, inertia);
  btRigidBody* capsule_body = new btRigidBody(body_ci);
  capsule_body->setUserIndex2(1);
  capsule_body->applyCentralForce(btVector3(0,0.8e-6,1.5e-6));
  dynamicsWorld->addRigidBody(capsule_body);

  for (int i = 0; i < 1000; i++) {
    // std::cout << "STEP: " << step << std::endl;
    dynamicsWorld->stepSimulation(kDT,1,kDT);
    btVector3 capsule = capsule_body->getCenterOfMassPosition();
    btVector3 nucleus = body->getCenterOfMassPosition();
    std::cout << capsule.getX() << " " << capsule.getY() << " " << capsule.getZ() << std::endl;

    // std::cout << "nucleus" << nucleus.getX() << " " << nucleus.getY() << " " << nucleus.getZ() << std::endl;
  }

  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;
  return 0;
}
