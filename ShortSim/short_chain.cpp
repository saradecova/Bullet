#include <btBulletDynamicsCommon.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <ctime>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/normal_distribution.hpp>

#include "constants.h"

const double kChromRadiusMono = 1e-8;
const double kChromLength = 1.25e-8;
const double kMass= 13e6*1.7e-27/5;
const double kCylLength = 15e-8;

// langevin force constants
const double kViscosity = 6.6e-3;
const double kB = 1.38e-23;
const double kTemp = 310.;
const double kFriction=6.0*M_PI*kViscosity*20e-9;
const double kDT = kMass/kFriction; //delta time (time step)
const int kNoDT = 5000;
const int kDuration = kNoDT*kDT;
const double kSigma = sqrt(2.0*kFriction*kB*kTemp/kDT);

const int kNoSegments = 18;
int step = 0;

// gaussian distribution for langevin force
time_t T = time(0);
boost::random::mt19937 rng(T);

boost::function<double()> randu =
       boost::bind(boost::random::uniform_real_distribution<>(0, 1), rng);
boost::function<double()> randn =
       boost::bind(boost::random::normal_distribution<>(0, 1), rng);

void loadSegmentData(std::vector<std::vector<double>>* segments,
		     std::vector<std::vector<double>>* poles) {
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

btIndexedMesh createMesh(int no_faces, double* vs, int no_vs, unsigned short* is) {
  std::vector<btVector3>* vertices = new std::vector<btVector3>(no_vs);
  for (int i = 0; i < no_vs; i++) {
    vertices->at(i) = btVector3(btScalar(vs[3*i]), btScalar(vs[3*i+1]), btScalar(vs[3*i+2]));
  }
  btIndexedMesh mesh;
  mesh.m_numTriangles = no_faces;
  mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(is);
  mesh.m_triangleIndexStride = sizeof (short)*3;
  mesh.m_numVertices = vertices->size();
  mesh.m_vertexStride = sizeof(btVector3);
  mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertices->data());
  return mesh;
}

void loggingCallback(btDynamicsWorld* world, btScalar timeStep) {
  // std::cout << ++step << std::endl;
  int col = 0;
  int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = contactManifold->getBody0();
    const btCollisionObject* obB = contactManifold->getBody1();

    if (obA->isStaticObject() || obB->isStaticObject()) continue;
    contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();

    // For each contact point in that manifold
    for (int j = 0; j < numContacts; j++) {
      //Get the contact information
      if (contactManifold->getContactPoint(j).getDistance() <0.f) {
        col++;
        // std::cout << obA->getUserIndex2() <<", " << obB->getUserIndex2() << std::endl;
      }
    }
  }
}

void setLangevinTr(btRigidBody* body, btVector3* rand_force, btVector3* friction) {
  const btVector3& lin_velocity = body->getLinearVelocity();
  // std::cout << "velocity: " << lin_velocity[0] << " " << lin_velocity[1] << " " << lin_velocity[2] << std::endl;
  btVector3 force;
  //force = - kFriction * lin_velocity + btVector3(randn() * kSigma, randn() * kSigma, randn() * kSigma);
  force[0] = 2000. * kSigma * (0.5-(randu())) - kFriction * lin_velocity[0]/50;
  force[1] = 2000. * kSigma * (0.5-(randu())) - kFriction * lin_velocity[1]/50;
  force[2] = 2000. * kSigma * (0.5-(randu())) - kFriction * lin_velocity[2]/50;
  // std::cout << "FORCE: "
  //           << force[0] << " "
  //           << force[1] << " "
  //           << force[2] << std::endl;
  body->applyCentralForce(force);
  *rand_force = force + kFriction*lin_velocity/50;
  *friction = kFriction*lin_velocity/50;
}

int main () {
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
  dynamicsWorld->setGravity(btVector3(0, 0, 0));
  // dynamicsWorld->getSolverInfo().m_erp = btScalar(0.8);
  // dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(0.01);

  dynamicsWorld->setInternalTickCallback(loggingCallback, &dynamicsWorld, kDT);

  // create Nucleus as Trimesh
  // Create a static triemsh shape to represent the nucleus and surrounding cylinder.
  btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
  // top hemisphere
  btIndexedMesh nuc_top = createMesh(Trimesh::kNucleusFaces, Trimesh::nuc1_vertices,
  				     Trimesh::kNucleusVertices, Trimesh::nuc_indices);
  meshInterface->addIndexedMesh(nuc_top, PHY_SHORT);
  // bottom hemisphere
  // btIndexedMesh nuc_bot = createMesh(Trimesh::kNucleusFaces, Trimesh::nuc2_vertices,
  //				     Trimesh::kNucleusVertices, Trimesh::nuc_indices);
  // meshInterface->addIndexedMesh(nuc_bot, PHY_SHORT);

  btTransform trans;
  trans.setIdentity();
  bool useQuantizedAabbCompression = false;
  btBvhTriangleMeshShape* trimeshShape =
    new btBvhTriangleMeshShape(meshInterface,useQuantizedAabbCompression);

  btRigidBody::btRigidBodyConstructionInfo nucleus_ci(0, NULL, trimeshShape, btVector3(0,0,0));
  btRigidBody* body = new btRigidBody(nucleus_ci);
  body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CollisionFlags::CF_STATIC_OBJECT);
  body->setUserIndex2(100);
  dynamicsWorld->addRigidBody(body);
  // btCollisionObject* nucleus = new btCollisionObject();
  // nucleus->setCollisionShape(trimeshShape);
  // dynamicsWorld->addCollisionObject(nucleus,short(btBroadphaseProxy::StaticFilter),short(btBroadphaseProxy::AllFilter^btBroadphaseProxy::StaticFilter));

  // load chromosome data
  std::vector<std::vector<double>> segments(kNoSegments);
  std::vector<std::vector<double>> poles(1);
  loadSegmentData(&segments, &poles);

  // Create Chromosome mono shape
  double len = kChromLength-2*kChromRadiusMono;
  btCollisionShape* capsuleShape = new btCapsuleShapeZ(kChromRadiusMono,len);
  btScalar mass(kMass);
  btVector3 inertia(1., 1., 1.);
  capsuleShape->calculateLocalInertia(mass, inertia);

  btAlignedObjectArray<btRigidBody*> cylinders;
  for(int i=0;i<kNoSegments;++i) {
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(btScalar(segments[i][1]),
				       btScalar(segments[i][2]),
				       btScalar(segments[i][3])));
    btDefaultMotionState* motion_state = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo body_ci(mass, motion_state, capsuleShape, inertia);
    btRigidBody* capsule_body = new btRigidBody(body_ci);
    capsule_body->setUserIndex2(i);
    cylinders.push_back(capsule_body);
    btVector3 friction;
    btVector3 rand_force;
    setLangevinTr(capsule_body, &rand_force, &friction);
    dynamicsWorld->addRigidBody(capsule_body);
  }

  // Create Pole
  btTransform startTransform;
  startTransform.setIdentity();
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

  // add N-1 spring constraints
  for(int i=0;i<kNoSegments-1;++i) {
    btRigidBody* b1 = cylinders[i];
    btRigidBody* b2 = cylinders[i+1];
    btPoint2PointConstraint* joint =
      new btPoint2PointConstraint(*b1, *b2,
				  btVector3(0,0,-1*kChromLength),
				  btVector3(0,0,kChromLength));
    dynamicsWorld->addConstraint(joint);
  }

  //  add contraint between 4th cylinder and pole:
  btRigidBody* b1 = cylinders[3];
  btPoint2PointConstraint* pole_chrom_joint =
    new btPoint2PointConstraint(*b1, *pole_body,
				btVector3(0,-1*kChromRadiusMono,0),
				btVector3(0,kCylLength,0));
  dynamicsWorld->addConstraint(pole_chrom_joint);
  btPoint2PointConstraint* pole_joint =
    new btPoint2PointConstraint(*pole_body, btVector3(0, -1*kCylLength,0));
  dynamicsWorld->addConstraint(pole_joint);

  // Stepping through the simulation:
  for (int step = 0; step < 5000; step++) {
    dynamicsWorld->stepSimulation(kDT,1,kDT);
    btAlignedObjectArray<btCollisionObject*> objs = dynamicsWorld->getCollisionObjectArray();
    for (int i = 0; i < objs.size(); i++) {
      btRigidBody* b = static_cast<btRigidBody*>(objs[i]);
      //if (b->getUserIndex2() == 20) {
	    btVector3 v = b->getCenterOfMassPosition();
	    std::cout << v.getX() << " " << v.getY() << " " << v.getZ() << std::endl;
	    //}
      btVector3 rand_force, friction;
      setLangevinTr(static_cast<btRigidBody*>(objs[i]),&rand_force,&friction);
    }
  }

  // Clean up behind ourselves like good little programmers
  for (int i = 0; i < kNoSegments; i++) {
    dynamicsWorld->removeRigidBody(cylinders[i]);
    delete cylinders[i]->getMotionState();
    delete cylinders[i];
  }
  dynamicsWorld->removeRigidBody(pole_body);
  delete pole_body->getMotionState();
  delete pole_body;

  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;
  return 0;
}
