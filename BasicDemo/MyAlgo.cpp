#include "MyAlgo.h"

#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btBoxBoxDetector.h>
#define USE_PERSISTENT_CONTACTS 1
#include <iostream>

MyAlgo::MyAlgo(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
: btActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->needsCollision(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}


MyAlgo::~MyAlgo()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void MyAlgo::processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;
	
	const btBoxShape* box0 = (btBoxShape*)body0Wrap->getCollisionShape();
	const btBoxShape* box1 = (btBoxShape*)body1Wrap->getCollisionShape();

	printf("point 1 = (%f,%f,%f) point 2 = (%f, %f, %f) point 3 = (%f,%f,%f) point 4 = (%f, %f, %f)\n", m_manifoldPtr[0], m_manifoldPtr[1], m_manifoldPtr[3], m_manifoldPtr[4]);

	/// report a contact. internally this will be kept persistent, and contact reduction is done
	resultOut->setPersistentManifold(m_manifoldPtr);
#ifndef USE_PERSISTENT_CONTACTS	
	m_manifoldPtr->clearManifold();
#endif //USE_PERSISTENT_CONTACTS

	btDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
	input.m_transformA = body0Wrap->getWorldTransform();
	input.m_transformB = body1Wrap->getWorldTransform();

	btBoxBoxDetector detector(box0,box1);
	detector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);

#ifdef USE_PERSISTENT_CONTACTS
	//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
#endif //USE_PERSISTENT_CONTACTS

}

btScalar MyAlgo::calculateTimeOfImpact(btCollisionObject* /*body0*/,btCollisionObject* /*body1*/,const btDispatcherInfo& /*dispatchInfo*/,btManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}
