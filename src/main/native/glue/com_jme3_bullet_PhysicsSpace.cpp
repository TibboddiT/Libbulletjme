/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Normen Hansen
 */
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "com_jme3_bullet_PhysicsSpace.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "jmeUserInfo.h"

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getWorldType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getWorldType
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);

    btDynamicsWorldType type = pWorld->getWorldType();
    return (jint) type;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addAction
(JNIEnv *pEnv, jclass, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHK(pEnv, pAction, "The action object does not exist.",)

    pWorld->addAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addCharacterObject
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    btAssert(pUser->m_jmeSpace == NULL);
    pUser->m_jmeSpace = pSpace;

    pWorld->addCollisionObject(pCollisionObject,
            btBroadphaseProxy::CharacterFilter,
            btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addConstraintC
 * Signature: (JJZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addConstraintC
(JNIEnv *pEnv, jclass, jlong spaceId, jlong constraintId,
        jboolean disableCollisions) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",)
    btAssert(pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE);
    btAssert(pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE);

    pWorld->addConstraint(pConstraint, bool(disableCollisions));
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addRigidBody
 * Signature: (JJII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addRigidBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong rigidBodyId, jint proxyGroup,
        jint proxyMask) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    jmeUserPointer const pUser = (jmeUserPointer) pBody->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    btAssert(pUser->m_jmeSpace == NULL);
    pUser->m_jmeSpace = pSpace;

    pWorld->addRigidBody(pBody, proxyGroup, proxyMask);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addProceduralStaticRigidBody
 * Signature: (JJII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addProceduralStaticRigidBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong rigidBodyId, jlong shapeId, jint proxyGroup,
        jint proxyMask) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btCollisionShape * const pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The collision shape does not exist.",)
    btAssert(pShape->getShapeType() == CUSTOM_CONCAVE_SHAPE_TYPE);

    jmeUserPointer const pUser0 = (jmeUserPointer) pBody->getUserPointer();
    NULL_CHK(pEnv, pUser0, "The user object does not exist.",)
    btAssert(pUser0->m_jmeSpace == NULL);
    pUser0->m_jmeSpace = pSpace;

    jmeUserPointer const pUser1 = (jmeUserPointer) pShape->getUserPointer();
    NULL_CHK(pEnv, pUser1, "The user object does not exist.",)
    btAssert(pUser1->m_jmeSpace == NULL);
    pUser1->m_jmeSpace = pSpace;

    pWorld->addRigidBody(pBody, proxyGroup, proxyMask);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    createPhysicsSpace
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;II)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_createPhysicsSpace
(JNIEnv *pEnv, jobject object, jobject minVector, jobject maxVector,
        jint broadphaseType, jint numSolvers) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, minVector, "The min vector does not exist.", 0)
    btVector3 min;
    jmeBulletUtil::convert(pEnv, minVector, &min);

    NULL_CHK(pEnv, maxVector, "The max vector does not exist.", 0)
    btVector3 max;
    jmeBulletUtil::convert(pEnv, maxVector, &max);

    jmePhysicsSpace * const
            pSpace = new jmePhysicsSpace(pEnv, object); //dance003
#if BT_THREADSAFE
    btAssert(numSolvers >= 1);
    btAssert(numSolvers <= BT_MAX_THREAD_COUNT);
    pSpace->createMultiThreadedSpace(min, max, (int) broadphaseType,
            (int) numSolvers);
#else
    btAssert(numSolvers == 1);
    pSpace->createPhysicsSpace(min, max, (int) broadphaseType);
#endif // BT_THREADSAFE

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_getGravity
(JNIEnv *pEnv, jclass, jlong spaceId, jobject storeVector) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);
    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& gravity = pWorld->getGravity();
    jmeBulletUtil::convert(pEnv, &gravity, storeVector);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getNumConstraints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getNumConstraints
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);

    int count = pWorld->getNumConstraints();
    return (jint) count;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getSolverInfo
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_getSolverInfo
(JNIEnv *pEnv, jclass, jlong spaceId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);

    btContactSolverInfo *pInfo = &pWorld->getSolverInfo();
    return reinterpret_cast<jlong> (pInfo);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    isSpeculativeContactRestitution
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_PhysicsSpace_isSpeculativeContactRestitution
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", JNI_FALSE);
    const btDiscreteDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", JNI_FALSE);

    bool result = pWorld->getApplySpeculativeContactRestitution();
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeAction
(JNIEnv *pEnv, jclass, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHK(pEnv, pAction, "The action object does not exist.",)

    pWorld->removeAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeCharacterObject
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    btAssert(pUser->m_jmeSpace == pSpace);
    pUser->m_jmeSpace = NULL;

    pWorld->removeCollisionObject(pCollisionObject);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeConstraint
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeConstraint
(JNIEnv *pEnv, jclass, jlong spaceId, jlong constraintId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The constraint does not exist.",)
    btAssert(pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE);
    btAssert(pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE);

    pWorld->removeConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeRigidBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeRigidBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong rigidBodyId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    jmeUserPointer const pUser = (jmeUserPointer) pBody->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    btAssert(pUser->m_jmeSpace == pSpace);
    pUser->m_jmeSpace = NULL;

    pWorld->removeRigidBody(pBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setGravity
(JNIEnv *pEnv, jclass, jlong spaceId, jobject gravityVector) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    NULL_CHK(pEnv, gravityVector, "The gravity vector does not exist.",)

    btVector3 gravity;
    jmeBulletUtil::convert(pEnv, gravityVector, &gravity);

    pWorld->setGravity(gravity);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setSolverType
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setSolverType
(JNIEnv *pEnv, jclass, jlong spaceId, jint solverType) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    btConstraintSolver *pConstraintSolver;
    btMLCPSolverInterface *pMLCP;
    switch (solverType) {
        case 0: // SI
            pConstraintSolver = new btSequentialImpulseConstraintSolver(); //dance006
            break;
        case 1: // Dantzig
            pMLCP = new btDantzigSolver(); // TODO leak
            pConstraintSolver = new btMLCPSolver(pMLCP); //dance006
            break;
        case 2: // Lemke
            pMLCP = new btLemkeSolver(); // TODO leak
            pConstraintSolver = new btMLCPSolver(pMLCP); //dance006
            break;
        case 3: // PGS
            pMLCP = new btSolveProjectedGaussSeidel(); // TODO leak
            pConstraintSolver = new btMLCPSolver(pMLCP); //dance006
            break;
        case 4: // NNCG
            pConstraintSolver = new btNNCGConstraintSolver(); //dance006
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The solver type is out of range.");
    }

    btConstraintSolver *pOldSolver = pWorld->getConstraintSolver();
    pWorld->setConstraintSolver(pConstraintSolver);
    delete pOldSolver; //dance006
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setSpeculativeContactRestitution
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setSpeculativeContactRestitution
(JNIEnv *pEnv, jclass, jlong spaceId, jboolean setting) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);
    btDiscreteDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    bool enable = (bool)setting;
    pWorld->setApplySpeculativeContactRestitution(enable);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    stepSimulation
 * Signature: (JFIFZZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_stepSimulation
(JNIEnv *pEnv, jclass, jlong spaceId, jfloat tpf, jint maxSteps,
        jfloat accuracy, jboolean enableContactProcessedCallback,
        jboolean enableContactStartedCallback) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    pSpace->stepSimulation(tpf, maxSteps, accuracy,
            enableContactProcessedCallback, enableContactStartedCallback);
}
