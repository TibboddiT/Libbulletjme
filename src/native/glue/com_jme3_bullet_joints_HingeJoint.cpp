/*
 * Copyright (c) 2009-2019 jMonkeyEngine
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

/**
 * Author: Normen Hansen
 */
#include "com_jme3_bullet_joints_HingeJoint.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    createJoint
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_HingeJoint_createJoint
    (JNIEnv * env, jobject object, jlong bodyIdA, jlong bodyIdB,
            jobject pivotInA, jobject axisInA, jobject pivotInB,
            jobject axisInB) {
        jmeClasses::initJavaClasses(env);

        btRigidBody* rbA = reinterpret_cast<btRigidBody*> (bodyIdA);
        if (rbA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "Rigid body A does not exist.");
            return 0L;
        }

        btRigidBody* rbB = reinterpret_cast<btRigidBody*> (bodyIdB);
        if (rbB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "Rigid body B does not exist.");
            return 0L;
        }

        if (pivotInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null pivotInA");
            return 0L;
        }
        btVector3 pivotA;
        jmeBulletUtil::convert(env, pivotInA, &pivotA);

        if (pivotInB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null pivotInB");
            return 0L;
        }
        btVector3 pivotB;
        jmeBulletUtil::convert(env, pivotInB, &pivotB);

        if (axisInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null axisInA");
            return 0L;
        }
        btVector3 axisA;
        jmeBulletUtil::convert(env, axisInA, &axisA);

        if (axisInB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null axisInB");
            return 0L;
        }
        btVector3 axisB;
        jmeBulletUtil::convert(env, axisInB, &axisB);

        btHingeConstraint* joint = new btHingeConstraint(*rbA, *rbB, pivotA,
                pivotB, axisA, axisB);
        return reinterpret_cast<jlong> (joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    createJoint1
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_HingeJoint_createJoint1
    (JNIEnv * env, jobject object, jlong bodyIdA, jobject pivotInA,
            jobject axisInA, jboolean useReferenceFrameA) {
        jmeClasses::initJavaClasses(env);

        btRigidBody* rbA = reinterpret_cast<btRigidBody*> (bodyIdA);
        if (rbA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btRigidBody does not exist.");
            return 0L;
        }

        if (pivotInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null pivotInA");
            return 0L;
        }
        btVector3 pivot;
        jmeBulletUtil::convert(env, pivotInA, &pivot);

        if (axisInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null axisInA");
            return 0L;
        }
        btVector3 axis;
        jmeBulletUtil::convert(env, axisInA, &axis);

        btHingeConstraint* joint
                = new btHingeConstraint(*rbA, pivot, axis, useReferenceFrameA);
        return reinterpret_cast<jlong> (joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    enableMotor
     * Signature: (JZFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_enableMotor
    (JNIEnv * env, jobject object, jlong jointId, jboolean enable, jfloat targetVelocity, jfloat maxMotorImpulse) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return;
        }
        joint->enableAngularMotor(enable, targetVelocity, maxMotorImpulse);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getEnableAngularMotor
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_HingeJoint_getEnableAngularMotor
    (JNIEnv * env, jobject object, jlong jointId) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return false;
        }
        return joint->getEnableAngularMotor();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getHingeAngle
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getHingeAngle
    (JNIEnv * env, jobject object, jlong jointId) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return 0;
        }
        return joint->getHingeAngle();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getLowerLimit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getLowerLimit
    (JNIEnv * env, jobject object, jlong jointId) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return 0;
        }
        return joint->getLowerLimit();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getMaxMotorImpulse
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getMaxMotorImpulse
    (JNIEnv * env, jobject object, jlong jointId) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return 0;
        }
        return joint->getMaxMotorImpulse();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getMotorTargetVelocity
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getMotorTargetVelocity
    (JNIEnv * env, jobject object, jlong jointId) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return 0;
        }
        return joint->getMotorTargetVelocity();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getUpperLimit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getUpperLimit
    (JNIEnv * env, jobject object, jlong jointId) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return 0;
        }
        return joint->getUpperLimit();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    setAngularOnly
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setAngularOnly
    (JNIEnv * env, jobject object, jlong jointId, jboolean angular) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return;
        }
        joint->setAngularOnly(angular);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    setLimit
     * Signature: (JFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setLimit__JFF
    (JNIEnv * env, jobject object, jlong jointId, jfloat low, jfloat high) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return;
        }
        return joint->setLimit(low, high);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    setLimit
     * Signature: (JFFFFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setLimit__JFFFFF
    (JNIEnv * env, jobject object, jlong jointId, jfloat low, jfloat high, jfloat softness, jfloat biasFactor, jfloat relaxationFactor) {
        btHingeConstraint* joint = reinterpret_cast<btHingeConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btHingeConstraint does not exist.");
            return;
        }
        return joint->setLimit(low, high, softness, biasFactor, relaxationFactor);
    }

#ifdef __cplusplus
}
#endif
