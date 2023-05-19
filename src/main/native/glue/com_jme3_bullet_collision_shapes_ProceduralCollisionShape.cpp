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
 * Author: tibbo
 */
#include "com_jme3_bullet_collision_shapes_ProceduralCollisionShape.h"

#include "jmePhysicsSpace.h"
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

#include "btBulletDynamicsCommon.h"

class btProcMeshShape: public btConcaveShape
{
public:
	btProcMeshShape()
	{
		m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE;
	}

	btVector3 m_localScaling;

	jfloat *buffer;

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 dims(20000, 3000, 20000);
		aabbMin = -dims;
		aabbMax = dims;
	}

	virtual void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const
	{
		// TODO 
		// - 3 m3 chunks, add some margins to current aabb
		// - Cache every 3 m3 computed chunks in a FIFO

        // printf("in ProceduralCollisionShape.processAllTriangles\n");
        // fflush(stdout);

        jmeUserPointer const pUser = (jmeUserPointer) getUserPointer();
        if (pUser == NULL) {
            printf("null userPointer in processAllTriangles\n");
            fflush(stdout);
            return;
        }

        jmePhysicsSpace * const pSpace = (jmePhysicsSpace *) pUser->m_jmeSpace;
        if (pSpace == NULL) {
            printf("null jmePhysicsSpace in processAllTriangles\n");
            fflush(stdout);
            return;
        }

        JNIEnv * const pEnv = pSpace->getEnvAndAttach();

        jobject javaShape = pEnv->NewLocalRef(pUser->m_javaRef);

        jint nbPoints = pEnv->CallIntMethod(javaShape,
                jmeClasses::ProceduralCollisionShape_getTriangles,
                aabbMin.getX(), aabbMin.getY(), aabbMin.getZ(),
                aabbMax.getX(), aabbMax.getY(), aabbMax.getZ());
        if (pEnv->ExceptionCheck()) {
            printf("exception in processAllTriangles CallObjectMethod\n");
            fflush(stdout);
            return;
        }

		// printf("processAllTriangles: nb triangles fetched: %d\n", nbPoints / 9);

		btVector3 triangle[3];

		for (int i = 0; i < nbPoints; i += 9)
		{
			// printf("  - [%f, %f, %f]\n", buffer[i], buffer[i + 1], buffer[i + 2]);

			triangle[0] = {buffer[i + 0], buffer[i + 1], buffer[i + 2]};
			triangle[1] = {buffer[i + 3], buffer[i + 4], buffer[i + 5]};
			triangle[2] = {buffer[i + 6], buffer[i + 7], buffer[i + 8]};

			callback->processTriangle(triangle, i / 9 + 1, 0);
		}

		// fflush(stdout);

        // printf("out ProceduralCollisionShape.processAllTriangles\n");
        // fflush(stdout);
	}

	void calculateLocalInertia(btScalar ,btVector3& inertia) const
	{
		inertia.setValue(btScalar(0.),btScalar(0.),btScalar(0.));
	}

	void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}

	const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	virtual const char*	getName() const {return "PROCTERRAIN";}
};

/*
 * Class:     com_jme3_bullet_collision_shapes_ProceduralCollisionShape
 * Method:    createShapeNative
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_ProceduralCollisionShape_createShapeNative
  (JNIEnv *pEnv, jobject object, jobject storage) {
    jmeClasses::initJavaClasses(pEnv);

    // printf("in ProceduralCollisionShape_createShapeNative\n");
    // fflush(stdout);

    btProcMeshShape * pShape = new btProcMeshShape();

    jmeUserPointer pUser = new jmeUserInfo();

    pUser->m_javaRef = pEnv->NewWeakGlobalRef(object);

	pShape->buffer = (jfloat *) pEnv->GetDirectBufferAddress(storage);

    pShape->setUserPointer(pUser);

    // printf("ProceduralCollisionShape created\n");
    // fflush(stdout);

    return reinterpret_cast<jlong> (pShape);
}
