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

        printf("in ProceduralCollisionShape.processAllTriangles\n");
        fflush(stdout);

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

        JNIEnv * const pEnv = pSpace->getEnv();

        jobject javaShape = pEnv->NewLocalRef(pUser->m_javaRef);

        jobject triangles = pEnv->CallObjectMethod(javaShape,
                jmeClasses::ProceduralCollisionShape_getTriangles,
                aabbMin.getX(), aabbMin.getY(), aabbMin.getZ(),
                aabbMax.getX(), aabbMax.getY(), aabbMax.getZ());
        if (pEnv->ExceptionCheck()) {
            printf("exception in processAllTriangles CallObjectMethod\n");
            fflush(stdout);
            return;
        }

        jfloat *buffer = (jfloat *) pEnv->GetDirectBufferAddress(triangles);

		btVector3 expand(1.5,1.5,1.5);
		btVector3 min = aabbMin-expand, max = aabbMax+expand;

		btVector3 c  = btVector3((min.x() + max.x()) * 0.5, 0.5, (min.z() + max.z()) * 0.5);
		btVector3 bl = btVector3(min.x(), 0.5, min.z());
		btVector3 br = btVector3(min.x(), 0.5, max.z());
		btVector3 tl = btVector3(max.x(), 0.5, min.z());
		btVector3 tr = btVector3(max.x(), 0.5, max.z());

		btVector3 vertices[3];

		vertices[0] = bl;
		vertices[1] = br;
		vertices[2] = c;
		assert(btCross(vertices[1]-vertices[0], vertices[2]-vertices[0]).y()>0);
		callback->processTriangle(vertices, 1, 0);

		vertices[0] = tr;
		vertices[1] = tl;
		vertices[2] = c;
		assert(btCross(vertices[1]-vertices[0], vertices[2]-vertices[0]).y()>0);
		callback->processTriangle(vertices, 2, 0);

		vertices[0] = br;
		vertices[1] = tr;
		vertices[2] = c;
		assert(btCross(vertices[1]-vertices[0], vertices[2]-vertices[0]).y()>0);
		callback->processTriangle(vertices, 3, 0);

		vertices[0] = tl;
		vertices[1] = bl;
		vertices[2] = c;
		assert(btCross(vertices[1]-vertices[0], vertices[2]-vertices[0]).y()>0);
		callback->processTriangle(vertices, 4, 0);

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
 * Method:    createShape_native
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_ProceduralCollisionShape_createShape_1native
  (JNIEnv *pEnv, jobject object) {
    jmeClasses::initJavaClasses(pEnv);

    printf("in ProceduralCollisionShape_createShape_native\n");
    fflush(stdout);

    btProcMeshShape * pShape = new btProcMeshShape();

    jmeUserPointer pUser = new jmeUserInfo();

    pUser->m_javaRef = pEnv->NewWeakGlobalRef(object);

    pShape->setUserPointer(pUser);

    printf("ProceduralCollisionShape created\n");
    fflush(stdout);

    return reinterpret_cast<jlong> (pShape);
}
