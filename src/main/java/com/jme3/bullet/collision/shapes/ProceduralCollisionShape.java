/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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
package com.jme3.bullet.collision.shapes;

import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;

/**
 * A procedural CollisionShape based on
 * Bullet's btTriangleMeshShape. Not for use in dynamic bodies. Collisions
 * between HeightfieldCollisionShape, ProceduralCollisionShape, and
 * PlaneCollisionShape objects are never detected.
 *
 * @author tibbo
 */
public class ProceduralCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(ProceduralCollisionShape.class.getName());
    // *************************************************************************
    // fields

    final private ProceduralCollisionShapeTrianglesFeeder trianglesFeeder;
    final private int maxTriangles;

    final private FloatBuffer trianglesStorage;

    private Vector3f aabbMin = new Vector3f();
    private Vector3f aabbMax = new Vector3f();

    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape.
     *
     * @param trianglesFeeder the triangles feeder
     * @param maxTriangles max number of triangles the feeder will return
     */
    public ProceduralCollisionShape(
        ProceduralCollisionShapeTrianglesFeeder trianglesFeeder,
        int maxTriangles) {
        this.trianglesFeeder = trianglesFeeder;
        this.maxTriangles = maxTriangles;

        trianglesStorage = BufferUtils.createFloatBuffer(maxTriangles * 3 * 3);

        createShape();
    }

    /**
     * This method is invoked by native code.
     *
     * @param aabbMinX AABB min x
     * @param aabbMinY AABB min y
     * @param aabbMinZ AABB min z
     * @param aabbMaxX AABB max x
     * @param aabbMaxY AABB max x
     * @param aabbMaxZ AABB max x
     * @return number of triangles returned
     */
    private int getTriangles(float aabbMinX, float aabbMinY, float aabbMinZ,
        float aabbMaxX, float aabbMaxY, float aabbMaxZ) {
        aabbMin.set(aabbMinX, aabbMinY, aabbMinZ);
        aabbMax.set(aabbMaxX, aabbMaxY, aabbMaxZ);

        trianglesStorage.clear();

        trianglesFeeder.getTriangles(aabbMin, aabbMax, trianglesStorage);

        trianglesStorage.flip();

        return trianglesStorage.limit();
    }

    /**
     * Instantiate the configured btTriangleMeshShape.
     */
    private void createShape() {
        long shapeId = createShapeNative(trianglesStorage);
        setNativeId(shapeId);
   }
    // *************************************************************************
    // native private methods

    native private long createShapeNative(FloatBuffer storage);
}
