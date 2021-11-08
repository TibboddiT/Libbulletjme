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

import java.nio.FloatBuffer;
import java.util.logging.Logger;

import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;

/**
 * A mesh CollisionShape based on
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
     */
    public ProceduralCollisionShape(ProceduralCollisionShapeTrianglesFeeder trianglesFeeder, int maxTriangles) {
        this.trianglesFeeder = trianglesFeeder;
        this.maxTriangles = maxTriangles;

        this.trianglesStorage = BufferUtils.createFloatBuffer(maxTriangles);

        createShape();
    }

    /**
     * This method is invoked by native code.
     */
    private FloatBuffer getTriangles(float aabbMinX, float aabbMinY, float aabbMinZ, float aabbMaxX, float aabbMaxY, float aabbMaxZ) {
        aabbMin.set(aabbMinX, aabbMinY, aabbMinZ);
        aabbMax.set(aabbMaxX, aabbMaxY, aabbMaxZ);

        this.trianglesFeeder.getTriangles(aabbMin, aabbMax, this.trianglesStorage);

        return this.trianglesStorage;
    }

    /**
     * Instantiate the configured btTriangleMeshShape.
     */
    private void createShape() {
        long shapeId = createShape_native();
        setNativeId(shapeId);
   }
    // *************************************************************************
    // native private methods

    native private long createShape_native();
}
