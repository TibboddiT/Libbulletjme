/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A convex collision shape optimized for 2-D, based on Bullet's
 * {@code btConvex2dShape}. For a rectangle, use Box2dShape instead.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Convex2dShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(Convex2dShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * shape on which this shape is based, must be convex and lie entirely in
     * the X-Y plane
     */
    final private ConvexShape base;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape based on the specified convex shape.
     *
     * @param base the base shape (not null, convex, alias created)
     */
    public Convex2dShape(ConvexShape base) {
        this.base = base;
        createShape();
    }

    /**
     * Instantiate a 2-D hull shape based on a flipped buffer containing
     * coordinates.
     *
     * @param flippedBuffer the coordinates on which to base the shape (not
     * null, not empty, length a multiple of 3, Z=0, unaffected)
     */
    public Convex2dShape(FloatBuffer flippedBuffer) {
        Validate.nonNull(flippedBuffer, "flipped buffer");
        int numFloats = flippedBuffer.limit();
        assert numFloats > 0 : numFloats;
        assert numFloats % MyVector3f.numAxes == 0 : numFloats;

        this.base = new HullCollisionShape(flippedBuffer);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public CollisionShape getBaseShape() {
        assert base != null;
        return base;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean result = base.canScale(scale);
        return result;
    }

    /**
     * Alter the scale of this shape and its base. CAUTION: Not all shapes can
     * be scaled arbitrarily.
     * <p>
     * Note that if shapes are shared (between collision objects and/or compound
     * shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    @Override
    public void setScale(Vector3f scale) {
        super.setScale(scale);
        /*
         * Update the base to keep its copied scale factors
         * in synch with the native ones.
         */
        base.updateScale();
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate an empty {@code btConvex2dShape}.
     */
    private void createShape() {
        long childId = base.nativeId();
        long shapeId = createShape(childId);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(long childId);
}
