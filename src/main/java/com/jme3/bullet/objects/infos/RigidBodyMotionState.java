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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * The motion state (transform) of a rigid body, with thread-safe access.
 *
 * @author normenhansen
 */
public class RigidBodyMotionState extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodyMotionState.class.getName());
    // *************************************************************************
    // fields

    /**
     * true &rarr; physics transform matches the spatial's local transform,
     * false &rarr; physics transform matches the spatial's world transform
     */
    private boolean applyPhysicsLocal = false;
    /**
     * vehicle reference, or null if the rigid body is a vehicle
     */
    private PhysicsVehicle vehicle = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a motion state.
     */
    public RigidBodyMotionState() {
        long motionStateId = createMotionState();
        super.setNativeId(motionStateId);
        logger.log(Level.FINE, "Created {0}", this);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the location to a Vector3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long motionStateId = nativeId();
        getWorldLocation(motionStateId, result);

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the location to a Vector3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    public Vec3d getLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        long motionStateId = nativeId();
        getWorldLocationDp(motionStateId, result);

        assert result.isFinite() : result;
        return result;
    }

    /**
     * Copy the orientation to a Matrix3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new matrix, not null)
     */
    public Matrix3f getOrientation(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        long motionStateId = nativeId();
        getWorldRotation(motionStateId, result);

        return result;
    }

    /**
     * Copy the orientation to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new instance, not null)
     */
    public Quaternion getOrientation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        long motionStateId = nativeId();
        getWorldRotationQuat(motionStateId, result);

        return result;
    }

    /**
     * Copy the orientation to a Matrix3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new matrix, not null)
     */
    public Matrix3d getOrientationMatrixDp(Matrix3d storeResult) {
        Matrix3d result = (storeResult == null) ? new Matrix3d() : storeResult;

        long motionStateId = nativeId();
        getWorldRotationDp(motionStateId, result);

        return result;
    }

    /**
     * Copy the orientation to a Quatd.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (in physics-space coordinates, either storeResult
     * or a new instance, not null)
     */
    public Quatd getOrientationQuaternionDp(Quatd storeResult) {
        Quatd result = (storeResult == null) ? new Quatd() : storeResult;

        long motionStateId = nativeId();
        getWorldRotationQuatDp(motionStateId, result);

        return result;
    }

    /**
     * Test whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @return true if matching local coordinates, false if matching world
     * coordinates
     */
    public boolean isApplyPhysicsLocal() {
        return applyPhysicsLocal;
    }

    /**
     * Calculate the body's physics transform.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the body's transform in physics-space coordinates (either
     * storeResult or a new instance, not null)
     */
    public Transform physicsTransform(Transform storeResult) {
        Transform transform;
        if (storeResult == null) {
            transform = new Transform();
        } else {
            transform = storeResult.setScale(1f);
        }

        long motionStateId = nativeId();
        getWorldLocation(motionStateId, transform.getTranslation());
        getWorldRotationQuat(motionStateId, transform.getRotation());

        return transform;
    }

    /**
     * Alter whether physics-space coordinates should match the spatial's local
     * coordinates.
     *
     * @param applyPhysicsLocal true&rarr;match local coordinates,
     * false&rarr;match world coordinates (default=false)
     */
    public void setApplyPhysicsLocal(boolean applyPhysicsLocal) {
        this.applyPhysicsLocal = applyPhysicsLocal;
    }

    /**
     * Alter which vehicle uses this motion state.
     *
     * @param vehicle the desired vehicle, or null for none (alias created)
     */
    public void setVehicle(PhysicsVehicle vehicle) {
        this.vehicle = vehicle;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param stateId the native identifier (not zero)
     */
    private static void freeNativeObject(long stateId) {
        assert stateId != 0L;
        finalizeNative(stateId);
    }
    // *************************************************************************
    // native private methods

    native private static boolean applyTransform(
            long stateId, Vector3f location, Quaternion rotation);

    native private static long createMotionState();

    native private static void finalizeNative(long objectId);

    native private static void
            getWorldLocation(long stateId, Vector3f storeVector);

    native private static void
            getWorldLocationDp(long stateId, Vec3d storeVector);

    native private static void
            getWorldRotation(long stateId, Matrix3f storeMatrix);

    native private static void
            getWorldRotationDp(long stateId, Matrix3d storeMatrix);

    native private static void
            getWorldRotationQuat(long stateId, Quaternion storeQuat);

    native private static void
            getWorldRotationQuatDp(long stateId, Quatd storeQuat);
}
