/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
package com.jme3.bullet.joints;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A SoftPhysicsJoint that allows rotation around an axis, based on Bullet's
 * btSoftBody::AJoint.
 *
 * @author dokthar
 */
public class SoftAngularJoint extends SoftPhysicsJoint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(SoftAngularJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the joint axis (in physics-space coordinates)
     */
    final private Vector3f axis;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a soft-rigid angular joint.
     *
     * @param axis the axis of the joint (in physics-space coordinates, not
     * null, unaffected)
     * @param softA the soft body for the A end (not null, alias created)
     * @param clusterIndexA the index of the cluster for the A end (&ge;0)
     * @param rigidB the rigid body for the B end (not null, alias created)
     */
    public SoftAngularJoint(Vector3f axis, PhysicsSoftBody softA,
            int clusterIndexA, PhysicsRigidBody rigidB) {
        super(softA, clusterIndexA, rigidB);
        Validate.finite(axis, "axis");

        this.axis = axis.clone();
        createJoint();
    }

    /**
     * Instantiate a soft-soft angular joint. Each soft body must contain a
     * cluster.
     *
     * @param axis the axis of the joint (in physics-space coordinates, not
     * null, unaffected)
     * @param softA the soft body for the A end (not null, alias created)
     * @param clusterIndexA the index of the cluster for the A end (&ge;0)
     * @param softB the soft body for the B end (not null, alias created)
     * @param clusterIndexB the index of the cluster for the B end (&ge;0)
     */
    public SoftAngularJoint(Vector3f axis, PhysicsSoftBody softA,
            int clusterIndexA, PhysicsSoftBody softB, int clusterIndexB) {
        super(softA, clusterIndexA, softB, clusterIndexB);
        Validate.finite(axis, "axis");

        this.axis = axis.clone();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the joint axis.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the axis (in physics-space coordinates, either storeResult or a
     * new vector, not null)
     */
    public Vector3f copyAxis(Vector3f storeResult) {
        Vector3f result;
        // TODO verify copy
        if (storeResult == null) {
            result = axis.clone();
        } else {
            result = storeResult.set(axis);
        }

        return result;
    }

    /**
     * Alter the joint axis.
     *
     * @param newAxis the desired axis (in physics-space coordinates, not null,
     * unaffected)
     */
    public void setAxis(Vector3f newAxis) {
        long jointId = nativeId();
        axis.set(newAxis);
        setAxis(jointId, newAxis);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured btSoftBody::AJoint.
     */
    private void createJoint() {
        PhysicsSoftBody a = getSoftBodyA();
        long ida = a.nativeId();
        int cia = clusterIndexA();
        assert cia >= 0 : cia;
        assert cia < a.countClusters() : cia;

        long idb = getBodyB().nativeId();
        int cib = clusterIndexB();

        float erp = getERP();
        float cfm = getCFM();
        float split = getSplit();

        long jointId;
        if (isSoftRigid()) {
            assert cib == -1 : cib;
            jointId = createJointSoftRigid(
                    ida, cia, idb, erp, cfm, split, axis);
        } else {
            PhysicsSoftBody b = getSoftBodyB();
            assert cib >= 0 : cib;
            assert cib < b.countClusters() : cib;
            jointId = createJointSoftSoft(
                    ida, cia, idb, cib, erp, cfm, split, axis);
        }
        setNativeIdNotTracked(jointId);
        assert checkParameters();
    }
    // *************************************************************************
    // native private methods

    native private static long createJointSoftRigid(long softIdA,
            int clusterIndexA, long rigidIdB, float erp, float cfm, float split,
            Vector3f axis);

    native private static long createJointSoftSoft(long softIdA,
            int clusterIndexA, long softIdB, int clusterIndexB, float erp,
            float cfm, float split, Vector3f axis);

    native private static void setAxis(long jointId, Vector3f axis);
}
