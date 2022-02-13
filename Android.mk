LOCAL_PATH := $(call my-dir)
N := $(LOCAL_PATH)/src/main/native

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := \
$(N)/bullet3 \
$(N)/bullet3/BulletDynamics/Featherstone \
$(N)/bullet3/LinearMath \
$(N)/glue \
$(N)/v-hacd/inc \
$(N)/v-hacd/public

LOCAL_CFLAGS := -std=c++11
LOCAL_MODULE := bulletjme

LOCAL_SRC_FILES := \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btDbvt.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btDispatcher.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp \
$(N)/bullet3/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCollisionObject.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCollisionWorld.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btGhostObject.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btManifoldResult.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/btUnionFind.cpp \
$(N)/bullet3/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btBox2dShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btBoxShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btCapsuleShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btCollisionShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btCompoundShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConcaveShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConeShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvex2dShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvexHullShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvexInternalShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvexShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btCylinderShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btEmptyShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btMiniSDF.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btMultiSphereShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btOptimizedBvh.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btSdfCollisionShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btShapeHull.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btSphereShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTetrahedronShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTriangleBuffer.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTriangleCallback.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTriangleMesh.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp \
$(N)/bullet3/BulletCollision/CollisionShapes/btUniformScalingShape.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btContactProcessing.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btGenericPoolAllocator.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btGImpactBvh.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btGImpactShape.cpp \
$(N)/bullet3/BulletCollision/Gimpact/btTriangleShapeEx.cpp \
$(N)/bullet3/BulletCollision/Gimpact/gim_box_set.cpp \
$(N)/bullet3/BulletCollision/Gimpact/gim_contact.cpp \
$(N)/bullet3/BulletCollision/Gimpact/gim_memory.cpp \
$(N)/bullet3/BulletCollision/Gimpact/gim_tri_collision.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp \
$(N)/bullet3/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp \
$(N)/bullet3/BulletDynamics/Character/btKinematicCharacterController.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btBatchedConstraints.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btContactConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btGearConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp \
$(N)/bullet3/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp \
$(N)/bullet3/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp \
$(N)/bullet3/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.cpp \
$(N)/bullet3/BulletDynamics/Dynamics/btRigidBody.cpp \
$(N)/bullet3/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp \
$(N)/bullet3/BulletDynamics/Dynamics/btSimulationIslandManagerMt.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBody.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyGearConstraint.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodySliderConstraint.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.cpp \
$(N)/bullet3/BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.cpp \
$(N)/bullet3/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp \
$(N)/bullet3/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp \
$(N)/bullet3/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp \
$(N)/bullet3/BulletDynamics/Vehicle/btRaycastVehicle.cpp \
$(N)/bullet3/BulletDynamics/Vehicle/btWheelInfo.cpp \
$(N)/bullet3/BulletSoftBody/btDefaultSoftBodySolver.cpp \
$(N)/bullet3/BulletSoftBody/btDeformableBackwardEulerObjective.cpp \
$(N)/bullet3/BulletSoftBody/btDeformableBodySolver.cpp \
$(N)/bullet3/BulletSoftBody/btDeformableContactConstraint.cpp \
$(N)/bullet3/BulletSoftBody/btDeformableContactProjection.cpp \
$(N)/bullet3/BulletSoftBody/btDeformableMultiBodyConstraintSolver.cpp \
$(N)/bullet3/BulletSoftBody/btDeformableMultiBodyDynamicsWorld.cpp \
$(N)/bullet3/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp \
$(N)/bullet3/BulletSoftBody/btSoftBody.cpp \
$(N)/bullet3/BulletSoftBody/btSoftBodyHelpers.cpp \
$(N)/bullet3/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp \
$(N)/bullet3/BulletSoftBody/btSoftMultiBodyDynamicsWorld.cpp \
$(N)/bullet3/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp \
$(N)/bullet3/BulletSoftBody/btSoftRigidDynamicsWorld.cpp \
$(N)/bullet3/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp \
$(N)/bullet3/BulletSoftBody/poly34.cpp \
$(N)/bullet3/LinearMath/btAlignedAllocator.cpp \
$(N)/bullet3/LinearMath/btConvexHullComputer.cpp \
$(N)/bullet3/LinearMath/btConvexHull.cpp \
$(N)/bullet3/LinearMath/btGeometryUtil.cpp \
$(N)/bullet3/LinearMath/btPolarDecomposition.cpp \
$(N)/bullet3/LinearMath/btQuickprof.cpp \
$(N)/bullet3/LinearMath/btSerializer64.cpp \
$(N)/bullet3/LinearMath/btSerializer.cpp \
$(N)/bullet3/LinearMath/btThreads.cpp \
$(N)/bullet3/LinearMath/btVector3.cpp \
$(N)/bullet3/LinearMath/btReducedVector.cpp \
$(N)/glue/com_jme3_bullet_collision_PhysicsCollisionEvent.cpp \
$(N)/glue/com_jme3_bullet_collision_PhysicsCollisionObject.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_Box2dShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_BoxCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_CapsuleCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_CollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_CompoundCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_ConeCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_Convex2dShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_CylinderCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_EmptyShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_GImpactCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_HeightfieldCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_HullCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_infos_CompoundMesh.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_infos_IndexedMesh.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_MeshCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_MultiSphere.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_PlaneCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_SimplexCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_collision_shapes_SphereCollisionShape.cpp \
$(N)/glue/com_jme3_bullet_CollisionSpace.cpp \
$(N)/glue/com_jme3_bullet_joints_Anchor.cpp \
$(N)/glue/com_jme3_bullet_joints_ConeJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_Constraint.cpp \
$(N)/glue/com_jme3_bullet_joints_HingeJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_motors_RotationalLimitMotor.cpp \
$(N)/glue/com_jme3_bullet_joints_motors_RotationMotor.cpp \
$(N)/glue/com_jme3_bullet_joints_motors_TranslationalLimitMotor.cpp \
$(N)/glue/com_jme3_bullet_joints_motors_TranslationMotor.cpp \
$(N)/glue/com_jme3_bullet_joints_New6Dof.cpp \
$(N)/glue/com_jme3_bullet_joints_Point2PointJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_SixDofJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_SixDofSpringJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_SliderJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_SoftAngularJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_SoftLinearJoint.cpp \
$(N)/glue/com_jme3_bullet_joints_SoftPhysicsJoint.cpp \
$(N)/glue/com_jme3_bullet_MultiBody.cpp \
$(N)/glue/com_jme3_bullet_MultiBodyLink.cpp \
$(N)/glue/com_jme3_bullet_MultiBodySpace.cpp \
$(N)/glue/com_jme3_bullet_objects_infos_CharacterController.cpp \
$(N)/glue/com_jme3_bullet_objects_infos_RigidBodyMotionState.cpp \
$(N)/glue/com_jme3_bullet_objects_infos_SoftBodyConfig.cpp \
$(N)/glue/com_jme3_bullet_objects_infos_SoftBodyMaterial.cpp \
$(N)/glue/com_jme3_bullet_objects_infos_VehicleController.cpp \
$(N)/glue/com_jme3_bullet_objects_infos_VehicleTuning.cpp \
$(N)/glue/com_jme3_bullet_objects_MultiBodyCollider.cpp \
$(N)/glue/com_jme3_bullet_objects_PhysicsCharacter.cpp \
$(N)/glue/com_jme3_bullet_objects_PhysicsGhostObject.cpp \
$(N)/glue/com_jme3_bullet_objects_PhysicsRigidBody.cpp \
$(N)/glue/com_jme3_bullet_objects_PhysicsSoftBody.cpp \
$(N)/glue/com_jme3_bullet_objects_VehicleWheel.cpp \
$(N)/glue/com_jme3_bullet_PhysicsSoftSpace.cpp \
$(N)/glue/com_jme3_bullet_PhysicsSpace.cpp \
$(N)/glue/com_jme3_bullet_RotationOrder.cpp \
$(N)/glue/com_jme3_bullet_SoftBodyWorldInfo.cpp \
$(N)/glue/com_jme3_bullet_SolverInfo.cpp \
$(N)/glue/com_jme3_bullet_util_DebugShapeFactory.cpp \
$(N)/glue/com_jme3_bullet_util_NativeLibrary.cpp \
$(N)/glue/com_jme3_bullet_util_NativeSoftBodyUtil.cpp \
$(N)/glue/jmeBulletUtil.cpp \
$(N)/glue/jmeClasses.cpp \
$(N)/glue/jmeCollisionSpace.cpp \
$(N)/glue/jmeMotionState.cpp \
$(N)/glue/jmeMultiBodySpace.cpp \
$(N)/glue/jmePhysicsSoftSpace.cpp \
$(N)/glue/jmePhysicsSpace.cpp \
$(N)/glue/vhacd_VHACD.cpp \
$(N)/glue/vhacd_VHACDHull.cpp \
$(N)/glue/vhacd_VHACDParameters.cpp \
$(N)/v-hacd/src/FloatMath.cpp \
$(N)/v-hacd/src/VHACD-ASYNC.cpp \
$(N)/v-hacd/src/VHACD.cpp \
$(N)/v-hacd/src/vhacdICHull.cpp \
$(N)/v-hacd/src/vhacdManifoldMesh.cpp \
$(N)/v-hacd/src/vhacdMesh.cpp \
$(N)/v-hacd/src/vhacdRaycastMesh.cpp \
$(N)/v-hacd/src/vhacdVolume.cpp

include $(BUILD_SHARED_LIBRARY)
