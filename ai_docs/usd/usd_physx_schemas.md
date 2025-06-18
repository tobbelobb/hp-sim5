Here are the PhysX classes, structs, unions and interfaces with brief descriptions:

ForceFieldSchemaPhysxForceFieldAPI | Force field base class that simply specifies the position and enables or disables the ForceField
ForceFieldSchemaPhysxForceFieldConicalAPI | A conical force field that attracts and/or repels rigid bodies from a central point, but not outside of the cone angle limit, depending on the function coefficients
ForceFieldSchemaPhysxForceFieldDragAPI | A force field that slows rigid bodies by generating a force that is opposite to their velocity direction using the forumula f = -linear \* v - square \* v^2
ForceFieldSchemaPhysxForceFieldLinearAPI | A linear force field that attracts and/or repels rigid bodies from a line, defined by a point and direction vector, depending on the function coefficients
ForceFieldSchemaPhysxForceFieldNoiseAPI | A force field that adds randomized motion to a rigid body
ForceFieldSchemaPhysxForceFieldPlanarAPI | A planar force field that attracts and/or repels rigid bodies from a plane, defined by a point and normal vector, depending on the function coefficients
ForceFieldSchemaPhysxForceFieldRingAPI | A force field that applies forces to rotate rigid bodies around a ring, defined by a normal axis through the center of the ring and radius from that axis
ForceFieldSchemaPhysxForceFieldSphericalAPI | A spherical force field that attracts and/or repels rigid bodies from a central point depending on the function coefficients
ForceFieldSchemaPhysxForceFieldSpinAPI | A force field that applies forces to rotate rigid bodies around a line, defined by a spin axis, and varies with the function coefficients
ForceFieldSchemaPhysxForceFieldWindAPI | A force field that simulates an unsteady wind that pushes rigid bodies
ForceFieldSchemaTokensType | ForceFieldSchemaTokens provides static, efficient TfTokens for use in all public USD API
PhysxSchemaJointStateAPI | The PhysicsJointStateAPI is applied to a joint primitive (i.e
PhysxSchemaPhysxArticulationAPI | PhysX articulation extended parameters
PhysxSchemaPhysxAutoAttachmentAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxAutoParticleClothAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxCameraAPI | PhysX camera
PhysxSchemaPhysxCameraDroneAPI | PhysX drone camera that follows its subject from the air as it drives
PhysxSchemaPhysxCameraFollowAPI | PhysX camera that follows behind the subject as it moves
PhysxSchemaPhysxCameraFollowLookAPI | PhysX camera that follows behind the subject's forward vector as it moves
PhysxSchemaPhysxCameraFollowVelocityAPI | PhysX camera that follows behind the subject's velocity vector as it moves, which allows the subject to roll and tumble
PhysxSchemaPhysxCharacterControllerAPI | PhysxCharacterControllerAPI can be applied to a capsuleGeom
PhysxSchemaPhysxCollisionAPI | PhysX collision extended parameters
PhysxSchemaPhysxContactReportAPI | Enables contact reporting for a rigid body or articulation
PhysxSchemaPhysxConvexDecompositionCollisionAPI | PhysX convex decomposition extended parameters
PhysxSchemaPhysxConvexHullCollisionAPI | PhysX convex hull collision extended parameters
PhysxSchemaPhysxCookedDataAPI | PhysX cooked data storage
PhysxSchemaPhysxDeformableAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxDeformableBodyAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxDeformableBodyMaterialAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxDeformableSurfaceAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxDeformableSurfaceMaterialAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxDiffuseParticlesAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxForceAPI | PhysX schema API that applies a force and torque to a rigid body (UsdGeom.Xformable with UsdPhysicsRigidBodyAPI)
PhysxSchemaPhysxHairAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxHairMaterialAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxIsosurfaceAPI | Applied to a PhysxParticleSystem
PhysxSchemaPhysxJointAPI | PhysX joint extended parameters
PhysxSchemaPhysxMaterialAPI | PhysX material extended parameters
PhysxSchemaPhysxMeshMergeCollisionAPI | PhysxMeshMergeCollisionAPI enables implicit mesh merging of given meshes with respect to physics collision geometry representation
PhysxSchemaPhysxMimicJointAPI | Applied to a Physics Joint that must be part of an articulation
PhysxSchemaPhysxParticleAnisotropyAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxParticleAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxParticleClothAPI | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxParticleIsosurfaceAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxParticleSamplingAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxParticleSetAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxParticleSmoothingAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxParticleSystem | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxPBDMaterialAPI | WARNING: This is a draft API; the design is not fixed and may change in the future
PhysxSchemaPhysxPhysicsAttachment | Deprecated: Will be replaced by a new deformable schema in a future release
PhysxSchemaPhysxPhysicsDistanceJointAPI | PhysX distance joint extended parameters
PhysxSchemaPhysxPhysicsGearJoint | Predefined gear joint type
PhysxSchemaPhysxPhysicsInstancer | Core class for instancing physics prims
PhysxSchemaPhysxPhysicsJointInstancer | Physics joint instancer, the prototypes are expected to be UsdPhysicsJoint prim types
PhysxSchemaPhysxPhysicsRackAndPinionJoint | Predefined rack & pinion joint type
PhysxSchemaPhysxResidualReportingAPI | Gives access to residual values that inform about the remaining physics solver error present during the last position and during the last velocity iteration
PhysxSchemaPhysxRigidBodyAPI | PhysX rigid body extended parameters
PhysxSchemaPhysxSceneAPI | PhysX scene extended parameters
PhysxSchemaPhysxSceneQuasistaticAPI | PhysxSceneQuasistaticAPI defines quasistatic mode for simulation
PhysxSchemaPhysxSDFMeshCollisionAPI | PhysX SDF mesh extended parameters
PhysxSchemaPhysxSphereFillCollisionAPI | PhysX sphere fill extended parameters
PhysxSchemaPhysxSurfaceVelocityAPI | PhysxSurfaceVelocityAPI enables surface velocity simulation that injects velocity to the solver through internal contact modify callback
PhysxSchemaPhysxTendonAttachmentAPI | WARNING: Draft API, this design is not fixed and may change in the future
PhysxSchemaPhysxTendonAttachmentLeafAPI | WARNING: Draft API, this design is not fixed and may change in the future
PhysxSchemaPhysxTendonAttachmentRootAPI | WARNING: Draft API, this design is not fixed and may change in the future
PhysxSchemaPhysxTendonAxisAPI | WARNING: Draft API, this design is not fixed and may change in the future
PhysxSchemaPhysxTendonAxisRootAPI | WARNING: Draft API, this design is not fixed and may change in the future
PhysxSchemaPhysxTriangleMeshCollisionAPI | PhysX triangle mesh extended parameters
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI | PhysX triangle mesh simplification extended parameters
PhysxSchemaPhysxTriggerAPI | PhysX trigger
PhysxSchemaPhysxTriggerStateAPI | PhysX trigger state
PhysxSchemaPhysxVehicleAckermannSteeringAPI | Describes a steering system with Ackermann correction for two wheels
PhysxSchemaPhysxVehicleAPI | PhysX vehicle
PhysxSchemaPhysxVehicleAutoGearBoxAPI | Properties of the PhysX vehicle automatic gear shift box
PhysxSchemaPhysxVehicleBrakesAPI | Describes a braking system for a vehicle by specifying which wheels are connected to the brake control and by defining the brake torque that gets applied to those wheels
PhysxSchemaPhysxVehicleClutchAPI | Properties of the PhysX vehicle clutch
PhysxSchemaPhysxVehicleContextAPI | PhysX vehicles general settings
PhysxSchemaPhysxVehicleControllerAPI | PhysX vehicle controller that samples user input to drive the vehicle
PhysxSchemaPhysxVehicleDriveBasicAPI | Very simple drive model for a PhysX vehicle
PhysxSchemaPhysxVehicleDriveStandardAPI | Standard drive model for a PhysX vehicle
PhysxSchemaPhysxVehicleEngineAPI | Properties of a PhysX vehicle engine
PhysxSchemaPhysxVehicleGearsAPI | Properties of PhysX vehicle gears
PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI | Describes which wheels of a vehicle are driven as well as the distribution of the drive torque among those wheels
PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI | Describes a system of graphs to define nonlinear responses to PhysxVehicleControllerAPI command values like accelerator, brake0, brake1 and steer
PhysxSchemaPhysxVehicleSteeringAPI | Describes a steering system for a vehicle by specifying which wheels are connected to the steer control and by defining the maximum steer angle for those wheels (see PhysxVehicleControllerAPI for the steer control)
PhysxSchemaPhysxVehicleSuspensionAPI | Properties of a PhysX vehicle wheel suspension
PhysxSchemaPhysxVehicleSuspensionComplianceAPI | Compliance describes how toe and camber angle and force application points are affected by suspension compression
PhysxSchemaPhysxVehicleTankControllerAPI | PhysX vehicle tank controller to divert torque from the engine to the wheels of the tracks of a wheel based tank vehicle (see PhysxVehicleTankDifferentialAPI)
PhysxSchemaPhysxVehicleTankDifferentialAPI | Differential to set up a wheeled tank vehicle
PhysxSchemaPhysxVehicleTireAPI | Properties of a PhysX vehicle tire
PhysxSchemaPhysxVehicleTireFrictionTable | Table defining the friction values of a tire against a given set of ground materials
PhysxSchemaPhysxVehicleWheelAPI | Properties of a PhysX vehicle wheel
PhysxSchemaPhysxVehicleWheelAttachmentAPI | For every wheel of a vehicle, this class defines the attachment properties
PhysxSchemaPhysxVehicleWheelControllerAPI | PhysX wheel controller that samples user input and allows direct control of the wheel torques and steer angle to drive the vehicle
PhysxSchemaTetrahedralMesh | Deprecated: Will be replaced by UsdGeom.TetMesh in a future release
PhysxSchemaTokensType | PhysxSchemaTokens provides static, efficient TfTokens for use in all public USD API

Scraped from https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/annotated.html on jun 17, 2025.
