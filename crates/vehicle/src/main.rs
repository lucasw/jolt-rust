// someday:
// #![forbid(unsafe_code)]

use joltc_sys::*;
use vehicle::*;

use rolt::{
    BroadPhaseLayer, BroadPhaseLayerInterface, CastShapeArgs, CastShapeCollectorImpl,
    ClosestHitCastShapeCollector, IntoJolt, ObjectLayer, ObjectLayerPairFilter,
    ObjectVsBroadPhaseLayerFilter, Quat, RShapeCast, RVec3, Vec3,
};

const OL_NON_MOVING: JPC_ObjectLayer = 0;
const OL_MOVING: JPC_ObjectLayer = 1;

const BPL_NON_MOVING: JPC_BroadPhaseLayer = 0;
const BPL_MOVING: JPC_BroadPhaseLayer = 1;
const BPL_COUNT: JPC_BroadPhaseLayer = 2;

struct BroadPhaseLayers;

impl BroadPhaseLayerInterface for BroadPhaseLayers {
    fn get_num_broad_phase_layers(&self) -> u32 {
        BPL_COUNT as u32
    }

    fn get_broad_phase_layer(&self, layer: ObjectLayer) -> BroadPhaseLayer {
        match layer.raw() {
            OL_NON_MOVING => BroadPhaseLayer::new(BPL_NON_MOVING),
            OL_MOVING => BroadPhaseLayer::new(BPL_MOVING),
            _ => unreachable!(),
        }
    }
}

struct ObjectVsBroadPhase;

impl ObjectVsBroadPhaseLayerFilter for ObjectVsBroadPhase {
    fn should_collide(&self, layer1: ObjectLayer, layer2: BroadPhaseLayer) -> bool {
        match layer1.raw() {
            OL_NON_MOVING => layer2.raw() == BPL_MOVING,
            OL_MOVING => true,
            _ => unreachable!(),
        }
    }
}

struct ObjectLayerPair;

impl ObjectLayerPairFilter for ObjectLayerPair {
    fn should_collide(&self, layer1: ObjectLayer, layer2: ObjectLayer) -> bool {
        match layer1.raw() {
            OL_NON_MOVING => layer2.raw() == OL_MOVING,
            OL_MOVING => true,
            _ => unreachable!(),
        }
    }
}

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("jolt_with_rerun")
        .spawn()
        .unwrap();

    rolt::register_default_allocator();
    rolt::factory_init();
    rolt::register_types();

    unsafe {
        let temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);

        let job_system =
            JPC_JobSystemThreadPool_new2(JPC_MAX_PHYSICS_JOBS as _, JPC_MAX_PHYSICS_BARRIERS as _);

        let broad_phase_layer_interface = BroadPhaseLayers;
        let object_vs_broad_phase_layer_filter = ObjectVsBroadPhase;
        let object_layer_pair_filter = ObjectLayerPair;

        let mut physics_system = rolt::PhysicsSystem::new();
        physics_system.set_gravity(Vec3::new(0.0, 0.0, -1.0));

        let max_bodies = 1024;
        let num_body_mutexes = 0;
        let max_body_pairs = 1024;
        let max_contact_constraints = 1024;

        physics_system.init(
            max_bodies,
            num_body_mutexes,
            max_body_pairs,
            max_contact_constraints,
            broad_phase_layer_interface,
            object_vs_broad_phase_layer_filter,
            object_layer_pair_filter,
        );

        // TODO: register body activation listener
        // TODO: register contact listener

        let body_interface = physics_system.body_interface();

        let floor_half_extent = 30.0;
        let floor_height = 1.0;
        let floor_shape = create_box(&JPC_BoxShapeSettings {
            HalfExtent: vec3(floor_half_extent, floor_half_extent, floor_height),
            ..Default::default()
        })
        .unwrap();

        let floor_pos = rvec3(0.0, 0.0, -1.0);
        let floor = body_interface
            .create_body(&JPC_BodyCreationSettings {
                Position: floor_pos,
                Rotation: JPC_Quat {
                    y: 0.0500478,
                    x: 0.0042435,
                    z: 0.0152304,
                    w: 0.9986217,
                },
                MotionType: JPC_MOTION_TYPE_STATIC,
                ObjectLayer: OL_NON_MOVING,
                Shape: floor_shape,
                ..Default::default()
            })
            .unwrap();
        let floor_id = floor.id();
        body_interface.add_body(floor_id, JPC_ACTIVATION_DONT_ACTIVATE);
        rec.log_static(
            "world/floor",
            &rerun::Boxes3D::from_half_sizes([(
                floor_half_extent,
                floor_half_extent,
                floor_height,
            )]),
        )
        .unwrap();

        let sphere_radius = 0.5;
        let sphere_shape = create_sphere(&JPC_SphereShapeSettings {
            Radius: sphere_radius,
            ..Default::default()
        })
        .unwrap();

        let sphere = body_interface
            .create_body(&JPC_BodyCreationSettings {
                Position: rvec3(5.0, 5.0, 3.0),
                MotionType: JPC_MOTION_TYPE_DYNAMIC,
                ObjectLayer: OL_MOVING,
                Shape: sphere_shape,
                ..Default::default()
            })
            .unwrap();
        let sphere_id = sphere.id();

        body_interface.add_body(sphere_id, JPC_ACTIVATION_ACTIVATE);
        body_interface.set_linear_velocity(sphere_id, Vec3::new(0.0, 0.0, 1.5));

        // build a car following the VehicleSixDOFTest.cpp example
        let half_vehicle_length = 2.0;
        let half_vehicle_width = 0.9;
        let half_vehicle_height = 0.2;

        let wheel_radius = 0.3;
        let half_wheel_width = 0.22;
        let half_wheel_travel = 0.6;

        // TODO(lucasw) need to offset center of mass
        let car_body_shape = create_box(&JPC_BoxShapeSettings {
            HalfExtent: vec3(half_vehicle_length, half_vehicle_width, half_vehicle_height),
            ..Default::default()
        })
        .unwrap();

        let car_pos = rvec3(0.0, 0.0, 3.0);
        let car_body = body_interface
            .create_body(&JPC_BodyCreationSettings {
                Position: car_pos,
                MotionType: JPC_MOTION_TYPE_DYNAMIC,
                ObjectLayer: OL_MOVING,
                Shape: car_body_shape,
                // Rotation: JPC_Quat{x: 0.0500478, y: 0.0042435, z: 0.0152304, w: 0.9986217},
                Rotation: JPC_Quat {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                // TODO(lucasw) MassPropertiesOverride is commented out in JoltC/Functions.h
                // so don't try the override
                // OverrideMassProperties: JPC_OVERRIDE_MASS_PROPS_CALC_INERTIA,
                // would have set Mass to 1500.0 if there was a way
                ..Default::default()
            })
            .unwrap();
        let car_body_id = car_body.id();
        body_interface.add_body(car_body_id, JPC_ACTIVATION_ACTIVATE);

        let mut wheel_ids = Vec::new();

        // add four wheels
        for i in 0..4 {
            let sc = 0.9;
            let x;
            let is_front;
            if i < 2 {
                x = half_vehicle_length * sc;
                is_front = true;
            } else {
                x = -half_vehicle_length * sc;
                is_front = false;
            }

            let y;
            let is_left;
            if i % 2 == 0 {
                y = half_vehicle_width * sc;
                is_left = true;
            } else {
                y = -half_vehicle_width * sc;
                is_left = false;
            }

            let wheel_radius = {
                if is_front {
                    wheel_radius
                } else {
                    wheel_radius * 1.5
                }
            };
            let wheel_shape = create_cylinder(&JPC_CylinderShapeSettings {
                HalfHeight: half_wheel_width,
                Radius: wheel_radius,
                ..Default::default()
            })
            .unwrap();

            let wheel_z_offset = -half_wheel_travel * 1.5;
            let wheel_pos1 = rvec3(car_pos.x + x, car_pos.y + y, car_pos.z);
            let wheel_pos2 = rvec3(wheel_pos1.x, wheel_pos1.y, wheel_pos1.z + wheel_z_offset);

            let wheel_body = body_interface
                .create_body(&JPC_BodyCreationSettings {
                    Position: wheel_pos2,
                    // Rotation: JPC_Quat{x: 0.7071068, y: 0.0, z: 0.0, w: 0.7071068},
                    Rotation: JPC_Quat {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                    MotionType: JPC_MOTION_TYPE_DYNAMIC,
                    ObjectLayer: OL_MOVING,
                    Shape: wheel_shape,
                    ..Default::default()
                })
                .unwrap();
            JPC_Body_SetFriction(wheel_body.raw(), 1.0);
            let wheel_id = wheel_body.id();
            body_interface.add_body(wheel_id, JPC_ACTIVATION_ACTIVATE);

            // hinges to let wheels roll
            let spring_settings = JPC_SpringSettings {
                Mode: JPC_SPRING_MODE_FREQUENCY_AND_DAMPING,
                FrequencyOrStiffness: 2.0,
                Damping: 1.0,
            };
            let motor_settings = JPC_MotorSettings {
                SpringSettings: spring_settings,
                MinForceLimit: 0.0,
                MaxForceLimit: 0.0,
                MinTorqueLimit: -0.5e4,
                MaxTorqueLimit: 0.5e4,
            };

            let hinge_settings = JPC_HingeConstraintSettings {
                ConstraintSettings: JPC_ConstraintSettings {
                    Enabled: true,
                    ConstraintPriority: 0,
                    NumVelocityStepsOverride: 0,
                    NumPositionStepsOverride: 0,
                    DrawConstraintSize: 1.0,
                    UserData: 0,
                },
                Space: JPC_ConstraintSpace::default(),
                __bindgen_padding_0: 0,
                // Point on body1 - relative to the car
                Point1: rvec3(x, y, wheel_z_offset),
                HingeAxis1: Vec3::Y.into_jolt(),
                NormalAxis1: Vec3::X.into_jolt(),
                // point on body2- the wheel
                Point2: rvec3(0.0, 0.0, 0.0),
                HingeAxis2: Vec3::Y.into_jolt(),
                NormalAxis2: Vec3::X.into_jolt(),
                // limits outside of +/- pi means no limits
                LimitsMin: -2.0 * std::f32::consts::PI,
                LimitsMax: 2.0 * std::f32::consts::PI,
                LimitsSpringSettings: spring_settings,
                MaxFrictionTorque: 0.0,
                MotorSettings: motor_settings,
            };

            let hinge_constraint = JPC_HingeConstraintSettings_Create(
                &hinge_settings,
                car_body.raw(),
                wheel_body.raw(),
            );
            let constraint = hinge_constraint.cast::<JPC_Constraint>();
            physics_system.add_constraint(constraint);

            /*
            // 2.0f, 1.0f, 1.0e5f, 0.0f
            let spring_settings = JPC_SpringSettings {
                Mode: JPC_SPRING_MODE_FREQUENCY_AND_DAMPING,
                FrequencyOrStiffness: 2.0,
                Damping: 1.0,
            };
            let motor_settings = JPC_MotorSettings {
                SpringSettings: spring_settings,
                MinForceLimit: -1.0e5,
                MaxForceLimit: 1.0e5,
                MinTorqueLimit: 0.0,
                MaxTorqueLimit: 0.0,
            };

            // suspension
            let slider_settings = JPC_SliderConstraintSettings {
                ConstraintSettings: JPC_ConstraintSettings {
                    Enabled: true,
                    ConstraintPriority: 0,
                    NumVelocityStepsOverride: 0,
                    NumPositionStepsOverride: 0,
                    DrawConstraintSize: 1.0,
                    UserData: 0,
                },
                Space: JPC_ConstraintSpace::default(),
                AutoDetectPoint: true,
                __bindgen_padding_0: 0,
                // point on body1 - the car?
                Point1: wheel_pos1,
                SliderAxis1: Vec3::Z.into_jolt(),
                NormalAxis1: Vec3::X.into_jolt(),
                // point on body2- the wheel?
                Point2: rvec3(0.0, 0.0, 0.0),
                SliderAxis2: Vec3::Z.into_jolt(),
                NormalAxis2: Vec3::X.into_jolt(),
                LimitsMin: -half_wheel_travel,
                LimitsMax: half_wheel_travel,
                LimitsSpringSettings: spring_settings,
                MaxFrictionForce: 0.0,
                MotorSettings: motor_settings,
                // ..Default::default()
            };

            let slider_constraint = JPC_SliderConstraintSettings_Create(
                &slider_settings,
                car_body.raw(),
                wheel_body.raw(),
            );

            JPC_SliderConstraint_SetMotorState(slider_constraint, JPC_MOTOR_STATE_POSITION);
            JPC_SliderConstraint_SetTargetPosition(slider_constraint, 0.0);

            let constraint = slider_constraint.cast::<JPC_Constraint>();
            physics_system.add_constraint(constraint);
            */

            // JPC_PhysicsSystem_AddConstraint(physics_system.raw(), constraint);

            // Need better JoltC SixDOF support
            /*
            // Create constraint
            let axis_x = {
                if is_left {
                    -Vec3::X
                } else {
                    Vec3::X
                }
            };
            let axis_y = Vec3::Z;
            // TODO(lucasw) where to put the motor settings?
            // Hinges and Sliders have them, but not six dof

            let settings = JPC_SixDOFConstraintSettings {
                Space: JPC_CONSTRAINT_SPACE_LOCAL_TO_BODY_COM,
                Position1: wheel_pos1,
                AxisX1: axis_x.into_jolt(),
                AxisY1: axis_y.into_jolt(),
                Position2: wheel_pos2,
                AxisX2: axis_x.into_jolt(),
                AxisY2: axis_y.into_jolt(),
                ..Default::default()
            };

            /*
            settings.MakeFixedAxis(EAxis::TranslationX);
            settings.SetLimitedAxis(EAxis::TranslationY, -half_wheel_travel, half_wheel_travel);
            settings.MakeFixedAxis(EAxis::TranslationZ);
            settings.mMotorSettings[EAxis::TranslationY] = motor_settings;

            // Front wheel can rotate around the Z axis for steering
            if (is_front) {
                settings.SetLimitedAxis(EAxis::RotationZ, -cMaxSteeringAngle, cMaxSteeringAngle);
            } else {
                settings.MakeFixedAxis(EAxis::RotationZ);
            }

            // The Z axis is static
            settings.MakeFixedAxis(EAxis::RotationZ);

            // The main engine drives the Y axis
            settings.MakeFreeAxis(EAxis::RotationY);
            settings.mMotorSettings[EAxis::RotationY] = MotorSettings(2.0f, 1.0f, 0.0f, 0.5e4f);
            */

            // The front wheel needs to be able to steer around the Y axis
            // However the motors work in the constraint space of the wheel, and since this rotates around the
            // X axis we need to drive both the Y and Z to steer
            if (is_front) {
                // settings.mMotorSettings[EAxis::RotationY] = settings.mMotorSettings[EAxis::RotationZ] = MotorSettings(10.0f, 1.0f, 0.0f, 1.0e6f);
            }

            // TODO(lucasw) this outputs a JPC_Constraint, not a JPC_SixDOFConstraint
            let wheel_constraint = JPC_SixDOFConstraintSettings_Create(settings, car_body, wheel_body);
            physics_system.add_constraint(wheel_constraint);
            // mWheels[i] = wheel_constraint;

            // Drive the suspension
            wheel_constraint.SetTargetPositionCS(rvec3(0.0, 0.0, -half_wheel_travel));
            // TODO(lucasw) no motor state for sixdof, only hinge and slider have it
            // wheel_constraint.SetMotorState(EAxis::TranslationY, EMotorState::Position);

            // The front wheels steer around the Y axis, but in constraint space of the wheel this means we need to drive
            // both Y and Z (see comment above)
            if is_front {
                wheel_constraint.SetTargetOrientationCS(Quat::IDENTITY.into_jolt());
                // TODO(lucasw) no motor state for sixdof, only hinge and slider have it
                // wheel_constraint.SetMotorState(EAxis::RotationY, EMotorState::Position);
                // wheel_constraint.SetMotorState(EAxis::RotationX, EMotorState::Position);
            }
            */

            wheel_ids.push((wheel_id, wheel_radius, 0));
        }

        // setup physics
        physics_system.optimize_broad_phase();

        let delta_time = 1.0 / 60.0;
        let collision_steps = 1;

        let mut step = 0;
        // while body_interface.is_active(wheel_ids[3]) {
        for _i in 0..1000 {
            rec.set_timestamp_secs_since_epoch("view", step as f64 * delta_time as f64);

            step += 1;

            let position = body_interface.center_of_mass_position(sphere_id);
            rec.log(
                "world/sphere",
                &rerun::Points3D::new([[position.x, position.y, position.z]])
                    .with_colors([rerun::Color::from_unmultiplied_rgba(20, 90, 80, 255)])
                    .with_radii([sphere_radius]),
            )
            .unwrap();

            let velocity = body_interface.linear_velocity(sphere_id);
            println!(
                "Step {step}: Position = ({}, {}, {}), Velocity = ({}, {}, {})",
                position.x, position.y, position.z, velocity.x, velocity.y, velocity.z
            );

            let position = body_interface.center_of_mass_position(car_body_id);
            let quat = body_interface.rotation(car_body_id);

            rec.log(
                "world/car_body",
                &rerun::Boxes3D::from_centers_and_half_sizes(
                    [(position.x, position.y, position.z)],
                    [(half_vehicle_length, half_vehicle_width, half_vehicle_height)],
                )
                .with_quaternions([rerun::Quaternion::from_xyzw([
                    quat.x, quat.y, quat.z, quat.w,
                ])]),
            )
            .unwrap();

            for (ind, (wheel_id, wheel_radius, wheel_constraint)) in wheel_ids.iter().enumerate() {
                let position = body_interface.center_of_mass_position(*wheel_id);
                // the orientation of a cylinder in Jolt and in rerun are not the same
                let quat = body_interface.rotation(*wheel_id);
                let rerun_quat =
                    rerun::external::glam::Quat::from_xyzw(quat.x, quat.y, quat.z, quat.w)
                        * rerun::external::glam::Quat::from_euler(
                            rerun::external::glam::EulerRot::XYZ,
                            std::f32::consts::FRAC_PI_2,
                            0.0,
                            0.0, // std::f32::consts::FRAC_PI_2,
                        );

                // TODO(lucasw) put all the wheels together into one rec.log?
                rec.log(
                    format!("world/wheel{ind}"),
                    &rerun::Cylinders3D::from_lengths_and_radii(
                        [half_wheel_width * 2.0],
                        [*wheel_radius],
                    )
                    .with_centers([rerun::external::glam::vec3(
                        position.x, position.y, position.z,
                    )])
                    .with_quaternions([rerun_quat]),
                )
                .unwrap();
            }

            physics_system.update(delta_time, collision_steps, temp_allocator, job_system);
        }

        // TEMPORARY: test out safe shapecasting API
        let narrow_phase = physics_system.narrow_phase_query();

        let mut collector = ClosestHitCastShapeCollector::new();
        narrow_phase.cast_shape(CastShapeArgs {
            shapecast: RShapeCast {
                shape: sphere_shape,
                scale: Vec3::ONE,
                center_of_mass_start: rmat44_translation(RVec3::new(-5.0, 0.0, 0.0).into_jolt()),
                direction: Vec3::new(10.0, 0.0, 0.0),
            },
            base_offset: RVec3::ZERO,
            settings: Default::default(),
            collector: Some(CastShapeCollectorImpl::new_borrowed(&mut collector)),
            broad_phase_layer_filter: None,
            object_layer_filter: None,
            body_filter: None,
            shape_filter: None,
        });

        println!("Hit: {}", collector.result.is_some());

        body_interface.remove_body(floor_id);
        body_interface.destroy_body(floor_id);

        body_interface.remove_body(sphere_id);
        body_interface.destroy_body(sphere_id);

        drop(physics_system);

        JPC_JobSystemThreadPool_delete(job_system);
        JPC_TempAllocatorImpl_delete(temp_allocator);
    }

    rolt::unregister_types();
    rolt::factory_delete();

    println!("Hello, world!");
}

#[test]
fn run_main() {
    main();
}
