// someday:
// #![forbid(unsafe_code)]

use joltc_sys::*;
use vehicle::*;

use rolt::{
    BroadPhaseLayer, BroadPhaseLayerInterface, CastShapeArgs, CastShapeCollectorImpl,
    ClosestHitCastShapeCollector, IntoJolt, ObjectLayer, ObjectLayerPairFilter,
    ObjectVsBroadPhaseLayerFilter, RShapeCast, RVec3, Vec3,
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

        let floor_half_extent = 100.0;
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
                Position: rvec3(0.0, 0.0, 3.0),
                MotionType: JPC_MOTION_TYPE_DYNAMIC,
                ObjectLayer: OL_MOVING,
                Shape: sphere_shape,
                ..Default::default()
            })
            .unwrap();
        let sphere_id = sphere.id();

        body_interface.add_body(sphere_id, JPC_ACTIVATION_ACTIVATE);
        body_interface.set_linear_velocity(sphere_id, Vec3::new(0.0, 0.0, 1.5));

        physics_system.optimize_broad_phase();

        let delta_time = 1.0 / 60.0;
        let collision_steps = 1;

        let mut step = 0;
        while body_interface.is_active(sphere_id) {
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
