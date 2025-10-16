use std::time::Instant;

use nalgebra::{Isometry, Point3, UnitQuaternion, Vector3};
use crate::motor_lin::MotorLin;

use rapier3d::prelude::*;

/// Configuration for single motor.
#[derive(Debug, Clone)]
pub struct MotorParams {
    pub mass_moment: Real,
    pub position: Vector3<Real>,
    pub normal: Vector3<Real>,
    pub time_constant: Real,
    pub motor_map: (f32, f32),
}

impl MotorParams {
    // Torque computation from input squared velocities
    pub fn body_force_and_torque(&self, thrust_force: Real) -> (Vector3<Real>, Vector3<Real>) {
        // Convert motor speed to thrust using the motor's thrust lookup table
        let motor_normal = self.normal.normalize();

        // Force contribution: thrust magnitude in the direction of motor normal
        let motor_force = motor_normal * thrust_force;

        // Torque contribution: cross product of position vector and force
        let position_torque = self.position.cross(&motor_force);

        // Reaction torque from motor spinning (around motor normal axis)
        let reaction_torque = motor_normal * thrust_force * self.mass_moment;

        // Return computed force and torques
        (motor_force, position_torque + reaction_torque)
    }
}

#[derive(Debug, Clone)]
pub struct VehicleParams {
    /// The total mass of the vehicle
    pub mass: f32,
    /// The local center of mass of the vehicle
    pub local_com: Point3<Real>,
    /// The angular inertial along the principal axes
    pub principal_inertia: Vector3<Real>,
    /// Linear velocity damping (air resistance)
    pub lin_damp: f32,
    /// Angular velocity damping (air resistance)
    pub ang_damp: f32,
    /// Motors configuration
    pub motors: Vec<MotorParams>,
}

/// The physical initial condition of the drone within a simulation.
#[derive(Debug, Clone, Default)]
pub struct Initial {
    pub rotation: Vector3<f32>,
    pub position: Vector3<f32>,
    pub ang_velocity: Vector3<f32>,
    pub lin_velocity: Vector3<f32>,
}

/// The current physical state of the drone within the simulation.
#[derive(Debug, Clone)]
pub struct VehicleState {
    /// Unit Quaternion representing the rotation of the drone
    pub rotation: UnitQuaternion<f32>,
    /// Position of the drone in the world frame
    pub position: Vector3<f32>,
    /// Velocity of the drone in the world frame
    pub velocity: Vector3<f32>,
    /// Acceleration of the drone in the body frame
    pub body_acc: Vector3<f32>,
    /// Angular velocity of the drone in the body frame
    pub body_gyr: Vector3<f32>,
    /// The dynamic state of the vehicles motors
    pub motors: Vec<MotorState>,
}

#[derive(Debug, Clone)]
pub struct MotorState {
    /// Force produced by the motor [N]
    pub force: f32,
    /// Commanded speed to the motor
    pub command: f32,
    /// Direction of rotation for motor
    pub direction: bool,
}

pub struct Simulation {
    pub(crate) start_time: Instant,
    pub(crate) state: VehicleState,
    pub(crate) params: VehicleParams,
    rb_handle: RigidBodyHandle,
    phys: RapierPhys,
}

#[derive(Default)]
struct RapierPhys {
    gravity: Vector3<f32>,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
}


impl RapierPhys {
    // Rigidbody handle
    pub fn get_rb_mut(&mut self, handle: RigidBodyHandle) -> &mut RigidBody {
        self.bodies.get_mut(handle).unwrap()
    }

    // Rigidbody handle
    pub fn get_rb_ref(&self, handle: RigidBodyHandle) -> &RigidBody {
        self.bodies.get(handle).unwrap()
    }
}

impl RapierPhys {
    pub fn step(&mut self, dt: f32) {
        // rapier physics engine step
        let physics_hooks = ();
        let events = ();
        self.integration_parameters.dt = dt;
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &physics_hooks,
            &events,
        )
    }
}

impl Simulation {
    pub fn new(vehicle: VehicleParams, initial: Initial) -> Self {
        let num_motors = vehicle.motors.len();

        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        // Create the drone's rigid body and colliders
        let drone_rb = RigidBodyBuilder::dynamic()
            .translation(initial.position.into())
            .rotation(initial.rotation.into())
            .angvel(initial.ang_velocity.into())
            .linvel(initial.lin_velocity.into())
            .build();

        let drone_collider = ColliderBuilder::cuboid(0.13, 0.13, 0.04)
            .mass_properties(MassProperties::new(
                vehicle.local_com,
                vehicle.mass,
                vehicle.principal_inertia,
            ))
            .build();

        let rb_handle = bodies.insert(drone_rb);
        colliders.insert_with_parent(drone_collider, rb_handle, &mut bodies);

        // Let the body hit the floor
        let floor = ColliderBuilder::cuboid(500., 500., 1.0)
            .translation(vector![0., 0., 1.0])
            .build();
        colliders.insert(floor);

        Simulation {
            start_time: Instant::now(),
            params: vehicle,
            rb_handle,
            state: VehicleState {
                rotation: UnitQuaternion::default(),
                position: Vector3::zeros(),
                velocity: Vector3::zeros(),
                body_acc: Vector3::zeros(),
                body_gyr: Vector3::zeros(),
                motors: vec![MotorState {
                    force: 0.0,
                    command: 0.0,
                    direction: false,
                }; num_motors],
            },
            phys: RapierPhys {
                bodies,
                colliders,
                gravity: vector![0.0, 0.0, 9.81],
                .. RapierPhys::default()
            },
        }
    }

    pub fn make_upright(&mut self) {
        let rb = self.phys.get_rb_mut(self.rb_handle);
        let z_rot = rb.rotation().euler_angles().2; // Preserve z-axis rotation
        let upright = Rotation::from_euler_angles(0.0, 0.0, z_rot);
        rb.set_rotation(upright, true);
    }

    pub fn reset_position(&mut self) {
        let rb = self.phys.get_rb_mut(self.rb_handle);
        rb.set_position(Isometry::identity(), true);
    }

    pub fn set_motors_cmd(&mut self, motors_cmd: &[f32]) {
        assert!(
            motors_cmd.len() == self.state.motors.len(),
            "Number of inputs and motors much match exactly"
        );

        for i in 0..motors_cmd.len() {
            self.state.motors[i].command = motors_cmd[i];
        }
    }

    pub fn set_motors_dir(&mut self, motors_dir: &[bool]) {
        assert!(
            motors_dir.len() == self.state.motors.len(),
            "Number of inputs and motors much match exactly"
        );

        for i in 0..motors_dir.len() {
            self.state.motors[i].direction = motors_dir[i];
        }
    }

    /// Update the simulation by applying the inputs and
    /// move the simulation forward a single time step.
    pub fn update(&mut self, dt: f32) {

        // Apply first-order filtering to input motor signals and
        // calculate per motor body-force and -torque contributions.
        let mut body_force = Vector3::zeros();
        let mut body_torque = Vector3::zeros();
        for (i, motor) in self.params.motors.iter().enumerate() {
            let map = MotorLin::new(motor.motor_map.0, motor.motor_map.1, 0.01, 1.0);
            let force = map.command_to_force(self.state.motors[i].command);
            let alpha = dt / (motor.time_constant + dt);
            self.state.motors[i].force *= 1.0 - alpha;
            self.state.motors[i].force += force * alpha;
            let thrust_force = self.state.motors[i].force;
            let (force, torque) = motor.body_force_and_torque(thrust_force);
            body_force += force;
            body_torque += torque;
        }

        let rb = self.phys.get_rb_mut(self.rb_handle);

        // Rotate into world frame
        let world_force = rb.rotation() * body_force;
        let world_torque = rb.rotation() * body_torque;

        // Apply forces and torques to rigid body
        rb.reset_forces(true);
        rb.reset_torques(true);
        rb.add_force(world_force, true);
        rb.add_torque(world_torque, true);

        // Get velocity prior to simulation step
        let rb = self.phys.get_rb_ref(self.rb_handle);
        let prev_world_linvel = rb.linvel().xyz();

        self.phys.step(dt);

        // Get velocity after simulation step
        let rb = self.phys.get_rb_ref(self.rb_handle);
        let world_linvel = rb.linvel().xyz();

        // Calculate world acceleration
        let world_accel = (world_linvel - prev_world_linvel).unscale(dt) - self.phys.gravity;

        // Update state from rigidbody simulation
        self.state.rotation = rb.rotation().clone();
        self.state.position = rb.translation().clone();
        self.state.velocity = rb.linvel().clone();
        self.state.body_gyr = rb.rotation().inverse() * rb.angvel();
        self.state.body_acc = rb.rotation().inverse() * world_accel;
    }
}
