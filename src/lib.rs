//! Simple 2D Newtonian physics simulation for Bevy.
use std::ops::Mul;

use bevy::{
    app::{App, FixedUpdate, Plugin, Update},
    ecs::{
        component::Component,
        hierarchy::{ChildOf, Children},
        query::{With, Without},
        reflect::ReflectComponent,
        schedule::{InternedSystemSet, IntoScheduleConfigs, ScheduleConfigs, SystemSet},
        system::{Query, Res},
    },
    math::{FloatExt, Mat2, Quat, Vec2},
    reflect::Reflect,
    state::{app::AppExtStates, condition::in_state, state::States},
    time::{Fixed, Time},
    transform::{TransformSystems, components::Transform},
};

use derive_more::{Add, AddAssign, Deref, DerefMut, Sub, SubAssign};

/// Plugin for basic 2d physics.
#[derive(Default)]
pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<PhysicsMaterial>()
            .register_type::<Mass>()
            .register_type::<Velocity2>()
            .register_type::<Force2>()
            .register_type::<Position2>()
            .register_type::<Rotation2>()
            .register_type::<CircleCollider>()
            .init_state::<PhysicsSimulationState>()
            .configure_sets(FixedUpdate, PhysicsSystem::fixed_update_config())
            .configure_sets(Update, PhysicsSystem::update_config())
            .add_systems(
                Update,
                update
                    .in_set(PhysicsSystem::UpdateTransform)
                    .before(TransformSystems::Propagate),
            )
            .add_systems(
                FixedUpdate,
                (fixed_update_children, (fixed_update, fixed_update_angular))
                    .chain()
                    .in_set(PhysicsSystem::ApplyForces),
            );
    }
}

/// Tags entity as unable to move.
#[derive(Component, Debug, Default)]
pub struct Static;

/// Mass per entity, which inhibits Torque2 and Force2.
#[derive(
    Component,
    Debug,
    Clone,
    Copy,
    Deref,
    DerefMut,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    PartialEq,
    Reflect,
)]
#[reflect(Component)]
pub struct Mass(pub f32);
impl Default for Mass {
    fn default() -> Self {
        Self(1.0)
    }
}

/// Circle collider with a specified radius.
#[derive(Component, Debug, Clone, Copy, Deref, DerefMut, PartialEq, Reflect)]
#[reflect(Component)]
pub struct CircleCollider {
    pub radius: f32,
}
impl Default for CircleCollider {
    fn default() -> Self {
        Self { radius: 1.0 }
    }
}
impl CircleCollider {
    /// Returns true if there is a collision between two circle colliders.
    pub fn is_colliding(self, other: CircleCollider, distance_squared: f32) -> bool {
        let combined_radius = self.radius + other.radius;
        distance_squared <= combined_radius * combined_radius
    }
}

/// 2D position (translation).
#[derive(
    Component,
    Debug,
    Default,
    Clone,
    Copy,
    Deref,
    DerefMut,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    PartialEq,
    Reflect,
)]
pub struct Position2(pub Vec2);
impl Position2 {
    pub const ZERO: Self = Self(Vec2::ZERO);
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vec2::new(x, y))
    }
}

/// Linear 2D velocity vector.
#[derive(
    Component,
    Debug,
    Default,
    Clone,
    Copy,
    Deref,
    DerefMut,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    PartialEq,
    Reflect,
)]
pub struct Velocity2(pub Vec2);
impl Velocity2 {
    pub const ZERO: Self = Self(Vec2::ZERO);
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vec2::new(x, y))
    }
}
impl Mul<f32> for Velocity2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0.mul(rhs))
    }
}

/// 2D linear force vector accumulated this frame.
/// This is zeroed out each frame after being applied to Velocity2.
#[derive(
    Component,
    Debug,
    Default,
    Clone,
    Copy,
    Deref,
    DerefMut,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    PartialEq,
    Reflect,
)]
pub struct Force2(pub Vec2);
impl Force2 {
    pub const ZERO: Self = Self(Vec2::ZERO);
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vec2::new(x, y))
    }
}
impl Mul<f32> for Force2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0.mul(rhs))
    }
}

/// 2D Rotation.
/// This is stored as a complex number pair, e.g. cos(theta), i sin(theta).
#[derive(
    Component,
    Debug,
    Clone,
    Copy,
    Deref,
    DerefMut,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    PartialEq,
    Reflect,
)]
pub struct Rotation2(pub Vec2);
impl Default for Rotation2 {
    fn default() -> Self {
        Self(Vec2::X)
    }
}
impl Rotation2 {
    /// Creates a 2D vector containing [angle.cos(), angle.sin()].
    /// `angle` is expected in radians.
    pub fn from_angle(angle: f32) -> Self {
        Self(Vec2::from_angle(angle))
    }

    /// Rotates a Rotation2 by the angular velocity.
    pub fn rotate_dt(&mut self, angular_velocity: AngularVelocity2, dt: f32) {
        self.0 = Mat2::from_angle(angular_velocity.0 * dt) * self.0;
    }
}
impl Into<Quat> for Rotation2 {
    /// Converts a Rotation2 into a Quat.
    ///
    /// Rotation2 stores cos(θ), sin(θ).
    /// If we had cos(θ / 2), sin(θ / 2), we can trivially construct a Quat with:
    /// q = (0, 0, cos(θ / 2), sin(θ / 2))
    ///
    /// Based on double-angle trig identity for cos:
    /// cos(θ) = 2 cos^2(θ / 2) - 1
    /// (cos(θ) + 1) / 2 = cos^2(θ / 2)
    /// cos(θ / 2) = sqrt((cos(θ) + 1) / 2)
    ///
    /// sin(θ) = 2 sin(θ / 2) cos(θ / 2)
    /// sin(θ / 2) = sin(θ) / (2 * cos(θ / 2))
    fn into(self) -> Quat {
        // Special case: self.x ≈ -1 (θ ≈ π), avoid divide by zero.
        if (self.0.x + 1.0).abs() < f32::EPSILON {
            Quat::from_xyzw(0.0, 0.0, 1.0, 0.0)
        } else {
            let w = ((self.0.x + 1.0) * 0.5).sqrt();
            let z = self.0.y / (2.0 * w);
            Quat::from_xyzw(0.0, 0.0, z, w)
        }
    }
}

/// 2D angular velocity.
/// This is stored as a complex number pair, e.g. cos(theta), i sin(theta).
#[derive(
    Component,
    Default,
    Debug,
    Clone,
    Copy,
    Deref,
    DerefMut,
    PartialEq,
    Reflect,
    AddAssign,
    Add,
    Sub,
    SubAssign,
)]
pub struct AngularVelocity2(pub f32);
impl Mul<f32> for AngularVelocity2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0 * rhs)
    }
}

/// 2D torque.
/// This is stored as a complex number pair, e.g. cos(theta), i sin(theta).
#[derive(
    Component,
    Default,
    Debug,
    Clone,
    Copy,
    Deref,
    DerefMut,
    PartialEq,
    Reflect,
    AddAssign,
    Add,
    Sub,
    SubAssign,
)]
pub struct Torque2(pub f32);
impl Torque2 {
    pub fn towards(start: Rotation2, end: Rotation2) -> Self {
        Self(start.0.angle_to(end.0))
    }
}
impl Mul<f32> for Torque2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0 * rhs)
    }
}
impl Add<AngularVelocity2> for Torque2 {
    type Output = Self;
    fn add(self, rhs: AngularVelocity2) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}
impl Sub<AngularVelocity2> for Torque2 {
    type Output = Self;
    fn sub(self, rhs: AngularVelocity2) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

// Propagate position to transform.
// Apply smoothing based on overstep.
pub fn update(
    fixed_time: Res<Time<Fixed>>,
    mut query: Query<
        (
            &Position2,
            &Velocity2,
            &Rotation2,
            &AngularVelocity2,
            &mut Transform,
        ),
        Without<ChildOf>,
    >,
) {
    // Normalize so dt ~= 1.0 for standard 60 hz fixed update.
    let dt = fixed_time.delta_secs() * 60.0;
    let overstep_fraction = fixed_time.overstep_fraction();
    for (position, velocity, &rotation, angular_velocity, mut transform) in query.iter_mut() {
        let prev_position = position.0 - velocity.0 * dt;
        let smoothed_position = prev_position.lerp(position.0, overstep_fraction);
        transform.translation.x = smoothed_position.x;
        transform.translation.y = smoothed_position.y;

        let angle = rotation.0.to_angle();
        let prev_angle = angle - angular_velocity.0 * dt;
        let smooth_rotation = prev_angle.lerp(angle, overstep_fraction);
        transform.rotation = Quat::from_rotation_z(smooth_rotation);
    }
}

/// Integrate physics over time.
pub fn fixed_update(
    time: Res<Time>,
    mut query: Query<
        (
            &mut Position2,
            &mut Velocity2,
            &mut Force2,
            &Mass,
            &PhysicsMaterial,
        ),
        (Without<ChildOf>, Without<Static>),
    >,
) {
    // Normalize so dt ~= 1.0 for standard 60 hz fixed update.
    let dt = time.delta_secs() * 60.0;
    for (mut position, mut velocity, mut force, mass, material) in &mut query {
        force.0 -= velocity.0 * material.friction;

        velocity.0 += force.0 * dt / (mass.0);
        velocity.0 = velocity.0.clamp_length_max(material.max_velocity);

        position.0 += velocity.0 * dt;

        *force = Force2::ZERO;
    }
}

/// Integrate angular physics over time.
pub fn fixed_update_angular(
    time: Res<Time>,
    mut query: Query<
        (
            &mut Rotation2,
            &mut AngularVelocity2,
            &mut Torque2,
            &Mass,
            &PhysicsMaterial,
        ),
        (Without<ChildOf>, Without<Static>),
    >,
) {
    // Normalize so dt ~= 1.0 for standard 60 hz fixed update.
    let dt = time.delta_secs() * 60.0;
    for (mut rotation, mut angular_velocity, mut torque, mass, material) in &mut query {
        torque.0 -= angular_velocity.0 * material.friction;

        angular_velocity.0 += torque.0 * dt / (mass.0);

        rotation.rotate_dt(*angular_velocity, dt);
        rotation.0 = Mat2::from_angle(angular_velocity.0 * dt) * rotation.0;
        if !rotation.0.is_normalized() {
            rotation.0 = rotation.0.normalize();
        }
        *torque = Torque2(0.0);
    }
}

/// For simulated objects that are parented, apply child forces on the parent.
/// Update child velocity so it can be read elsewhere.
pub fn fixed_update_children(
    mut parents_query: Query<(&Velocity2, &mut Force2, &Children), Without<ChildOf>>,
    mut children_query: Query<(&mut Position2, &mut Velocity2, &mut Force2), With<ChildOf>>,
) {
    for (velocity, mut force, children) in parents_query.iter_mut() {
        // Sum all child forces.
        let mut children_force = Force2::ZERO;
        let mut num_children = 0;
        for child in children.iter() {
            if let Ok((mut child_position, mut child_velocity, mut child_force)) =
                children_query.get_mut(*child)
            {
                num_children += 1;
                children_force += *child_force;
                *child_velocity = *velocity;
                *child_force = Force2::ZERO;

                child_position.0 += velocity.0;
            }
        }
        if num_children > 0 {
            *force += children_force * (num_children as f32).recip();
        }
    }
}

/// Physics material defining common properties of the object.
#[derive(Clone, Reflect, Component, Debug, serde::Deserialize)]
#[reflect(Component)]
#[require(
    Position2,
    Velocity2,
    Force2,
    Rotation2,
    AngularVelocity2,
    Torque2,
    CircleCollider,
    Mass
)]
pub struct PhysicsMaterial {
    pub max_velocity: f32,
    pub friction: f32,
}
impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            max_velocity: 8.0,
            friction: 0.0,
        }
    }
}

/// Set enum for the systems relating to transform propagation
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub enum PhysicsSystem {
    ApplyForces,
    UpdateTransform,
}
impl PhysicsSystem {
    pub fn fixed_update_config() -> ScheduleConfigs<InternedSystemSet> {
        Self::ApplyForces.run_if(in_state(PhysicsSimulationState::Running))
    }
    pub fn update_config() -> ScheduleConfigs<InternedSystemSet> {
        Self::UpdateTransform.run_if(in_state(PhysicsSimulationState::Running))
    }
}

/// Physics simulation state.
/// Starts paused (for title screens etc.), so be sure to set it to running when your game begins.
#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum PhysicsSimulationState {
    /// Physics is paused.
    #[default]
    Paused,
    /// Physics is running.
    Running,
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use bevy::math::Quat;

    use crate::{AngularVelocity2, Rotation2, Torque2};

    #[test]
    fn test_quat() {
        let rotation = Rotation2::from_angle(PI);
        let quat: Quat = rotation.into();
        assert_eq!(quat.z, 1.0);
    }

    #[test]
    fn test_torque() {
        let mut theta = Rotation2::from_angle(0.0);
        let mut omega = AngularVelocity2(0.0);
        let tau = Torque2(PI / 2.0);
        let dt = 0.5;
        omega.0 += tau.0 * dt;
        theta.rotate_dt(omega, dt);
        assert!(theta.0.to_angle() > PI / 8.0 - f32::EPSILON);
        assert!(theta.0.to_angle() < PI / 8.0 + f32::EPSILON);
    }
}
