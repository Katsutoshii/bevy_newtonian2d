//! Example to demonstrate using the system to simulate bouncing balls.
use bevy::{
    DefaultPlugins,
    app::{App, FixedUpdate, Startup},
    asset::Assets,
    camera::{Camera2d, ClearColor},
    color::{
        Color,
        palettes::css::{BLUE, GREEN, RED},
    },
    ecs::{
        component::Component,
        entity::Entity,
        query::With,
        schedule::IntoScheduleConfigs,
        system::{Commands, Query, ResMut},
    },
    math::{Vec2, primitives::Circle},
    mesh::{Mesh, Mesh2d},
    sprite_render::{ColorMaterial, MeshMaterial2d},
    state::state::NextState,
    utils::default,
};
use bevy_newtonian2d::{
    CircleCollider, Force2, PhysicsMaterial, PhysicsPlugin, PhysicsSimulationState, PhysicsSystem,
    Position2, Velocity2,
};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugin))
        .insert_resource(ClearColor(Color::BLACK))
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (fixed_update, fixed_update_bounce_off_floor)
                .chain()
                .before(PhysicsSystem::ApplyForces),
        )
        .run();
}

#[derive(Component, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
#[require(PhysicsMaterial {
    friction: 0.01,
    ..default()
})]
struct Ball;

/// Spawn balls
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut physics_state: ResMut<NextState<PhysicsSimulationState>>,
) {
    commands.spawn(Camera2d);
    let circle_mesh = meshes.add(Circle { radius: 10.0 });
    commands.spawn((
        Ball,
        Position2::new(0.0, 200.0),
        Mesh2d(circle_mesh.clone()),
        MeshMaterial2d(materials.add(ColorMaterial::from_color(GREEN))),
        CircleCollider { radius: 10.0 },
    ));
    commands.spawn((
        Ball,
        Position2::new(5.0, 220.0),
        Mesh2d(circle_mesh.clone()),
        MeshMaterial2d(materials.add(ColorMaterial::from_color(RED))),
        CircleCollider { radius: 10.0 },
    ));
    commands.spawn((
        Ball,
        Position2::new(-15.0, 100.0),
        Mesh2d(circle_mesh.clone()),
        MeshMaterial2d(materials.add(ColorMaterial::from_color(BLUE))),
        CircleCollider { radius: 10.0 },
    ));
    physics_state.set(PhysicsSimulationState::Running);
}

/// Apply gravity and collision forces.
fn fixed_update(
    mut balls: Query<(Entity, &Position2, &CircleCollider, &mut Force2), With<Ball>>,
    other_balls: Query<(Entity, &Position2, &CircleCollider), With<Ball>>,
) {
    let gravity = 0.1;
    for (entity, position, collider, mut force) in balls.iter_mut() {
        *force += Force2(-Vec2::Y) * gravity;

        for (other_entity, other_position, other_collider) in other_balls.iter() {
            if entity == other_entity {
                continue;
            }
            let delta = position.0 - other_position.0;
            let distance_squared = delta.length_squared();
            if collider.is_colliding(*other_collider, distance_squared) {
                *force += Force2(delta.normalize_or_zero());
            }
        }
    }
}

/// Bound off of the floor.
fn fixed_update_bounce_off_floor(
    mut balls: Query<(&mut Position2, &Velocity2, &CircleCollider, &mut Force2), With<Ball>>,
) {
    let floor = 0.0;
    for (mut position, velocity, collider, mut force) in balls.iter_mut() {
        if position.0.y < floor + collider.radius {
            position.y = floor + collider.radius;
            *force += Force2(Vec2::Y) * velocity.0.y.abs() * 2.0;
        }
    }
}
