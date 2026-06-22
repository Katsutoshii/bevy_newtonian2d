//! Example to demonstrate using the system to simulate bouncing balls.
//! `cargo run --example bouncing_balls`
use bevy::{
    DefaultPlugins,
    app::{App, FixedUpdate, Startup},
    asset::{DirectAssetAccessExt, Handle},
    camera::{Camera2d, ClearColor},
    color::{
        Color, Srgba,
        palettes::css::{BLUE, GREEN, RED},
    },
    dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin, FrameTimeGraphConfig},
    ecs::{
        component::Component,
        lifecycle::HookContext,
        query::With,
        resource::Resource,
        schedule::IntoScheduleConfigs,
        system::{Commands, Query, ResMut},
        world::{DeferredWorld, FromWorld, World},
    },
    math::{Vec2, Vec3, primitives::Circle},
    mesh::{Mesh, Mesh2d},
    sprite_render::{ColorMaterial, MeshMaterial2d},
    state::state::NextState,
    transform::components::Transform,
    utils::default,
};
use bevy_newtonian2d::{
    CircleCollider, Force2, PhysicsMaterial, PhysicsPlugin, PhysicsSimulationState, PhysicsSystem,
    Position2, Velocity2,
};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugin,
            FpsOverlayPlugin {
                config: FpsOverlayConfig {
                    frame_time_graph_config: FrameTimeGraphConfig {
                        enabled: false,
                        ..default()
                    },
                    ..default()
                },
            },
        ))
        .init_resource::<BallAssets>()
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

#[derive(Resource, Clone)]
struct BallAssets {
    red_material: Handle<ColorMaterial>,
    green_material: Handle<ColorMaterial>,
    blue_material: Handle<ColorMaterial>,
    circle_mesh: Handle<Mesh>,
}
impl FromWorld for BallAssets {
    fn from_world(world: &mut World) -> Self {
        Self {
            red_material: world.add_asset(ColorMaterial::from_color(RED)),
            green_material: world.add_asset(ColorMaterial::from_color(GREEN)),
            blue_material: world.add_asset(ColorMaterial::from_color(BLUE)),
            circle_mesh: world.add_asset(Circle { radius: 1.0 }),
        }
    }
}

#[derive(Component, PartialEq, Clone, Copy)]
#[require(PhysicsMaterial {
    friction: 0.01,
    ..default()
})]
#[component(on_add = Ball::on_add)]
struct Ball {
    radius: f32,
    color: Srgba,
}
impl Ball {
    fn on_add(mut world: DeferredWorld, context: HookContext) {
        let BallAssets {
            red_material,
            green_material,
            blue_material,
            circle_mesh,
        } = world.resource::<BallAssets>().clone();
        let ball = world.get::<Self>(context.entity).unwrap().clone();
        let material = match ball.color {
            RED => red_material,
            GREEN => green_material,
            BLUE => blue_material,
            _ => unreachable!(),
        };
        world.commands().entity(context.entity).insert((
            Mesh2d(circle_mesh),
            MeshMaterial2d(material),
            CircleCollider {
                radius: ball.radius,
            },
            Transform {
                scale: Vec3::splat(ball.radius),
                ..default()
            },
        ));
    }
}

/// Spawn balls
fn setup(mut commands: Commands, mut physics_state: ResMut<NextState<PhysicsSimulationState>>) {
    commands.spawn(Camera2d);
    let x_step = 2.0;
    let y_step = 2.0;
    let n = 96;
    for y in -n..n {
        for x in -n..n {
            commands.spawn((
                Ball {
                    radius: 1.0,
                    color: GREEN,
                },
                Position2::new(x_step * x as f32, 200.0 + y_step * y as f32),
            ));
        }
    }
    physics_state.set(PhysicsSimulationState::Running);
}

/// Apply gravity and collision forces.
fn fixed_update(mut balls: Query<&mut Force2, With<Ball>>) {
    let gravity = 0.1;
    for mut force in balls.iter_mut() {
        *force += Force2(-Vec2::Y) * gravity;
    }
}

/// Bound off of the floor.
fn fixed_update_bounce_off_floor(
    mut balls: Query<(&mut Position2, &Velocity2, &CircleCollider, &mut Force2), With<Ball>>,
) {
    let floor = -200.0;
    for (mut position, velocity, collider, mut force) in balls.iter_mut() {
        if position.0.y < floor + collider.radius {
            position.y = floor + collider.radius;
            *force += Force2(Vec2::Y) * velocity.0.y.abs() * 2.0;
        }
    }
}
