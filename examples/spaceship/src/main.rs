use rand::prelude::*;
use bevy::{
    prelude::*,
    render::camera::Camera,
};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{
    physics::{RapierPhysicsPlugin, RapierConfiguration},
    rapier::{
        dynamics::RigidBodyBuilder,
        geometry::ColliderBuilder,
        math::Vector,
    }
};

use thruster::{Engine, SubEngine, Steering, ThrusterPlugin, ThrustScale, EngineEvent};

fn main() {
    let mut app = App::build();
    app.add_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_resource(ThrustScale(4000.0))
        .add_plugin(ThrusterPlugin)
        .add_plugin(RapierPhysicsPlugin)
        .add_resource(RapierConfiguration {
            gravity: Vector::zeros(),
            ..Default::default()
        });
    #[cfg(target_arch = "wasm32")]
    app.add_plugin(bevy_webgl2::WebGL2Plugin);
    app
        .add_system(player_controls.system())
        .add_system(randomize_player_ship.system())
        .add_system(maintain_engine_indicators.system())
        .add_system(camera_tracking.system())
        .add_startup_system(setup.system())
        .add_startup_system(setup_backdrop.system())
        .run();
}


fn player_controls(
    keyboard_input: Res<Input<KeyCode>>,
    mut steering_query: Query<&mut Steering>,
) {
    if let Some(mut steering) = steering_query.iter_mut().next() {
        if keyboard_input.pressed(KeyCode::W) || keyboard_input.pressed(KeyCode::Up) {
            steering.desired_force.y = 1.0;
        } else if keyboard_input.pressed(KeyCode::S) || keyboard_input.pressed(KeyCode::Down) {
            steering.desired_force.y = -1.0;
        } else {
            steering.desired_force.y = 0.0;
        }
        if keyboard_input.pressed(KeyCode::Q) {
            steering.desired_force.x = 1.0;
        } else if keyboard_input.pressed(KeyCode::E) {
            steering.desired_force.x = -1.0;
        } else {
            steering.desired_force.x = 0.0;
        }
        if keyboard_input.pressed(KeyCode::D) || keyboard_input.pressed(KeyCode::Right) {
            steering.desired_torque = -1.0;
        } else if keyboard_input.pressed(KeyCode::A) || keyboard_input.pressed(KeyCode::Left) {
            steering.desired_torque = 1.0;
        } else {
            steering.desired_torque = 0.0;
        }
    }
}

fn randomize_player_ship(
    commands: &mut Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    keyboard_input: Res<Input<KeyCode>>,
    player_query: Query<(Entity, &Children), With<Steering>>,
) {
    if keyboard_input.just_pressed(KeyCode::Space) {
        if let Some((entity, children)) = player_query.iter().next() {
            for child in children.iter() {
                commands.despawn_recursive(*child);
            }
            let mut rng = rand::thread_rng();

            let material = materials.add(Color::rgb(0.9, 0.01, 0.01).into());
            let mut new_engines = vec![];
            let count = rng.gen_range(3..10);
            let mut a = 0.0f32;
            let da = (std::f32::consts::PI*2.0) / count as f32;

            for _ in 0..count {
                let r = rng.gen_range(20.0..60.0);
                let x = a.cos() * r;
                let y = a.sin() * r;
                a += da;
                let engine_angle = if rng.gen::<f32>() < 0.25 {
                    -a
                } else {
                    0.0
                };
                let thrust_vector = Vec2::new(engine_angle.cos(), engine_angle.sin()).normalize();
                new_engines.push(
                    SubEngine {
                        offset: Vec2::new(x, y),
                        thrust_vector,
                        ..Default::default()
                    },
                );
            }
            let engine_material = materials.add(Color::rgb(0.4, 0.01, 0.01).into());
            add_engine_polygons(commands, entity, engine_material, &mut materials, &new_engines);
            commands
                .insert(entity, shapes::Polygon {
                    points: new_engines.iter().map(|e| (e.offset.x, e.offset.y).into()).collect(),
                    closed: true,
                }.draw(
                    material.clone(),
                    TessellationMode::Fill(FillOptions::default()),
                    Transform::default(),
                ))
                .with_children(|parent| {
                    parent
                        .spawn(
                            shapes::Polygon {
                                points: vec![(-10.0, 0.0).into(), (10.0, 0.0).into(), (0.0, 40.0).into()],
                                closed: true,
                                ..Default::default()
                            }
                            .draw(
                            materials.add(Color::rgb(0.0, 0.0, 1.0).into()),
                            TessellationMode::Fill(FillOptions::default()),
                            Transform::from_translation(Vec3::new(0.0, 0.0, 0.5))
                        ));
                })
                .insert_one(entity, Engine(new_engines));
        }
    }
}

fn add_engine_polygons(
    commands: &mut Commands,
    entity: Entity,
    material: Handle<ColorMaterial>,
    materials: &mut Assets<ColorMaterial>,
    sub_engines: &[SubEngine]
) {
    let mut children = Vec::with_capacity(sub_engines.len());
    for (i, engine) in sub_engines.iter().enumerate() {
        let shape = shapes::Polygon {
            points: vec![(0.0, 0.0).into(), (-20.0, -30.0).into(), (20.0, -30.0).into()],
            closed: true,
            ..Default::default()
        };
        let engine_rotation = Quat::from_rotation_z(engine.thrust_vector.y.atan2(engine.thrust_vector.x) - std::f32::consts::PI/2.0);
        let indicator_offset = engine_rotation.mul_vec3(Vec3::new(0.0, -30.0, 0.0));
        let e = commands
            .spawn(shape.draw(
                material.clone(),
                TessellationMode::Fill(FillOptions::default()),
                Transform {
                    translation: Vec3::new(engine.offset.x, engine.offset.y, 10.0),
                    rotation: engine_rotation,
                    ..Default::default()
                }
            ))
            .current_entity().unwrap();
        let indicator = commands
            .spawn(shapes::Circle {
                radius: 10.0,
                ..Default::default()
                }.draw(
                    materials.add(Color::rgba(1.0, 0.0, 0.0, 0.0).into()),
                    TessellationMode::Fill(FillOptions::default()),
                    Transform {
                        translation: Vec3::new(engine.offset.x + indicator_offset.x, engine.offset.y+indicator_offset.y, 9.0),
                        rotation: Quat::from_rotation_z(engine.thrust_vector.y.atan2(engine.thrust_vector.x) - std::f32::consts::PI/2.0),
                        ..Default::default()
                    }
            ))
            .with(EngineIndicator(i))
            .current_entity().unwrap();
        children.push(e);
        children.push(indicator);
    }
    commands.push_children(entity, &children);
}

fn setup(
    commands: &mut Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let material = materials.add(Color::rgb(0.9, 0.01, 0.01).into());


    let entity = commands
        .insert_resource(ClearColor(Color::rgb(0.001, 0.001, 0.011)))
        .spawn(Camera2dBundle::default())
        .spawn(
            shapes::Polygon {
                points: vec![(-30.0, 0.0).into(), (30.0, 0.0).into(), (0.0, 60.0).into()],
                closed: true,
                ..Default::default()
            }
            .draw(
            material.clone(),
            TessellationMode::Fill(FillOptions::default()),
            Transform::default()
        ))
        .with_children(|parent| {
            parent
                .spawn(
                    shapes::Polygon {
                        points: vec![(-10.0, 0.0).into(), (10.0, 0.0).into(), (0.0, 40.0).into()],
                        closed: true,
                        ..Default::default()
                    }
                    .draw(
                    materials.add(Color::rgb(0.0, 0.0, 1.0).into()),
                    TessellationMode::Fill(FillOptions::default()),
                    Transform::from_translation(Vec3::new(0.0, 0.0, 0.5))
                ));
        })
        .with(Steering::default())
        .with(RigidBodyBuilder::new_dynamic().linear_damping(0.9).angular_damping(0.99))
        .with(ColliderBuilder::ball(30.0))
        .current_entity().unwrap();

    let engines = vec![
        SubEngine {
            offset: Vec2::new(-30.0, 0.0),
            thrust_vector: Vec2::new(0.0, 1.0),
            ..Default::default()
        },
        SubEngine {
            offset: Vec2::new(30.0, 0.0),
            thrust_vector: Vec2::new(0.0, 1.0),
            ..Default::default()
        },
        SubEngine {
            offset: Vec2::new(0.0, 60.0),
            thrust_vector: Vec2::new(0.1, 0.0),
            ..Default::default()
        },
        SubEngine {
            offset: Vec2::new(0.0, 60.0),
            thrust_vector: Vec2::new(-0.1, 0.0),
            ..Default::default()
        },
    ];
    let engine_material = materials.add(Color::rgb(0.4, 0.01, 0.01).into());
    add_engine_polygons(commands, entity, engine_material, &mut materials, &engines);
    commands
        .insert_one(entity, Engine(engines));
}

fn setup_backdrop(
    commands: &mut Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let material = materials.add(Color::rgb(0.01, 0.9, 0.01).into());
    for x in -20..21 {
        commands
            .spawn(
                shapes::Polygon {
                    points: vec![(0.0, -2000.0).into(), (0.0, 2000.0).into()],
                    closed: false,
                }.draw(
                    material.clone(),
                    TessellationMode::Stroke(StrokeOptions::default().with_line_width(5.0)),
                    Transform::from_translation(Vec3::new(x as f32 * 100.0, 0.0, 0.0))
            ));
    }
    for y in -20..21 {
        commands
            .spawn(
                shapes::Polygon {
                    points: vec![(-2000.0, 0.0).into(), (2000.0, 0.0).into()],
                    closed: false,
                }.draw(
                    material.clone(),
                    TessellationMode::Stroke(StrokeOptions::default().with_line_width(5.0)),
                    Transform::from_translation(Vec3::new(0.0, y as f32 * 100.0, 0.0))
            ));
    }
}

fn camera_tracking(
    player_query: Query<&Transform, With<Steering>>,
    mut camera_query: Query<&mut Transform, With<Camera>>,
) {
    if let (Some(player), Some(mut camera)) = (player_query.iter().next(), camera_query.iter_mut().next()) {
        camera.translation = camera.translation * 0.99 + player.translation * 0.01;
    }
}

struct EngineIndicator(usize);
fn maintain_engine_indicators(
    mut reader: Local<EventReader<EngineEvent>>,
    events: Res<Events<EngineEvent>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    engine_query: Query<&Children>,
    indicator_query: Query<(&Handle<ColorMaterial>, &EngineIndicator)>,
) {
    for event in reader.iter(&events) {
        if let Ok(engine_children) = engine_query.get(event.engine().0) {
            for child in engine_children.iter() {
                if let Ok((handle, indicator)) = indicator_query.get(*child) {
                    if indicator.0 != event.engine().1 {
                        continue
                    }
                    if let Some(material) = materials.get_mut(handle) {
                        match event {
                            EngineEvent::StartedFiring(_e, _i, amount) => {
                                material.color.set_a(1.0);
                            },
                            EngineEvent::StoppedFiring(..) => {
                                material.color.set_a(0.0);
                            },
                        }
                    }
                }
            }
        }
    }
}
