use bevy::{prelude::*, render::camera::Camera};
use bevy_prototype_lyon::prelude::*;
use bevy_rapier2d::{
    physics::{RapierConfiguration, RapierPhysicsPlugin},
    rapier::{dynamics::RigidBodyBuilder, geometry::ColliderBuilder, math::Vector},
};
use rand::prelude::*;

use thruster::{Engine, EngineEvent, Steering, SubEngine, ThrustScale, ThrusterPlugin};

fn main() {
    let mut app = App::build();
    app.add_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_resource(ThrustScale(3.0))
        .add_plugin(ThrusterPlugin)
        .add_plugin(RapierPhysicsPlugin)
        .add_resource(RapierConfiguration {
            gravity: Vector::zeros(),
            ..Default::default()
        });
    #[cfg(target_arch = "wasm32")]
    app.add_plugin(bevy_webgl2::WebGL2Plugin);
    app.add_system(player_controls.system())
        .add_system(randomize_player_ship.system())
        .add_system(maintain_engine_indicators.system())
        .add_system(camera_tracking.system())
        .add_startup_system(setup.system())
        .add_startup_system(setup_backdrop.system())
        .run();
}

fn player_controls(keyboard_input: Res<Input<KeyCode>>, mut steering_query: Query<&mut Steering>) {
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
            make_random_ship(entity, commands, &mut materials);
        }
    }
}

fn make_random_ship(
    entity: Entity,
    commands: &mut Commands,
    materials: &mut Assets<ColorMaterial>,
) {
    let mut rng = rand::thread_rng();

    let mut new_engines = vec![];
    let count = rng.gen_range(2..20);
    let mut a = std::f32::consts::PI / 2.0;
    let da = std::f32::consts::PI / count as f32;

    let engine_angles = [
        (std::f32::consts::PI / 2.0, 70.0),
        (-std::f32::consts::PI / 2.0, 10.0),
        (0.0, 10.0),
        (std::f32::consts::PI * 2.0, 10.0),
    ];

    for _ in 0..count {
        let r = rng.gen_range(20.0..60.0) * (count as f32 / 10.0).max(1.0).powf(1.5);
        let x = a.cos() * r;
        let y = a.sin() * r;
        a += da;
        let engine_angle = engine_angles
            .choose_weighted(&mut rng, |item| item.1)
            .unwrap()
            .0;
        let thrust_vector = Vec2::new(engine_angle.cos(), engine_angle.sin()).normalize();
        new_engines.push(SubEngine {
            offset: Vec2::new(x, y),
            thrust_vector,
            max_thrust: 1.0 * (4.0 / count as f32).min(0.2),
        });
    }
    let mut reflected_engines = new_engines.clone();
    reflected_engines.reverse();
    for e in &mut reflected_engines {
        e.offset.x *= -1.0;
        e.thrust_vector.x *= -1.0;
    }
    new_engines.extend(reflected_engines.into_iter());

    make_ship_from_engines(entity, new_engines, commands, materials);
}

fn make_ship_from_engines(
    entity: Entity,
    engines: Vec<SubEngine>,
    commands: &mut Commands,
    materials: &mut Assets<ColorMaterial>,
) {
    let ship_material = materials.add(Color::rgb_u8(169, 39, 9).into());
    let center: Vec2 = engines.iter().map(|e| &e.offset).sum::<Vec2>() / engines.len() as f32;
    let r = engines
        .iter()
        .map(|e| (e.offset.distance(center) * 1000.0) as i32)
        .max()
        .unwrap() as f32
        / 1000.0;
    add_engine_polygons(commands, entity, materials, &engines);
    commands.set_current_entity(entity);
    commands
        .with_bundle(
            shapes::Polygon {
                points: engines
                    .iter()
                    .map(|e| (e.offset.x, e.offset.y).into())
                    .collect(),
                closed: true,
            }
            .draw(
                ship_material,
                TessellationMode::Fill(FillOptions::default()),
                Transform::from_translation(Vec3::new(0.0, 0.0, 10.0)),
            ),
        )
        .with_children(|parent| {
            parent.spawn(
                shapes::Polygon {
                    points: vec![
                        (center.x - 10.0, center.y - 20.0).into(),
                        (center.x + 10.0, center.y - 20.0).into(),
                        (center.x, center.y + 20.0).into(),
                    ],
                    closed: true,
                }
                .draw(
                    materials.add(Color::rgb_u8(82, 16, 53).into()),
                    TessellationMode::Fill(FillOptions::default()),
                    Transform::from_translation(Vec3::new(0.0, 0.0, 0.5)),
                ),
            );
        })
        .with_bundle((ColliderBuilder::ball(r), Engine(engines)));
}

fn add_engine_polygons(
    commands: &mut Commands,
    entity: Entity,
    materials: &mut Assets<ColorMaterial>,
    engines: &[SubEngine],
) {
    let material = materials.add(Color::rgb_u8(145, 34, 8).into());
    let mut children = Vec::with_capacity(engines.len());
    for (i, engine) in engines.iter().enumerate() {
        let shape = shapes::Polygon {
            points: vec![
                (0.0, 0.0).into(),
                (-10.0, -15.0).into(),
                (10.0, -15.0).into(),
            ],
            closed: true,
        };
        let engine_rotation = Quat::from_rotation_z(
            engine.thrust_vector.y.atan2(engine.thrust_vector.x) - std::f32::consts::PI / 2.0,
        );
        let indicator_offset = engine_rotation.mul_vec3(Vec3::new(0.0, -15.0, 0.0));
        let e = commands
            .spawn(shape.draw(
                material.clone(),
                TessellationMode::Fill(FillOptions::default()),
                Transform {
                    translation: Vec3::new(engine.offset.x, engine.offset.y, 10.0),
                    rotation: engine_rotation,
                    ..Default::default()
                },
            ))
            .current_entity()
            .unwrap();
        let indicator = commands
            .spawn(
                shapes::Circle {
                    radius: 5.0,
                    ..Default::default()
                }
                .draw(
                    materials.add(Color::rgba(1.0, 0.0, 0.0, 0.0).into()),
                    TessellationMode::Fill(FillOptions::default()),
                    Transform {
                        translation: Vec3::new(
                            engine.offset.x + indicator_offset.x,
                            engine.offset.y + indicator_offset.y,
                            9.0,
                        ),
                        rotation: Quat::from_rotation_z(
                            engine.thrust_vector.y.atan2(engine.thrust_vector.x)
                                - std::f32::consts::PI / 2.0,
                        ),
                        ..Default::default()
                    },
                ),
            )
            .with(EngineIndicator(i))
            .current_entity()
            .unwrap();
        children.push(e);
        children.push(indicator);
    }
    commands.push_children(entity, &children);
}

fn setup(
    commands: &mut Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands
        .spawn(NodeBundle {
            material: materials.add(Color::rgb_u8(9, 7, 67).into()),
            style: Style {
                padding: Rect::all(Val::Px(20.0)),
                align_self: AlignSelf::FlexStart,
                align_items: AlignItems::FlexStart,
                flex_direction: FlexDirection::ColumnReverse,
                ..Default::default()
            },
            ..Default::default()
        })
        .with_children(|parent| {
            parent
                .spawn(TextBundle {
                    style: Style {
                        margin: Rect::all(Val::Px(5.0)),
                        ..Default::default()
                    },
                    text: Text {
                        value: "AWSD/Arrows to fly.".to_string(),
                        font: asset_server.load("FiraSans-Bold.ttf"),
                        style: TextStyle {
                            font_size: 40.0,
                            color: Color::rgb_u8(80, 155, 199),
                            ..Default::default()
                        },
                        ..Default::default()
                    },
                    ..Default::default()
                })
                .spawn(TextBundle {
                    style: Style {
                        margin: Rect::all(Val::Px(5.0)),
                        ..Default::default()
                    },
                    text: Text {
                        value: "Space to randomize ship.".to_string(),
                        font: asset_server.load("FiraSans-Bold.ttf"),
                        style: TextStyle {
                            font_size: 40.0,
                            color: Color::rgb_u8(80, 155, 199),
                            ..Default::default()
                        },
                        ..Default::default()
                    },
                    ..Default::default()
                });
        });

    #[cfg(target_arch = "wasm32")]
    commands.insert_resource(ClearColor(Color::rgb(
        9.0 / 256.0,
        7.0 / 256.0,
        67.0 / 256.0,
    )));
    #[cfg(not(target_arch = "wasm32"))]
    commands.insert_resource(ClearColor(Color::rgb_linear(
        9.0 / 256.0,
        7.0 / 256.0,
        67.0 / 256.0,
    )));

    let entity = commands
        .spawn(Camera2dBundle::default())
        .spawn(CameraUiBundle::default())
        .spawn((
            RigidBodyBuilder::new_dynamic()
                .linear_damping(0.9)
                .angular_damping(0.9),
            Steering::default(),
        ))
        .current_entity()
        .unwrap();

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
    make_ship_from_engines(entity, engines, commands, &mut materials);
}

fn setup_backdrop(commands: &mut Commands, mut materials: ResMut<Assets<ColorMaterial>>) {
    let material = materials.add(Color::rgb_u8(80, 155, 199).into());
    let umbra_material = materials.add(Color::rgba_u8(80, 155, 199, 100).into());
    for x in -20..21 {
        commands
            .spawn(
                shapes::Polygon {
                    points: vec![(0.0, -2000.0).into(), (0.0, 2000.0).into()],
                    closed: false,
                }
                .draw(
                    umbra_material.clone(),
                    TessellationMode::Stroke(StrokeOptions::default().with_line_width(6.0)),
                    Transform::from_translation(Vec3::new(x as f32 * 100.0, 0.0, 0.0)),
                ),
            )
            .spawn(
                shapes::Polygon {
                    points: vec![(0.0, -2000.0).into(), (0.0, 2000.0).into()],
                    closed: false,
                }
                .draw(
                    material.clone(),
                    TessellationMode::Stroke(StrokeOptions::default().with_line_width(2.0)),
                    Transform::from_translation(Vec3::new(x as f32 * 100.0, 0.0, 0.1)),
                ),
            );
    }
    for y in -20..21 {
        commands
            .spawn(
                shapes::Polygon {
                    points: vec![(-2000.0, 0.0).into(), (2000.0, 0.0).into()],
                    closed: false,
                }
                .draw(
                    umbra_material.clone(),
                    TessellationMode::Stroke(StrokeOptions::default().with_line_width(6.0)),
                    Transform::from_translation(Vec3::new(0.0, y as f32 * 100.0, 0.0)),
                ),
            )
            .spawn(
                shapes::Polygon {
                    points: vec![(-2000.0, 0.0).into(), (2000.0, 0.0).into()],
                    closed: false,
                }
                .draw(
                    material.clone(),
                    TessellationMode::Stroke(StrokeOptions::default().with_line_width(2.0)),
                    Transform::from_translation(Vec3::new(0.0, y as f32 * 100.0, 0.1)),
                ),
            );
    }
}

fn camera_tracking(
    player_query: Query<&Transform, With<Steering>>,
    mut camera_query: Query<&mut Transform, With<Camera>>,
) {
    if let (Some(player), Some(mut camera)) =
        (player_query.iter().next(), camera_query.iter_mut().next())
    {
        let t = camera.translation * 0.99 + player.translation * 0.01;
        camera.translation.x = t.x;
        camera.translation.y = t.y;
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
                        continue;
                    }
                    if let Some(material) = materials.get_mut(handle) {
                        match event {
                            EngineEvent::StartedFiring(_e, _i, amount) => {
                                material.color.set_a(*amount);
                            }
                            EngineEvent::StoppedFiring(..) => {
                                material.color.set_a(0.0);
                            }
                        }
                    }
                }
            }
        }
    }
}
