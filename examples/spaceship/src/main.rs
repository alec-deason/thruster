use bevy::{prelude::*, render::camera::Camera};
use bevy_prototype_lyon::{prelude::*, entity::ShapeBundle};
use bevy_rapier2d::{
    physics::{RapierConfiguration, RapierPhysicsPlugin, RigidBodyHandleComponent},
    rapier::{dynamics::{RigidBodyBuilder, RigidBodySet}, geometry::ColliderBuilder, math::{Vector, Isometry}},
};
use rand::prelude::*;

use thruster::{Engine, EngineEvent, EngineSet, Steering, ThrustScale, ThrusterPlugin};

fn main() {
    let mut app = App::build();
    app.insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .insert_resource(ThrustScale(200000.0))
        .add_plugin(ThrusterPlugin)
        .add_plugin(RapierPhysicsPlugin)
        .insert_resource(RapierConfiguration {
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
    mut commands: Commands,
    bodies: Res<RigidBodySet>,
    keyboard_input: Res<Input<KeyCode>>,
    player_query: Query<(Entity, &RigidBodyHandleComponent), With<Steering>>,
) {
    if keyboard_input.just_pressed(KeyCode::Space) {
        if let Some((entity, body_handle)) = player_query.iter().next() {
            commands.entity(entity).despawn_recursive();
            let pos = if let Some(body) = bodies.get(body_handle.handle()) {
                body.position().clone()
            } else {
                Isometry::identity()
            };
            let entity = commands
                .spawn_bundle((
                    RigidBodyBuilder::new_dynamic()
                        .linear_damping(0.9)
                        .angular_damping(0.9)
                        .position(pos),
                    Steering::default(),
                    Transform {
                        translation: Vec3::new(pos.translation.x, pos.translation.y, 0.0),
                        rotation: Quat::from_rotation_z(pos.rotation.angle()),
                        ..Default::default()
                    }
                ))
                .id();
            make_random_ship(entity, &mut commands);
        }
    }
}

fn make_random_ship(
    entity: Entity,
    commands: &mut Commands,
) {
    let mut rng = rand::thread_rng();

    let mut new_engines = vec![];
    let count = rng.gen_range(2..20);
    let mut a = std::f32::consts::PI / 2.0;
    let da = std::f32::consts::PI / count as f32;

    for _ in 0..count {
        let r = rng.gen_range(20.0..60.0) * (count as f32 / 10.0).max(1.0).powf(1.5);
        let x = a.cos() * r;
        let y = a.sin() * r;
        a += da;
        let engine_angle = if rng.gen::<f32>() < 0.5 {
            rng.gen::<f32>() * std::f32::consts::PI * 2.0
        } else {
            std::f32::consts::PI / 2.0
        };
        let thrust_vector = Vec2::new(engine_angle.cos(), engine_angle.sin()).normalize();
        new_engines.push(Engine {
            offset: Vec2::new(x, y),
            thrust_vector,
            max_thrust: 1.0,
        });
    }
    let mut reflected_engines = new_engines.clone();
    reflected_engines.reverse();
    for e in &mut reflected_engines {
        e.offset.x *= -1.0;
        e.thrust_vector.x *= -1.0;
    }
    new_engines.extend(reflected_engines.into_iter());

    make_ship_from_engines(entity, new_engines, commands);
}

fn make_ship_from_engines(
    entity: Entity,
    engines: Vec<Engine>,
    commands: &mut Commands,
) {
    let center: Vec2 = engines.iter().map(|e| &e.offset).sum::<Vec2>() / engines.len() as f32;
    let r = engines
        .iter()
        .map(|e| (e.offset.distance(center) * 1000.0) as i32)
        .max()
        .unwrap() as f32
        / 1000.0;
    add_engine_polygons(commands, entity, &engines);
    let shape = shapes::Polygon {
        points: engines
            .iter()
            .map(|e| (e.offset.x, e.offset.y).into())
            .collect(),
        closed: true,
    };
    commands
        .entity(entity)
        .insert_bundle(GeometryBuilder::build_as(
            &shape,
            ShapeColors::new(Color::rgb_u8(169, 39, 9)),
            DrawMode::Fill(FillOptions::default()),
            Transform::from_translation(Vec3::new(0.0, 0.0, 10.0)),
        ))
        .with_children(|parent| {
            let shape = shapes::Polygon {
                points: vec![
                    (center.x - 10.0, center.y - 20.0).into(),
                    (center.x + 10.0, center.y - 20.0).into(),
                    (center.x, center.y + 20.0).into(),
                ],
                closed: true,
            };
            parent.spawn_bundle(GeometryBuilder::build_as(
                &shape,
                ShapeColors::new(Color::rgb_u8(82, 16, 53)),
                DrawMode::Fill(FillOptions::default()),
                Transform::from_translation(Vec3::new(0.0, 0.0, 0.5)),
            ));
        })
        .insert_bundle((ColliderBuilder::ball(r), EngineSet(engines)));
}

fn add_engine_polygons(
    commands: &mut Commands,
    entity: Entity,
    engines: &[Engine],
) {
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
            .spawn_bundle(GeometryBuilder::build_as(
                &shape,
                ShapeColors::new(Color::rgb_u8(145, 34, 8)),
                DrawMode::Fill(FillOptions::default()),
                Transform {
                    translation: Vec3::new(engine.offset.x, engine.offset.y, 10.0),
                    rotation: engine_rotation,
                    ..Default::default()
                },
            ))
            .id();
        let indicator = commands
            .spawn_bundle(GeometryBuilder::build_as(
                &shapes::Circle {
                    radius: 5.0,
                    ..Default::default()
                },
                ShapeColors::new(Color::rgba(1.0, 0.0, 0.0, 0.0)),
                DrawMode::Fill(FillOptions::default()),
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
            ))
            .insert(EngineIndicator(i))
            .id();
        children.push(e);
        children.push(indicator);
    }
    commands.entity(entity).push_children(&children);
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands
        .spawn_bundle(NodeBundle {
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
                .spawn_bundle(TextBundle {
                    style: Style {
                        margin: Rect::all(Val::Px(5.0)),
                        ..Default::default()
                    },
                    text: Text {
                        sections: vec![TextSection {
                            value: "AWSD/Arrows to fly.".to_string(),
                            style: TextStyle {
                                font: asset_server.load("FiraSans-Bold.ttf"),
                                font_size: 40.0,
                                color: Color::rgb_u8(80, 155, 199),
                                ..Default::default()
                            },
                        }],
                        ..Default::default()
                    },
                    ..Default::default()
                });
            parent
                .spawn_bundle(TextBundle {
                    style: Style {
                        margin: Rect::all(Val::Px(5.0)),
                        ..Default::default()
                    },
                    text: Text {
                        sections: vec![TextSection {
                            value: "Space to randomize ship.".to_string(),
                            style: TextStyle {
                                font: asset_server.load("FiraSans-Bold.ttf"),
                                font_size: 40.0,
                                color: Color::rgb_u8(80, 155, 199),
                                ..Default::default()
                            },
                        }],
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

    commands
        .spawn_bundle(OrthographicCameraBundle::new_2d());
    commands
        .spawn_bundle(UiCameraBundle::default());
    let entity = commands
        .spawn_bundle((
            RigidBodyBuilder::new_dynamic()
                .linear_damping(0.9)
                .angular_damping(0.9),
            Steering::default(),
        ))
        .id();

    let engines = vec![
        Engine {
            offset: Vec2::new(-30.0, 0.0),
            thrust_vector: Vec2::new(0.0, 1.0),
            ..Default::default()
        },
        Engine {
            offset: Vec2::new(30.0, 0.0),
            thrust_vector: Vec2::new(0.0, 1.0),
            ..Default::default()
        },
        Engine {
            offset: Vec2::new(0.0, 60.0),
            thrust_vector: Vec2::new(0.1, 0.0),
            ..Default::default()
        },
        Engine {
            offset: Vec2::new(0.0, 60.0),
            thrust_vector: Vec2::new(-0.1, 0.0),
            ..Default::default()
        },
    ];
    make_ship_from_engines(entity, engines, &mut commands);
}

fn setup_backdrop(mut commands: Commands, mut materials: ResMut<Assets<ColorMaterial>>) {
    let material = materials.add(Color::rgb_u8(80, 155, 199).into());
    let umbra_material = materials.add(Color::rgba_u8(80, 155, 199, 100).into());
    for x in -20..21 {
        commands
            .spawn_bundle(GeometryBuilder::build_as(
                &shapes::Polygon {
                    points: vec![(0.0, -2000.0).into(), (0.0, 2000.0).into()],
                    closed: false,
                },
                ShapeColors::new(Color::rgba_u8(80, 155, 199, 100)),
                DrawMode::Stroke(StrokeOptions::default().with_line_width(6.0)),
                Transform::from_translation(Vec3::new(x as f32 * 100.0, 0.0, 0.0)),
            ));
        commands
            .spawn_bundle(GeometryBuilder::build_as(
                &shapes::Polygon {
                    points: vec![(0.0, -2000.0).into(), (0.0, 2000.0).into()],
                    closed: false,
                },
                ShapeColors::new(Color::rgb_u8(80, 155, 199)),
                DrawMode::Stroke(StrokeOptions::default().with_line_width(2.0)),
                Transform::from_translation(Vec3::new(x as f32 * 100.0, 0.0, 0.1)),
            ));
    }
    for y in -20..21 {
        commands
            .spawn_bundle(GeometryBuilder::build_as(
                &shapes::Polygon {
                    points: vec![(-2000.0, 0.0).into(), (2000.0, 0.0).into()],
                    closed: false,
                },
                ShapeColors::new(Color::rgba_u8(80, 155, 199, 100)),
                DrawMode::Stroke(StrokeOptions::default().with_line_width(6.0)),
                Transform::from_translation(Vec3::new(0.0, y as f32 * 100.0, 0.0)),
            ));
        commands
            .spawn_bundle(GeometryBuilder::build_as(
                &shapes::Polygon {
                    points: vec![(-2000.0, 0.0).into(), (2000.0, 0.0).into()],
                    closed: false,
                },
                ShapeColors::new(Color::rgb_u8(80, 155, 199)),
                DrawMode::Stroke(StrokeOptions::default().with_line_width(2.0)),
                Transform::from_translation(Vec3::new(0.0, y as f32 * 100.0, 0.1)),
            ));
    }
}

fn camera_tracking(
    player_query: Query<&Transform, (With<Steering>, Without<Camera>)>,
    mut camera_query: Query<&mut Transform, With<Camera>>,
) {
    if let (Some(player), Some(mut camera)) =
        (player_query.iter().next(), camera_query.iter_mut().next())
    {
        let d = camera.translation.distance(player.translation).min(500.0) / 500.0;
        let t = 0.04 * d;
        let t = camera.translation * (1.0 - t) + player.translation * t;
        camera.translation.x = t.x;
        camera.translation.y = t.y;
    }
}

struct EngineIndicator(usize);
fn maintain_engine_indicators(
    mut events: EventReader<EngineEvent>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    engine_query: Query<&Children>,
    indicator_query: Query<(&Handle<ColorMaterial>, &EngineIndicator)>,
) {
    for event in events.iter() {
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
