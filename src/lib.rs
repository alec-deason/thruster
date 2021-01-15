mod optimizer;

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bevy_rapier2d::{
    physics::{RapierConfiguration, RigidBodyHandleComponent},
    rapier::{
        dynamics::RigidBodySet,
        math::{Point, Vector},
    },
};

const CACHE_COARSENESS: f32 = std::f32::consts::PI / 10.0;

pub struct ThrusterPlugin;
impl Plugin for ThrusterPlugin {
    fn build(&self, app: &mut AppBuilder) {
        if !app.resources().contains::<ThrustScale>() {
            app.resources_mut().insert(ThrustScale::default());
        }
        app.register_type::<EngineSet>()
            .add_event::<EngineEvent>()
            .add_stage_after(stage::UPDATE, "update_steering", SystemStage::parallel())
            .add_system_to_stage("update_steering", invalidate_caches.system())
            .add_system_to_stage("update_steering", fire_engines.system());
    }
}

pub struct ThrustScale(pub f32);
impl Default for ThrustScale {
    fn default() -> Self {
        Self(1.0)
    }
}

#[derive(Copy, Clone, Serialize, Deserialize, Debug)]
pub struct Engine {
    pub offset: Vec2,
    pub thrust_vector: Vec2,
    pub max_thrust: f32,
}
impl Default for Engine {
    fn default() -> Self {
        Self {
            offset: Vec2::splat(0.0),
            thrust_vector: Vec2::new(0.0, 1.0),
            max_thrust: 1.0,
        }
    }
}
bevy::reflect::impl_reflect_value!(Engine);
#[derive(Reflect, Default, Debug)]
pub struct EngineSet(pub Vec<Engine>);

#[derive(Default)]
pub struct Steering {
    pub desired_force: Vec2,
    pub desired_torque: f32,
    last_seen_center_of_mass: Vec2,
    firings_cache: HashMap<(i32, i32, i32), Vec<f32>>,
    engines: Option<Vec<(Vec2, Vec2, f32, (Entity, usize))>>,
    currently_firing: HashSet<(Entity, usize)>,
}

impl Steering {
    pub fn clear_desire(&mut self) {
        self.desired_force = Vec2::splat(0.0);
        self.desired_torque = 0.0;
    }
}

fn fire_engines(
    thrust_scale: Res<ThrustScale>,
    rapier_config: Res<RapierConfiguration>,
    mut body_set: ResMut<RigidBodySet>,
    mut engine_events: ResMut<Events<EngineEvent>>,
    mut parent_query: Query<(
        Entity,
        &GlobalTransform,
        &mut Steering,
        &RigidBodyHandleComponent,
        Option<&Children>,
    )>,
    engine_query: Query<(&Transform, &EngineSet)>,
) {
    for (parent, parent_transform, mut steering, body_handle, maybe_children) in
        parent_query.iter_mut()
    {
        let mut just_fired = Vec::with_capacity(steering.currently_firing.len());
        if steering.desired_force != Vec2::splat(0.0) || steering.desired_torque != 0.0 {
            if let Some(body) = body_set.get_mut(body_handle.handle()) {
                if steering.engines.is_none() {
                    let mut entities = vec![parent];
                    if let Some(children) = maybe_children {
                        entities.extend(children.iter().copied());
                    }
                    entities.sort();
                    let mut engines = Vec::with_capacity(entities.len());
                    for e in entities {
                        if let Ok((transform, engine_set)) = engine_query.get(e) {
                            let transform = if e == parent {
                                Transform::identity()
                            } else {
                                *transform
                            };
                            for (i, engine) in engine_set.0.iter().enumerate() {
                                engines.push((
                                    transform.translation.truncate() + engine.offset,
                                    transform
                                        .rotation
                                        .mul_vec3(engine.thrust_vector.extend(0.0))
                                        .truncate(),
                                    engine.max_thrust,
                                    (e, i),
                                ));
                            }
                        }
                    }
                    steering.engines = Some(engines);
                }

                let center_of_mass = Vec2::new(body.world_com.x, body.world_com.y)
                    - parent_transform.translation.truncate();
                // TODO: This epsilon needs to depend on rapier scale? Or maybe be user configurable?
                if steering
                    .last_seen_center_of_mass
                    .distance_squared(center_of_mass)
                    > 0.5
                {
                    steering.last_seen_center_of_mass = center_of_mass;
                    steering.firings_cache.clear();
                }

                let key = (
                    (steering.desired_force.x / CACHE_COARSENESS) as i32,
                    (steering.desired_force.y / CACHE_COARSENESS) as i32,
                    (steering.desired_torque / CACHE_COARSENESS) as i32,
                );

                let Steering {
                    ref engines,
                    ref mut firings_cache,
                    desired_force,
                    desired_torque,
                    ..
                } = &mut *steering;
                let firing = firings_cache.entry(key).or_insert_with(|| {
                    optimizer::calculate_firing(
                        engines.as_ref().unwrap(),
                        center_of_mass,
                        *desired_force,
                        *desired_torque,
                    )
                });

                for ((position, thrust_vector, max_thrust, event_key), firing) in
                    engines.as_ref().unwrap().iter().zip(firing)
                {
                    if *firing > 0.0 {
                        just_fired.push((event_key.0, event_key.1, *firing));
                        let p = parent_transform.mul_vec3(position.extend(0.0));
                        let p = Point::new(p.x, p.y);
                        let thrust_vector = parent_transform
                            .rotation
                            .mul_vec3(thrust_vector.extend(0.0));
                        let thrust_vector = Vector::new(thrust_vector.x, thrust_vector.y);
                        body.apply_force_at_point(
                            thrust_vector
                                * *max_thrust
                                * thrust_scale.0
                                * (80000.0 / rapier_config.scale),
                            p,
                            true,
                        );
                    }
                }
            }
        }
        let mut new_current = HashSet::new();

        for (e, i, f) in just_fired {
            new_current.insert((e, i));
            if !steering.currently_firing.contains(&(e, i)) {
                engine_events.send(EngineEvent::StartedFiring(e, i, f));
            }
        }
        for (e, i) in steering.currently_firing.difference(&new_current) {
            engine_events.send(EngineEvent::StoppedFiring(*e, *i));
        }
        steering.currently_firing = new_current;
    }
}

fn invalidate_caches(
    parent_engines: Query<Entity, (Changed<EngineSet>, With<Steering>)>,
    child_engines: Query<&Parent, (Changed<EngineSet>, Without<Steering>)>,
    mut steering_query: Query<&mut Steering>,
) {
    let mut to_clear = HashSet::new();
    for entity in parent_engines.iter() {
        to_clear.insert(entity);
    }
    for parent in child_engines.iter() {
        to_clear.insert(**parent);
    }
    for entity in to_clear {
        if let Ok(mut steering) = steering_query.get_mut(entity) {
            steering.firings_cache.clear();
            steering.engines.take();
        }
    }
}

#[derive(Debug)]
pub enum EngineEvent {
    StartedFiring(Entity, usize, f32),
    StoppedFiring(Entity, usize),
}

impl EngineEvent {
    pub fn engine(&self) -> (Entity, usize) {
        match self {
            EngineEvent::StartedFiring(e, i, ..) | EngineEvent::StoppedFiring(e, i, ..) => (*e, *i),
        }
    }
}
