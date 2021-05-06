mod optimizer;

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

use bevy::app::Events;
use bevy::prelude::*;
use bevy_rapier2d::{
    physics::{RapierConfiguration, RigidBodyHandleComponent},
    rapier::{
        dynamics::{RigidBody, RigidBodySet},
        math::{Point, Vector},
    },
};

const CACHE_COARSENESS: f32 = std::f32::consts::PI / 1000.0;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemLabel)]
pub enum SystemLabels {
    InvalidateCaches,
    FireEngines,
}

#[derive(Default)]
pub struct ThrusterPlugin;

impl Plugin for ThrusterPlugin {
    fn build(&self, app: &mut AppBuilder) {
        if !app.world().contains_resource::<ThrustScale>() {
            app.world_mut().insert_resource(ThrustScale::default());
        }
        let cache_system = invalidate_caches
            .system()
            .label(SystemLabels::InvalidateCaches);
        app.register_type::<EngineSet>()
            .add_event::<EngineEvent>()
            .add_system_to_stage(CoreStage::PostUpdate, cache_system)
            .add_system(
                fire_engines
                    .system()
                    .label(SystemLabels::FireEngines)
                    .after(SystemLabels::InvalidateCaches),
            );
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

    pub fn update_engine_cache(
        &mut self,
        parent: Entity,
        rapier_scale: f32,
        maybe_children: Option<&Children>,
        engine_query: &Query<(&Transform, &EngineSet)>,
    ) {
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
                        (transform.translation.truncate() + engine.offset) / rapier_scale,
                        transform
                            .rotation
                            .mul_vec3(engine.thrust_vector.extend(0.0))
                            .truncate()
                            .normalize(),
                        engine.max_thrust,
                        (e, i),
                    ));
                }
            }
        }
        self.engines = Some(engines);
    }

    pub fn estimate_acceleration(
        &mut self,
        body: &RigidBody,
        engine_scale: f32,
    ) -> Option<(Vec2, f32)> {
        let key = (
            (self.desired_force.x / CACHE_COARSENESS) as i32,
            (self.desired_force.y / CACHE_COARSENESS) as i32,
            (self.desired_torque / CACHE_COARSENESS) as i32,
        );
        let center_of_mass = body.mass_properties().local_com;
        let center_of_mass = Vec2::new(center_of_mass.x, center_of_mass.y);
        if self
            .last_seen_center_of_mass
            .distance_squared(center_of_mass)
            > 0.5
        {
            self.last_seen_center_of_mass = center_of_mass;
            self.firings_cache.clear();
        }
        let Steering {
            ref engines,
            ref mut firings_cache,
            desired_force,
            desired_torque,
            ..
        } = self;
        if !firings_cache.contains_key(&key) {
            if let Some(engines) = engines.as_ref() {
                firings_cache.insert(
                    key,
                    optimizer::calculate_firing(
                        engines,
                        center_of_mass,
                        *desired_force,
                        *desired_torque,
                    ),
                );
            } else {
                return None;
            }
        }
        let firing = firings_cache.get(&key).unwrap();
        Some(optimizer::estimate_acceleration(
            body.effective_world_inv_inertia_sqrt,
            body.effective_inv_mass,
            engine_scale,
            center_of_mass,
            self.engines.as_ref().unwrap(),
            &firing,
        ))
    }
}

fn fire_engines(
    thrust_scale: Res<ThrustScale>,
    rapier_config: Res<RapierConfiguration>,
    mut body_set: ResMut<RigidBodySet>,
    mut engine_events: ResMut<Events<EngineEvent>>,
    mut parent_query: Query<(
        Entity,
        &mut GlobalTransform,
        &mut Steering,
        &RigidBodyHandleComponent,
        Option<&Children>,
    )>,
    engine_query: Query<(&Transform, &EngineSet)>,
) {
    for (parent, mut parent_transform, mut steering, body_handle, maybe_children) in
        parent_query.iter_mut()
    {
        let mut just_fired = Vec::with_capacity(steering.currently_firing.len());
        if steering.desired_force != Vec2::splat(0.0) || steering.desired_torque != 0.0 {
            if let Some(body) = body_set.get_mut(body_handle.handle()) {
                if steering.engines.is_none() {
                    steering.update_engine_cache(
                        parent,
                        rapier_config.scale,
                        maybe_children,
                        &engine_query,
                    );
                }

                let center_of_mass = body.mass_properties().local_com;
                let center_of_mass = Vec2::new(center_of_mass.x, center_of_mass.y);
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
                        parent_transform.translation /= rapier_config.scale;
                        let p = parent_transform.mul_vec3(position.extend(0.0));
                        let p = Point::new(p.x, p.y);
                        let thrust_vector = parent_transform
                            .rotation
                            .mul_vec3(thrust_vector.extend(0.0));
                        let thrust_vector =
                            Vector::new(thrust_vector.x, thrust_vector.y).normalize();
                        body.apply_force_at_point(
                            thrust_vector * *max_thrust * thrust_scale.0,
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
