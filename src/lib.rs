use std::collections::{HashMap, HashSet};
use serde::{Serialize, Deserialize};

use bevy::prelude::*;
use bevy_rapier2d::{
    physics::{EventQueue, RapierConfiguration, RapierPhysicsPlugin, RigidBodyHandleComponent},
    rapier::{
        dynamics::RigidBodySet,
        geometry::{ColliderSet, ContactEvent, InteractionGroups},
        math::{Point, Vector},
    },
};
use minilp::{ComparisonOp, OptimizationDirection, Problem};

const CACHE_COARSENESS: f32 = std::f32::consts::PI / 10.0;

pub struct ThrusterPlugin;
impl Plugin for ThrusterPlugin {
    fn build(&self, app: &mut AppBuilder) {
        if !app.resources().contains::<ThrustScale>() {
            app.resources_mut().insert(ThrustScale::default());
        }
        app
            .register_type::<Engine>()
            .add_stage_after(stage::UPDATE, "update_steering", SystemStage::parallel())
            .add_system_to_stage("update_steering", invalidate_caches.system())
            .add_system_to_stage("update_steering", steering.system());
    }
}

pub struct ThrustScale(pub f32);
impl Default for ThrustScale {
    fn default() -> Self {
        Self(8000.0)
    }
}

#[derive(Copy, Clone, Default, Serialize, Deserialize, Debug)]
pub struct SubEngine {
    pub thrust_vector: Vec2,
    pub max_thrust: f32,
}
bevy::reflect::impl_reflect_value!(SubEngine);
#[derive(Reflect, Default, Debug)]
pub struct Engine(pub Vec<SubEngine>);

#[derive(Default)]
pub struct Steering {
    pub desired_force: Vec2,
    pub desired_torque: f32,
    last_seen_com: Vec2,
    firings_cache: HashMap<(i32, i32, i32), Vec<f32>>,
    engines: Option<Vec<(Vec2, Vec2, f32)>>,
}

impl Steering {
    pub fn clear_desire(&mut self) {
        self.desired_force = Vec2::splat(0.0);
        self.desired_torque = 0.0;
    }
}

fn steering(
    thrust_scale: Res<ThrustScale>,
    mut body_set: ResMut<RigidBodySet>,
    mut parent_query: Query<(
        Entity,
        &GlobalTransform,
        &mut Steering,
        &RigidBodyHandleComponent,
        Option<&Children>,
    )>,
    engine_query: Query<(&Transform, &Engine)>,
) {
    for (parent, parent_transform, mut steering, body_handle, maybe_children) in
        parent_query.iter_mut()
    {
        if steering.desired_force != Vec2::splat(0.0) || steering.desired_torque != 0.0 {
            if let Some(body) = body_set.get_mut(body_handle.handle()) {
                if steering.engines.is_none() {
                    let mut entities = vec![parent];
                    if let Some(children) = maybe_children {
                        entities.extend(children.iter().copied());
                    }
                    entities.sort();
                    let mut engines = Vec::with_capacity(entities.len());
                    let mut total_thrust = 0.0;
                    for e in entities {
                        if let Ok((transform, engine)) = engine_query.get(e) {
                            let transform = if e == parent {
                                Transform::identity()
                            } else {
                                *transform
                            };
                            for sub_engine in &engine.0 {
                                engines.push((
                                    transform.translation.truncate(),
                                    transform
                                        .rotation
                                        .mul_vec3(sub_engine.thrust_vector.extend(0.0))
                                        .truncate(),
                                    sub_engine.max_thrust,
                                ));
                            }
                        }
                    }
                    steering.engines = Some(engines);
                }

                let center_of_mass = Vec2::new(body.world_com.x, body.world_com.y)
                    - parent_transform.translation.truncate();
                // TODO: This epsilon needs to depend on rapier scale? Or maybe be user configurable?
                if steering.last_seen_com.distance_squared(center_of_mass) > 0.5 {
                    steering.last_seen_com = center_of_mass;
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
                    let total_thrust:f32 = engines.as_ref().unwrap().iter().map(|e| e.2).sum::<f32>();
                    let mut problem = Problem::new(OptimizationDirection::Minimize);
                    let mut activations = vec![];
                    let mut torques = vec![];
                    let mut forces = vec![];

                    let mut torque_pos_constraint = vec![];
                    let mut torque_neg_constraint = vec![];
                    let mut force_x_pos_constraint = vec![];
                    let mut force_x_neg_constraint = vec![];
                    let mut force_y_pos_constraint = vec![];
                    let mut force_y_neg_constraint = vec![];

                    let desire_var = problem.add_var(0.0, (1.0, 1.0));
                    let u = problem.add_var(1.0, (f64::NEG_INFINITY, f64::INFINITY));
                    let v = problem.add_var(1.0, (f64::NEG_INFINITY, f64::INFINITY));
                    let w = problem.add_var(1.0, (f64::NEG_INFINITY, f64::INFINITY));
                    let mut total_torque = 0.0;

                    let torque_weight = 1.0;
                    let total_force_weight = 1.0;
                    let fuel_consumption_weight = 0.0001;
                    let desire = *desired_force * total_thrust;

                    for (engine_position, thrust_vector, max_thrust) in engines.as_ref().unwrap() {
                        let distance_to_com = *engine_position - center_of_mass;
                        let torque =
                            distance_to_com.extend(0.0).cross(thrust_vector.extend(0.0)).z * torque_weight;
                        let ev = *thrust_vector * total_force_weight;
                        let par = ((ev * desire) / desire.length_squared()) * desire;
                        let mut scale = desire.dot(par) / par.dot(par);
                        if scale.is_nan() {
                            scale = 0.0;
                        }
                        let v = problem.add_var(fuel_consumption_weight, (0.0, 1.0));
                        activations.push(v);
                        torques.push(torque);
                        total_torque += torque.abs();
                        forces.push(ev);
                    }
                    let desired_torque = *desired_torque * total_torque;

                    for (torque, (force, v)) in
                        torques.iter().zip(forces.iter().zip(activations.iter()))
                    {
                        torque_pos_constraint.push((*v, *torque as f64));
                        torque_neg_constraint.push((*v, -torque as f64));

                        force_x_pos_constraint.push((*v, force.x as f64));
                        force_x_neg_constraint.push((*v, -force.x as f64));
                        force_y_pos_constraint.push((*v, force.y as f64));
                        force_y_neg_constraint.push((*v, -force.y as f64));
                    }
                    torque_pos_constraint.push((u, -1.0));
                    torque_pos_constraint.push((desire_var, -desired_torque as f64));
                    torque_neg_constraint.push((u, -1.0));
                    torque_neg_constraint.push((desire_var, desired_torque as f64));
                    problem.add_constraint(&torque_pos_constraint, ComparisonOp::Le, 0.0);
                    problem.add_constraint(&torque_neg_constraint, ComparisonOp::Le, 0.0);

                    force_x_pos_constraint.push((v, -1.0));
                    force_x_pos_constraint.push((desire_var, -desire.x as f64));
                    force_x_neg_constraint.push((v, -1.0));
                    force_x_neg_constraint.push((desire_var, desire.x as f64));
                    problem.add_constraint(&force_x_pos_constraint, ComparisonOp::Le, 0.0);
                    problem.add_constraint(&force_x_neg_constraint, ComparisonOp::Le, 0.0);
                    force_y_pos_constraint.push((w, -1.0));
                    force_y_pos_constraint.push((desire_var, -desire.y as f64));
                    force_y_neg_constraint.push((w, -1.0));
                    force_y_neg_constraint.push((desire_var, desire.y as f64));
                    problem.add_constraint(&force_y_pos_constraint, ComparisonOp::Le, 0.0);
                    problem.add_constraint(&force_y_neg_constraint, ComparisonOp::Le, 0.0);
                    let solution = problem.solve().unwrap();

                    activations.into_iter().map(|a| solution[a] as f32).collect()
                });

                for ((position, thrust_vector, max_thrust), firing) in
                    engines.as_ref().unwrap().iter().zip(firing)
                {
                    if *firing > 0.0 {
                        let p = parent_transform.mul_vec3(position.extend(0.0));
                        let p = Point::new(p.x, p.y);
                        let thrust_vector = parent_transform
                            .rotation
                            .mul_vec3(thrust_vector.extend(0.0));
                        let thrust_vector = Vector::new(thrust_vector.x, thrust_vector.y);
                        body.apply_impulse_at_point(
                            thrust_vector * *max_thrust * thrust_scale.0,
                            p,
                            true,
                        );
                    }
                }
            }
        }
    }
}

fn invalidate_caches(
    parent_engines: Query<Entity, (Changed<Engine>, With<Steering>)>,
    child_engines: Query<&Parent, (Changed<Engine>, Without<Steering>)>,
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
