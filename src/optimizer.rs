use bevy::prelude::*;
use minilp::{ComparisonOp, OptimizationDirection, Problem};

pub(crate) fn estimate_acceleration(
    inverse_moment_of_inertia_sqrt: f32,
    inverse_mass: f32,
    engine_scale: f32,
    center_of_mass: Vec2,
    engines: &[(Vec2, Vec2, f32, (Entity, usize))],
    firing: &[f32],
) -> (Vec2, f32) {
    let mut acceleration = Vec2::ZERO;
    let mut angular_acceleration = 0.0;

    for ((engine_position, thrust_vector, max_thrust, _), firing_amount) in
        engines.iter().zip(firing)
    {
        if *firing_amount > 0.0 {
            let distance_to_com = *engine_position - center_of_mass;
            //let distance_to_com = (distance_to_com * 1000.0).round() / 1000.0;
            let thrust_vector = *thrust_vector * *max_thrust * *firing_amount * engine_scale;
            //let torque = distance_to_com.x*thrust_vector.y - distance_to_com.y  * thrust_vector.x;
            //let torque = (torque * 1000.0).round() / 1000.0;

            let torque = distance_to_com
                .extend(0.0)
                .cross(thrust_vector.extend(0.0))
                .z
                * -1.0;

            angular_acceleration +=
                inverse_moment_of_inertia_sqrt * (inverse_moment_of_inertia_sqrt * torque);
            acceleration += thrust_vector * inverse_mass;
        }
    }
    //((acceleration * 1000.0).round() / 1000.0, (angular_acceleration * 1000.0).round() / 1000.0)
    (acceleration, angular_acceleration)
}

pub(crate) fn calculate_firing(
    engines: &[(Vec2, Vec2, f32, (Entity, usize))],
    center_of_mass: Vec2,
    desired_force: Vec2,
    desired_torque: f32,
) -> Vec<f32> {
    let total_thrust: f32 = engines.iter().map(|e| e.2).sum::<f32>();
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
    let mut total_positive_torque = 0.0;
    let mut total_negative_torque = 0.0;

    let torque_weight = total_thrust * 10.0;
    let total_force_weight = 1.0;
    let fuel_consumption_weight = 0.0001;
    let desire = desired_force * total_thrust;

    for (engine_position, thrust_vector, max_thrust, _event_key) in engines {
        let distance_to_com = *engine_position - center_of_mass;
        let thrust_vector = thrust_vector.normalize() * *max_thrust;
        let torque = distance_to_com
            .extend(0.0)
            .cross(thrust_vector.extend(0.0))
            .z
            * torque_weight;
        let ev = thrust_vector * total_force_weight;
        let v = problem.add_var(fuel_consumption_weight, (0.0, 1.0));
        activations.push(v);
        torques.push(torque);
        if torque > 0.0 {
            total_positive_torque += torque.abs();
        } else {
            total_negative_torque += torque.abs();
        }
        forces.push(ev);
    }
    let desired_torque = if desired_torque > 0.0 {
        desired_torque * total_positive_torque
    } else {
        desired_torque * total_negative_torque
    };

    for (torque, (force, v)) in torques.iter().zip(forces.iter().zip(activations.iter())) {
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

    activations
        .into_iter()
        // FIXME: I am reducing precision here because the optimizer sometimes produces
        // results that are _very close_ but not quite right. It's possible that some
        // games will actualy need the extra precision and I should figure out what's
        // wrong with the optimizer anyway
        .map(|a| (solution[a] as f32 * 100.0).round() / 100.0)
        .collect()
}
