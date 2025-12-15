#include "plant_model.hpp"
#include <cmath>
#include <algorithm>

PlantModel::PlantModel(double m, double k, double c, double b, double fc)
    : mass(m), spring_constant(k), damping(c), viscous_friction(b), coulomb_friction(fc), position(0.0), velocity(0.0) {}

void PlantModel::reset(double initial_position, double initial_velocity) {
    position = initial_position;
    velocity = initial_velocity;
}

double PlantModel::compute_acceleration(double x_in, double x_in_dot) const {
    double spring_force = spring_constant * (x_in - position);
    double damper_force = damping * (x_in_dot - velocity);
    double viscous_force = -viscous_friction * velocity;

    double coulomb_force;
    if (std::abs(velocity) > 1e-6) {
        coulomb_force = -coulomb_friction * sign(velocity);
    } else {
        double net_force = spring_force + damper_force;
        coulomb_force = -std::clamp(net_force, -coulomb_friction, coulomb_friction);
    }

    return (spring_force + damper_force + viscous_force + coulomb_force) / mass;
}

void PlantModel::update(double x_in, double x_in_dot, double dt) {
    double acceleration = compute_acceleration(x_in, x_in_dot);
    velocity += acceleration * dt;
    position += velocity * dt;
}

void PlantModel::update_rk4(double x_in, double x_in_dot, double dt) {
    double x0 = position;
    double v0 = velocity;

    double a1 = compute_acceleration(x_in, x_in_dot);
    double v1 = velocity;

    position = x0 + v1 * dt / 2;
    velocity = v0 + a1 * dt / 2;
    double a2 = compute_acceleration(x_in, x_in_dot);
    double v2 = velocity;

    position = x0 + v2 * dt / 2;
    velocity = v0 + a2 * dt / 2;
    double a3 = compute_acceleration(x_in, x_in_dot);
    double v3 = velocity;

    position = x0 + v3 * dt;
    velocity = v0 + a3 * dt;
    double a4 = compute_acceleration(x_in, x_in_dot);
    double v4 = velocity;

    position = x0 + (v1 + 2 * v2 + 2 * v3 + v4) * dt / 6;
    velocity = v0 + (a1 + 2 * a2 + 2 * a3 + a4) * dt / 6;
}

double PlantModel::get_position() const {
    return position;
}

double PlantModel::get_velocity() const {
    return velocity;
}

double PlantModel::sign(double x) {
    return (x > 0) - (x < 0);
}