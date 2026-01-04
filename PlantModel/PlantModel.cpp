#include "PlantModel.hpp"
#include <cmath>
#include <algorithm>

PlantModel::PlantModel(double m, double k, double c, double b, double fc)
    : mass(m), spring_constant(k), damping(c), viscous_friction(b), coulomb_friction(fc), mass_position(0.0), mass_velocity(0.0) {}

void PlantModel::reset(double initial_position, double initial_velocity) {
    mass_position = initial_position;
    mass_velocity = initial_velocity;
}

double PlantModel::compute_acceleration(double x_in, double x_in_dot) const {
    double spring_force = spring_constant * (x_in - mass_position);
    double damper_force = damping * (x_in_dot - mass_velocity);
    double viscous_force = -viscous_friction * mass_velocity;

    double coulomb_force;
    if (std::abs(mass_velocity) > 1e-6) {
        coulomb_force = -coulomb_friction * sign(mass_velocity);
    } else {
        double net_force = spring_force + damper_force;
        coulomb_force = -std::clamp(net_force, -coulomb_friction, coulomb_friction);
    }

    return (spring_force + damper_force + viscous_force + coulomb_force) / mass;
}

void PlantModel::update(double drive_velocity, double dt) {
    drive_position += drive_velocity * dt;
    double acceleration = compute_acceleration(drive_position, drive_velocity);
    mass_velocity += acceleration * dt;
    mass_position += mass_velocity * dt;
}

double PlantModel::get_position() const {
    return mass_position;
}

double PlantModel::get_velocity() const {
    return mass_velocity;
}

double PlantModel::sign(double x) {
    return (x > 0) - (x < 0);
}