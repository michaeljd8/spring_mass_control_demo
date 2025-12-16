#include "controller.hpp"

// Constructor
Controller::Controller(double target_velocity, double target_distance)
    : target_velocity(target_velocity), target_distance(target_distance) {}

// Setters
void Controller::set_target_velocity(double velocity) {
    target_velocity = velocity;
}

void Controller::set_target_distance(double distance) {
    target_distance = distance;
}

// Getters
double Controller::get_target_velocity() const {
    return target_velocity;
}

double Controller::get_target_distance() const {
    return target_distance;
}

// Control logic
double Controller::compute_control_signal(double current_velocity, double current_position) {
    // Placeholder logic for control signal computation
    double error_velocity = target_velocity - current_velocity;
    double error_position = target_distance - current_position;

    // Simple proportional control (for demonstration purposes)
    double control_signal = 0.1 * error_velocity + 0.01 * error_position;
    return control_signal;
}