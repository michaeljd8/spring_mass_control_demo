#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>

class Controller {
private:
    double target_velocity;
    double target_distance;

public:
    // Constructor
    Controller(double target_velocity = 0.0, double target_distance = 0.0);

    // Setters
    void set_target_velocity(double velocity);
    void set_target_distance(double distance);

    // Getters
    double get_target_velocity() const;
    double get_target_distance() const;

    // Control logic
    double compute_control_signal(double current_velocity, double current_position);
};

#endif // CONTROLLER_HPP