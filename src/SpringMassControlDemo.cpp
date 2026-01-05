#include "SpringMassControlDemo.hpp"
#include <iostream>
#include <cmath>

SpringMassControlDemo::SpringMassControlDemo(double final_velocity,
                                             double approach_distance,
                                             double final_distance,
                                             double approach_offset,
                                             double travel_velocity,
                                             double acceleration)
    : final_velocity_(final_velocity),
      approach_distance_(approach_distance),
      final_distance_(final_distance),
      approach_offset_(approach_offset),
      travel_velocity_(travel_velocity),
      acceleration_(acceleration),
      control_velocity_(0.0),
      drive_position_(0.0),
      drive_velocity_(0.0),
      mass_position_(0.0),
      mass_velocity_(0.0) {

    // Create initial velocity profile with default parameters
    create_velocity_profile();
}


// Create trapezoidal Velocity Profile based on user defined parameters
void SpringMassControlDemo::create_velocity_profile() {
    velocity_profile_.clear();

    // Simple trapezoidal profile generation (placeholder logic)
    double distance_to_accelerate = approach_distance_ - approach_offset_;
    double time_to_accelerate = travel_velocity_ / acceleration_;
    double distance_accelerating = 0.5 * acceleration_ * time_to_accelerate * time_to_accelerate;

    if (distance_accelerating > distance_to_accelerate) {
        time_to_accelerate = std::sqrt(2 * distance_to_accelerate / acceleration_);
        travel_velocity_ = acceleration_ * time_to_accelerate;
    }

    double total_time = (distance_to_accelerate / travel_velocity_) + (final_distance_ - approach_distance_) / final_velocity_;
    int total_steps = static_cast<int>(total_time / SAMPLING_TIME);

    for (int i = 0; i < total_steps; ++i) {
        double current_time = i * SAMPLING_TIME;
        double current_velocity;

        if (current_time < time_to_accelerate) {
            current_velocity = acceleration_ * current_time;
        } else if (current_time < (total_time - (final_distance_ - approach_distance_) / final_velocity_)) {
            current_velocity = travel_velocity_;
        } else {
            double decel_time = current_time - (total_time - (final_distance_ - approach_distance_) / final_velocity_);
            current_velocity = travel_velocity_ - acceleration_ * decel_time;
            if (current_velocity < final_velocity_) {
                current_velocity = final_velocity_;
            }
        }

        velocity_profile_.push_back(current_velocity);
    }
}


// Closed Loop Control based on current mass position and velocity
void SpringMassControlDemo::velocity_control(double drive_velocity, double mass_position, double mass_velocity) {
    // Implementation of closed loop velocity control
}

// Setters for User Defined Parameters with validation for limits
void SpringMassControlDemo::set_final_velocity(double final_velocity) {
    if (final_velocity < MIN_VELOCITY) {
        final_velocity_ = MIN_VELOCITY;
        std::cerr << "Out of Bounds: Final velocity set to minimum limit." << std::endl;
    } else if (final_velocity > MAX_VELOCITY) {
        final_velocity_ = MAX_VELOCITY;
        std::cerr << "Out of Bounds: Final velocity set to maximum limit." << std::endl;
    } else {
        final_velocity_ = final_velocity;
    }
}

void SpringMassControlDemo::set_approach_distance(double approach_distance) {
    if (approach_distance < MIN_DISTANCE) {
        approach_distance_ = MIN_DISTANCE;
        std::cerr << "Out of Bounds: Approach distance set to minimum limit." << std::endl;
    } else if (approach_distance > MAX_DISTANCE) {
        approach_distance_ = MAX_DISTANCE;
        std::cerr << "Out of Bounds: Approach distance set to maximum limit." << std::endl;
    } else {
        approach_distance_ = approach_distance;
    }
}

void SpringMassControlDemo::set_final_distance(double final_distance) {
    if (final_distance < MIN_DISTANCE) {
        final_distance_ = MIN_DISTANCE;
        std::cerr << "Out of Bounds: Final distance set to minimum limit." << std::endl;
    } else if (final_distance > MAX_DISTANCE) {
        final_distance_ = MAX_DISTANCE;
        std::cerr << "Out of Bounds: Final distance set to maximum limit." << std::endl;
    } else {
        final_distance_ = final_distance;
    }
}

void SpringMassControlDemo::set_approach_offset(double approach_offset) {
    if (approach_offset < 0.0) {
        approach_offset_ = 0.0;
        std::cerr << "Out of Bounds: Approach offset set to minimum limit of 0." << std::endl;
    } else if (approach_offset > approach_distance_) {
        approach_offset_ = approach_distance_;
        std::cerr << "Out of Bounds: Approach offset set to target distance limit." << std::endl;
    } else {
        approach_offset_ = approach_offset;
    }
}

void SpringMassControlDemo::set_travel_velocity(double travel_velocity) {
    if (travel_velocity < MIN_VELOCITY) {
        travel_velocity_ = MIN_VELOCITY;
        std::cerr << "Out of Bounds: Travel velocity set to minimum limit." << std::endl;
    } else if (travel_velocity > MAX_VELOCITY) {
        travel_velocity_ = MAX_VELOCITY;
        std::cerr << "Out of Bounds: Travel velocity set to maximum limit." << std::endl;
    } else {
        travel_velocity_ = travel_velocity;
    }
}

void SpringMassControlDemo::set_acceleration(double acceleration) {
    if (acceleration < MIN_ACCELERATION) {
        acceleration_ = MIN_ACCELERATION;
        std::cerr << "Out of Bounds: Acceleration set to minimum limit." << std::endl;
    } else if (acceleration > MAX_ACCELERATION) {
        acceleration_ = MAX_ACCELERATION;
        std::cerr << "Out of Bounds: Acceleration set to maximum limit." << std::endl;
    } else {
        acceleration_ = acceleration;
    }
}

void SpringMassControlDemo::set_target_distance(double target_distance) {
    if (target_distance < MIN_DISTANCE) {
        approach_distance_ = MIN_DISTANCE;
        std::cerr << "Out of Bounds: Target distance set to minimum limit." << std::endl;
    } else if (target_distance > MAX_DISTANCE) {
        approach_distance_ = MAX_DISTANCE;
        std::cerr << "Out of Bounds: Target distance set to maximum limit." << std::endl;
    } else {
        approach_distance_ = target_distance;
    }
}

// Getters for User Defined Parameters
double SpringMassControlDemo::get_approach_distance() const {
    return approach_distance_;
}
double SpringMassControlDemo::get_final_distance() const {
    return final_distance_;
}
double SpringMassControlDemo::get_approach_offset() const {
    return approach_offset_;
}
double SpringMassControlDemo::get_travel_velocity() const {
    return travel_velocity_;
}
double SpringMassControlDemo::get_acceleration() const {
    return acceleration_;
}
double SpringMassControlDemo::get_final_velocity() const {
    return final_velocity_;
}
double SpringMassControlDemo::get_target_distance() const {
    return approach_distance_;
}


// Getters for Internal State Variables - Used for monitoring and graphing
double SpringMassControlDemo::get_control_velocity() const {
    return control_velocity_;
}
double SpringMassControlDemo::get_drive_position() const {
    return drive_position_;
}
double SpringMassControlDemo::get_drive_velocity() const {
    return drive_velocity_;
}
double SpringMassControlDemo::get_mass_position() const {
    return mass_position_;
}
double SpringMassControlDemo::get_mass_velocity() const {
    return mass_velocity_;
}

