#include "SpringMassControlDemo.hpp"
#include <iostream>

SpringMassControlDemo::SpringMassControlDemo(double final_velocity,
                                             double target_distance,
                                             double final_distance,
                                             double approach_offset,
                                             double travel_velocity,
                                             double acceleration,
                                             double deceleration)
    : final_velocity_(final_velocity),
      target_distance_(target_distance),
      final_distance_(final_distance),
      approach_offset_(approach_offset),
      travel_velocity_(travel_velocity),
      acceleration_(acceleration),
      deceleration_(deceleration),
      control_velocity_(0.0),
      drive_position_(0.0),
      drive_velocity_(0.0),
      mass_position_(0.0),
      mass_velocity_(0.0) {}

// Create Velocity Profile based on user defined parameters
void SpringMassControlDemo::create_velocity_profile() {
    // Calculate distance to jerk to set acceleration
    double jerk_duration_ = acceleration_ / JERK;
    double distance_to_jerk = 0.5 * acceleration_ * (jerk_duration_ * jerk_duration_);
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

void SpringMassControlDemo::set_target_distance(double target_distance) {
    if (target_distance < MIN_DISTANCE) {
        target_distance_ = MIN_DISTANCE;
        std::cerr << "Out of Bounds: Target distance set to minimum limit." << std::endl;
    } else if (target_distance > MAX_DISTANCE) {
        target_distance_ = MAX_DISTANCE;
        std::cerr << "Out of Bounds: Target distance set to maximum limit." << std::endl;
    } else {
        target_distance_ = target_distance;
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
    } else if (approach_offset > target_distance_) {
        approach_offset_ = target_distance_;
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

void SpringMassControlDemo::set_deceleration(double deceleration) {
    if (deceleration < MIN_DECELERATION) {
        deceleration_ = MIN_DECELERATION;
        std::cerr << "Out of Bounds: Deceleration set to minimum limit." << std::endl;
    } else if (deceleration > MAX_DECELERATION) {
        deceleration_ = MAX_DECELERATION;
        std::cerr << "Out of Bounds: Deceleration set to maximum limit." << std::endl;
    } else {
        deceleration_ = deceleration;
    }
}

// Getters for User Defined Parameters
double SpringMassControlDemo::get_final_velocity() const {
    return final_velocity_;
}
double SpringMassControlDemo::get_target_distance() const {
    return target_distance_;
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
double SpringMassControlDemo::get_deceleration() const {
    return deceleration_;
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

