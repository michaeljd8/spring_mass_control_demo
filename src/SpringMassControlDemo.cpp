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
      mass_velocity_(0.0) {

    // Create initial velocity profile with default parameters
    create_velocity_profile();
}

// Helper function to calculate the distance of an S-curve profile
double SpringMassControlDemo::s_curve_distance(double initial_velocity, double final_velocity, double acceleration, double jerk) {
    // Calculate distance to go from initial to final velocity including jerk
    double jerk_time = acceleration / jerk; // Time to reach max acceleration
    double jerk_distance = (1.0 / 6.0) * jerk * jerk_time * jerk_time * jerk_time; // Distance covered during jerk phase

    double constant_accel_distance = (travel_velocity_ * travel_velocity_) / (2.0 * acceleration_); // Distance during constant acceleration

    double s_curve_distance = 2.0 * jerk_distance + constant_accel_distance; // Total distance including jerk
    return s_curve_distance;

}

// Create Velocity Profile based on user defined parameters
void SpringMassControlDemo::create_velocity_profile() {

    // Clear the velocity profile
    velocity_profile_.clear();

    // Calculate the total time for the velocity profile
    double total_distance = target_distance_ - approach_offset_;
    double accel_to_max_dist = s_curve_distance(0.0, travel_velocity_, acceleration_, JERK);
    double decel_to_final_dist = s_curve_distance(travel_velocity_, final_velocity_, deceleration_, JERK);

    // Check for triangular profile condition
    if (total_distance < (accel_to_max_dist + decel_to_final_dist)) {
        std::cerr << "Warning: Triangular profile detected. Adjusting travel velocity." << std::endl;
        travel_velocity_ = total_distance / (accel_to_max_dist + decel_to_final_dist) * travel_velocity_;
        accel_to_max_dist = s_curve_distance(0.0, travel_velocity_, acceleration_, JERK);
        decel_to_final_dist = s_curve_distance(travel_velocity_, final_velocity_, deceleration_, JERK);
    }

    double accel_time = travel_velocity_ / acceleration_;
    double decel_time = travel_velocity_ / deceleration_;
    double constant_velocity_time = (total_distance - accel_to_max_dist - decel_to_final_dist) / travel_velocity_;

    // Generate the velocity profile
    double current_time = 0.0;
    double current_velocity = 0.0;

    // Acceleration phase
    while (current_time < accel_time) {
        current_velocity = current_time * acceleration_;
        velocity_profile_.push_back(current_velocity);
        current_time += SAMPLING_TIME;
    }

    // Constant velocity phase
    while (current_time < (accel_time + constant_velocity_time)) {
        velocity_profile_.push_back(travel_velocity_);
        current_time += SAMPLING_TIME;
    }

    // Deceleration phase
    while (current_time < (accel_time + constant_velocity_time + decel_time)) {
        current_velocity = travel_velocity_ - ((current_time - accel_time - constant_velocity_time) * deceleration_);
        velocity_profile_.push_back(current_velocity);
        current_time += SAMPLING_TIME;
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

