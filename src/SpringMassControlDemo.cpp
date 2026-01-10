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
      control_velocity_(1.0), // Initial control velocity to ensure movement
      drive_position_(0.0),
      mass_position_(0.0),
      mass_velocity_(0.0),
      kp_(1.0),           // Default proportional gain
      ki_(0.1),           // Default integral gain
      kd_(0.05),          // Default derivative gain
      integral_error_(0.0),
      previous_error_(0.0),
      time_counter_(0) // Initialize time counter
{

    // Create initial velocity profile with default parameters
    create_velocity_profile();
}


// Create S-Curve Velocity Profile based on user defined parameters
// S-curve uses jerk-limited motion for smoother acceleration/deceleration transitions
void SpringMassControlDemo::create_velocity_profile() {
    velocity_profile_.clear(); // Clear the vector before populating it

    double distance_to_travel = approach_distance_ - approach_offset_;

    // Time to ramp acceleration from 0 to max (and back to 0)
    double t_jerk = acceleration_ / JERK;
    
    // Velocity gained during jerk phase (acceleration ramping up or down)
    double v_jerk = 0.5 * acceleration_ * t_jerk;
    
    // Calculate if we can reach travel velocity with S-curve
    // Full S-curve acceleration has: jerk-up, constant-accel, jerk-down phases
    double v_accel_phase = travel_velocity_; // Target velocity after acceleration
    
    // Distance covered during acceleration phase (with S-curve)
    // Simplified: assume we can reach full travel velocity
    double t_accel_constant = (travel_velocity_ - 2.0 * v_jerk) / acceleration_;
    if (t_accel_constant < 0) {
        // Not enough time for constant acceleration phase, recalculate
        t_accel_constant = 0;
        t_jerk = std::sqrt(travel_velocity_ / JERK);
    }
    
    // Create the velocity profile
    double current_distance = 0.0;
    double current_velocity = control_velocity_; // Start with initial control velocity to avoid zero
    double current_acceleration = 0.0;
    
    // Phase 1: Jerk-up (acceleration increases from 0 to max)
    double t_phase1_end = t_jerk;
    double t = 0.0;
    while (t < t_phase1_end && current_velocity < travel_velocity_) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        
        // Update acceleration (increases linearly with jerk)
        double new_acceleration = current_acceleration + JERK * SAMPLING_TIME;
        if (new_acceleration > acceleration_) new_acceleration = acceleration_;
        
        // Update velocity using average acceleration
        double new_velocity = current_velocity + (current_acceleration + new_acceleration) / 2.0 * SAMPLING_TIME;
        
        // Update distance using average velocity
        current_distance += (current_velocity + new_velocity) / 2.0 * SAMPLING_TIME;
        current_velocity = new_velocity;
        current_acceleration = new_acceleration;
        t += SAMPLING_TIME;
    }
    
    // Phase 2: Constant acceleration
    double t_phase2_end = t_phase1_end + t_accel_constant;
    while (t < t_phase2_end && current_velocity < travel_velocity_) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        
        double new_velocity = current_velocity + acceleration_ * SAMPLING_TIME;
        current_distance += (current_velocity + new_velocity) / 2.0 * SAMPLING_TIME;
        current_velocity = new_velocity;
        t += SAMPLING_TIME;
    }
    
    // Phase 3: Jerk-down (acceleration decreases from max to 0)
    double t_phase3_end = t_phase2_end + t_jerk;
    while (t < t_phase3_end && current_velocity < travel_velocity_) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        
        double new_acceleration = current_acceleration - JERK * SAMPLING_TIME;
        if (new_acceleration < 0) new_acceleration = 0;
        
        double new_velocity = current_velocity + (current_acceleration + new_acceleration) / 2.0 * SAMPLING_TIME;
        if (new_velocity > travel_velocity_) new_velocity = travel_velocity_;
        
        current_distance += (current_velocity + new_velocity) / 2.0 * SAMPLING_TIME;
        current_velocity = new_velocity;
        current_acceleration = new_acceleration;
        t += SAMPLING_TIME;
    }
    
    // Ensure we're at travel velocity
    current_velocity = travel_velocity_;
    current_acceleration = 0.0;
    
    // Calculate deceleration distances for S-curve
    double decel_velocity_change = travel_velocity_ - final_velocity_;
    double t_decel_jerk = acceleration_ / JERK;
    double t_decel_constant = (decel_velocity_change - 2.0 * (0.5 * acceleration_ * t_decel_jerk)) / acceleration_;
    if (t_decel_constant < 0) t_decel_constant = 0;
    
    // Estimate deceleration distance
    double decel_distance = decel_velocity_change * (t_decel_jerk + t_decel_constant + t_decel_jerk) 
                          - 0.5 * acceleration_ * std::pow(t_decel_constant + t_decel_jerk, 2);
    if (decel_distance < 0) {
        decel_distance = (travel_velocity_ * travel_velocity_ - final_velocity_ * final_velocity_) / (2.0 * acceleration_);
    }
    
    // Phase 4: Constant velocity phase
    double decel_start_distance = distance_to_travel - decel_distance;
    while (current_distance < decel_start_distance) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        current_distance += current_velocity * SAMPLING_TIME;
    }
    
    // Phase 5: Jerk-down for deceleration (acceleration goes from 0 to -max)
    double decel_phase_start = t;
    t = 0.0;
    while (t < t_decel_jerk && current_velocity > final_velocity_) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        
        double new_acceleration = current_acceleration - JERK * SAMPLING_TIME;
        if (new_acceleration < -acceleration_) new_acceleration = -acceleration_;
        
        double new_velocity = current_velocity + (current_acceleration + new_acceleration) / 2.0 * SAMPLING_TIME;
        if (new_velocity < final_velocity_) new_velocity = final_velocity_;
        
        current_distance += (current_velocity + new_velocity) / 2.0 * SAMPLING_TIME;
        current_velocity = new_velocity;
        current_acceleration = new_acceleration;
        t += SAMPLING_TIME;
    }
    
    // Phase 6: Constant deceleration
    while (t < t_decel_jerk + t_decel_constant && current_velocity > final_velocity_) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        
        double new_velocity = current_velocity - acceleration_ * SAMPLING_TIME;
        if (new_velocity < final_velocity_) new_velocity = final_velocity_;
        
        current_distance += (current_velocity + new_velocity) / 2.0 * SAMPLING_TIME;
        current_velocity = new_velocity;
        t += SAMPLING_TIME;
    }
    
    // Phase 7: Jerk-up for deceleration (acceleration goes from -max to 0)
    while (t < 2.0 * t_decel_jerk + t_decel_constant && current_velocity > final_velocity_) {
        velocity_profile_.emplace_back(current_distance, current_velocity);
        
        double new_acceleration = current_acceleration + JERK * SAMPLING_TIME;
        if (new_acceleration > 0) new_acceleration = 0;
        
        double new_velocity = current_velocity + (current_acceleration + new_acceleration) / 2.0 * SAMPLING_TIME;
        if (new_velocity < final_velocity_) new_velocity = final_velocity_;
        
        current_distance += (current_velocity + new_velocity) / 2.0 * SAMPLING_TIME;
        current_velocity = new_velocity;
        current_acceleration = new_acceleration;
        t += SAMPLING_TIME;
    }

}


// Closed Loop Control based on current mass position and velocity
// Uses PID control to adjust drive velocity to track desired velocity profile
void SpringMassControlDemo::velocity_control(double control_velocity, double mass_position, double mass_velocity) {
    // Update internal state variables
    control_velocity_ = control_velocity;
    mass_position_ = mass_position;
    mass_velocity_ = mass_velocity;

    // Get Velocity Profile
    const auto& velocity_profile = get_velocity_profile();

    // Look up desired velocity from velocity profile based on mass position
    double desired_velocity = control_velocity_; 
    
    // Find the desired velocity from the profile based on current mass position
    desired_velocity = velocity_profile[time_counter_].second;

    // If position is beyond the profile, use final velocity
    if (drive_position_ >= velocity_profile_.back().first) {
        desired_velocity = final_velocity_;
    }

    // Calculate velocity error (desired - actual)
    double error = desired_velocity - mass_velocity_;
    
    // Proportional term
    double p_term = kp_ * error;
    
    // Integral term with anti-windup (limit integral accumulation)
    integral_error_ += error * SAMPLING_TIME;
    
    // Anti-windup: limit integral term to prevent excessive accumulation
    double max_integral = MAX_VELOCITY / (ki_ + 1e-6); // Prevent division by zero
    if (integral_error_ > max_integral) {
        integral_error_ = max_integral;
    } else if (integral_error_ < -max_integral) {
        integral_error_ = -max_integral;
    }
    double i_term = ki_ * integral_error_;
    
    // Derivative term (rate of change of error)
    double derivative = (error - previous_error_) / SAMPLING_TIME;
    double d_term = kd_ * derivative;
    previous_error_ = error;
    
    // Calculate PID output
    double pid_output = p_term + i_term + d_term;
    
    // Control velocity is the desired velocity plus PID correction
    control_velocity_ = desired_velocity + pid_output;

    // Track drive position by integrating control velocity
    drive_position_ += control_velocity_ * SAMPLING_TIME;
    time_counter_++;
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

// Setters for PID Controller Gains
void SpringMassControlDemo::set_pid_gains(double kp, double ki, double kd) {
    kp_ = (kp >= 0.0) ? kp : 0.0;
    ki_ = (ki >= 0.0) ? ki : 0.0;
    kd_ = (kd >= 0.0) ? kd : 0.0;
}

void SpringMassControlDemo::set_kp(double kp) {
    kp_ = (kp >= 0.0) ? kp : 0.0;
}

void SpringMassControlDemo::set_ki(double ki) {
    ki_ = (ki >= 0.0) ? ki : 0.0;
}

void SpringMassControlDemo::set_kd(double kd) {
    kd_ = (kd >= 0.0) ? kd : 0.0;
}

void SpringMassControlDemo::reset_pid() {
    integral_error_ = 0.0;
    previous_error_ = 0.0;
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

// Getters for PID Controller Gains
double SpringMassControlDemo::get_kp() const {
    return kp_;
}
double SpringMassControlDemo::get_ki() const {
    return ki_;
}
double SpringMassControlDemo::get_kd() const {
    return kd_;
}


// Getters for Internal State Variables - Used for monitoring and graphing
double SpringMassControlDemo::get_control_velocity() const {
    return control_velocity_;
}
double SpringMassControlDemo::get_drive_position() const {
    return drive_position_;
}
double SpringMassControlDemo::get_mass_position() const {
    return mass_position_;
}
double SpringMassControlDemo::get_mass_velocity() const {
    return mass_velocity_;
}

