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
    // Initialize Ruckig trajectory generator with sampling time
    ruckig_ = std::make_unique<ruckig::Ruckig<1>>(SAMPLING_TIME);
    
    // Initialize Ruckig input parameters for the trajectory
    ruckig_input_.max_velocity[0] = travel_velocity_;
    ruckig_input_.max_acceleration[0] = acceleration_;
    ruckig_input_.max_jerk[0] = JERK;
    
    // Set initial state (position, velocity, acceleration)
    ruckig_input_.current_position[0] = 0.0;
    ruckig_input_.current_velocity[0] = control_velocity_;
    ruckig_input_.current_acceleration[0] = 0.0;
    
    // Set target state: reach approach_distance with final_velocity
    double target_distance = approach_distance_ - approach_offset_;
    ruckig_input_.target_position[0] = target_distance;
    ruckig_input_.target_velocity[0] = final_velocity_;
    ruckig_input_.target_acceleration[0] = 0.0;

    // Create initial velocity profile with default parameters (for backwards compatibility)
    // create_velocity_profile();
}

// Main update function for the state machine
void SpringMassControlDemo::update() {
    switch (motion_state_) {
        case MotionState::Home:
            handle_home();
            break;

        case MotionState::Extending:
            handle_extending();
            break;

        case MotionState::Final_Velocity:
            handle_final_velocity();
            break;

        case MotionState::At_Final_Distance:
            handle_at_final_distance();
            break;

        case MotionState::Retracting:
            handle_retracting();
            break;

        case MotionState::Manual_Stop:
            handle_manual_stop();
            break;

        case MotionState::Error:
            handle_error();
            break;
    }
}

// Handlers for state machine
void SpringMassControlDemo::handle_home() {
    // In Home state, wait for command to start extending
    // No action needed here for now
}

void SpringMassControlDemo::handle_extending() {
    mass_position_ = read_mass_position();
    mass_velocity_ = read_mass_velocity();
    velocity_control(mass_position_, mass_velocity_);
    set_motor_velocity(control_velocity_, direction_);
    // State transition handled in velocity_control() when Ruckig finishes
}

void SpringMassControlDemo::handle_final_velocity() {
    mass_position_ = read_mass_position();
    mass_velocity_ = read_mass_velocity();
    velocity_control(mass_position_, mass_velocity_);
    set_motor_velocity(control_velocity_, direction_);
    // State transition to At_Final_Distance handled in velocity_control()
}

void SpringMassControlDemo::handle_at_final_distance() {
    // Hold position, wait for retract command
    control_velocity_ = 0.0;
    set_motor_velocity(0.0, direction_);
    // User calls start_retract() to transition to Retracting state
}

void SpringMassControlDemo::handle_retracting() {
    mass_position_ = read_mass_position();
    mass_velocity_ = read_mass_velocity();
    velocity_control(mass_position_, mass_velocity_);
    set_motor_velocity(control_velocity_, direction_);
    // State transition to Home handled in velocity_control() when Ruckig finishes
}

void SpringMassControlDemo::handle_manual_stop() {
    // Hold at zero velocity, wait for reset command
    control_velocity_ = 0.0;
    set_motor_velocity(0.0, direction_);
    // User calls reset_trajectory() to return to Home state
}

void SpringMassControlDemo::handle_error() {
    // Safe state: stop all motion
    control_velocity_ = 0.0;
    set_motor_velocity(0.0, direction_);
    // User must call reset_trajectory() to recover
}

// Read mass position sensor function
// This function is virtual so that it can be overridden for either the plant model or the HAL
double SpringMassControlDemo::read_mass_position() {
    return mass_position_;
}

// Read mass velocity sensor function
// Virtual so derived classes can override for plant model or HAL
double SpringMassControlDemo::read_mass_velocity() {
    return mass_velocity_;
}

void SpringMassControlDemo::set_motor_velocity(double drive_velocity, int8_t direction) {
    // Base implementation does nothing
    // Derived classes should override to set actual motor velocity from plant or HAL
}

void SpringMassControlDemo::read_manual_stop_input() {
    // Base implementation: no hardware, assume stop not pressed
    // Derived classes override to read actual hardware input
}

// Manual stop function to halt motion immediately
void SpringMassControlDemo::manual_stop() {
    // Set control velocity to zero to stop motion
    control_velocity_ = 0.0;
    // Update motion state to Manual_Stop
    motion_state_ = MotionState::Manual_Stop;

    // Update Ruckig input to reflect immediate stop
    ruckig_input_.current_position[0] = drive_position_;
    ruckig_input_.current_velocity[0] = 0.0;
    ruckig_input_.current_acceleration[0] = 0.0;

    // Reset PID
    reset_pid();
}

// Closed Loop Control based on current mass position and velocity
// Uses Ruckig for real-time S-curve trajectory generation with PID control
void SpringMassControlDemo::velocity_control(double mass_position, double mass_velocity) {
    // Update internal state variables
    mass_position_ = mass_position;
    mass_velocity_ = mass_velocity;

    // Use Ruckig to compute the next trajectory state
    ruckig::Result result = ruckig_->update(ruckig_input_, ruckig_output_);

    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
        // Get desired velocity from Ruckig trajectory
        desired_velocity_ = ruckig_output_.new_velocity[0];
        
        // Pass the output state as input for the next iteration
        ruckig_output_.pass_to_input(ruckig_input_);
    } else {
        // Trajectory calculation failed, use appropriate velocity based on direction
        if (direction_ > 0) {
            desired_velocity_ = final_velocity_;
        } else {
            desired_velocity_ = 0.0;  // Stop when retracting
        }
    }

    // If trajectory is finished, handle based on motion state
    if (result == ruckig::Result::Finished) {
        if (motion_state_ == MotionState::Extending) {
            desired_velocity_ = final_velocity_;  // Maintain final velocity after extend
            motion_state_ = MotionState::Final_Velocity;
        } else if (motion_state_ == MotionState::Retracting) {
            desired_velocity_ = 0.0;  // Stop after retract
            motion_state_ = MotionState::Home;
        }
    }

    // If mass position has reached or exceeded final distance, stop motion and set to At_Final_Distance state
    if (mass_position_ >= final_distance_ && motion_state_ == MotionState::Final_Velocity) {
        desired_velocity_ = 0.0;
        motion_state_ = MotionState::At_Final_Distance;
        // Exit early to prevent overshoot
        control_velocity_ = 0.0;
        return;
    }

    // Calculate velocity error (desired - actual)
    double error = desired_velocity_ - mass_velocity_;

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
    control_velocity_ = desired_velocity_ + pid_output;

    // Track drive position by integrating control velocity
    drive_position_ += control_velocity_ * SAMPLING_TIME;
    time_counter_++;
}



void SpringMassControlDemo::reset_trajectory() {
    // Reset internal state variables
    drive_position_ = 0.0;
    time_counter_ = 0;
    control_velocity_ = 1.0;
    
    // Reset PID state
    reset_pid();
    
    // Reinitialize Ruckig input parameters
    ruckig_input_.max_velocity[0] = travel_velocity_;
    ruckig_input_.max_acceleration[0] = acceleration_;
    ruckig_input_.max_jerk[0] = JERK;
    
    // Set initial state (position, velocity, acceleration)
    ruckig_input_.current_position[0] = 0.0;
    ruckig_input_.current_velocity[0] = control_velocity_;
    ruckig_input_.current_acceleration[0] = 0.0;
    
    // Set target state: reach approach_distance with final_velocity
    double target_distance = approach_distance_ - approach_offset_;
    ruckig_input_.target_position[0] = target_distance;
    ruckig_input_.target_velocity[0] = final_velocity_;
    ruckig_input_.target_acceleration[0] = 0.0;
    
    // Reset motion state
    motion_state_ = MotionState::Home;
}

void SpringMassControlDemo::start_extend() {
    // Set direction to extend
    direction_ = 1;
    motion_state_ = MotionState::Extending;
    
    // Reset PID state
    reset_pid();
    
    // Configure Ruckig with user-defined parameters for extend
    ruckig_input_.max_velocity[0] = travel_velocity_;
    ruckig_input_.max_acceleration[0] = acceleration_;
    ruckig_input_.max_jerk[0] = JERK;
    
    // Set current state from actual position
    ruckig_input_.current_position[0] = drive_position_;
    ruckig_input_.current_velocity[0] = control_velocity_;
    ruckig_input_.current_acceleration[0] = 0.0;
    
    // Set target state: reach approach_distance with final_velocity
    double target_distance = approach_distance_ - approach_offset_;
    ruckig_input_.target_position[0] = target_distance;
    ruckig_input_.target_velocity[0] = final_velocity_;
    ruckig_input_.target_acceleration[0] = 0.0;
}

void SpringMassControlDemo::start_retract() {
    // Set direction to retract
    direction_ = -1;
    motion_state_ = MotionState::Retracting;
    
    // Reset PID state
    reset_pid();
    
    // Configure Ruckig with MAX parameters for retract (fastest return)
    ruckig_input_.max_velocity[0] = MAX_VELOCITY;
    ruckig_input_.max_acceleration[0] = MAX_ACCELERATION;
    ruckig_input_.max_jerk[0] = JERK;
    
    // Set current state from actual position
    ruckig_input_.current_position[0] = drive_position_;
    ruckig_input_.current_velocity[0] = control_velocity_;
    ruckig_input_.current_acceleration[0] = 0.0;
    
    // Set target state: return to home position (0.0) and stop
    ruckig_input_.target_position[0] = 0.0;
    ruckig_input_.target_velocity[0] = 0.0;
    ruckig_input_.target_acceleration[0] = 0.0;
}

void SpringMassControlDemo::set_direction(int8_t direction) {
    if (direction >= 0) {
        direction_ = 1;
    } else {
        direction_ = -1;
    }
}

int8_t SpringMassControlDemo::get_direction() const {
    return direction_;
}

SpringMassControlDemo::MotionState SpringMassControlDemo::get_motion_state() const {
    return motion_state_;
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
double SpringMassControlDemo::get_desired_velocity() const {
    return desired_velocity_;
}
double SpringMassControlDemo::get_mass_position() const {
    return mass_position_;
}
double SpringMassControlDemo::get_mass_velocity() const {
    return mass_velocity_;
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

// ====== Legacy Velocity Profile Generation (S-Curve) ======
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
