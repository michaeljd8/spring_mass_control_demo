#include "SpringMassControlDemo.hpp"

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
    // Implementation of velocity profile creation
}

// Closed Loop Control based on current mass position and velocity
void SpringMassControlDemo::velocity_control(double drive_velocity, double mass_position, double mass_velocity) {
    // Implementation of closed loop velocity control
}

// Setters for User Defined Parameters with validation for limits
void SpringMassControlDemo::set_final_velocity(double final_velocity) {
    if (final_velocity < MIN_VELOCITY) {
        final_velocity_ = MIN_VELOCITY;
    } else if (final_velocity > MAX_VELOCITY) {
        final_velocity_ = MAX_VELOCITY;
    } else {
        final_velocity_ = final_velocity;
    }
}
