/*
This ControllerDemo Class will manage the control logic for the spring-mass system.
It will create a desired velocity profile based on user inputs and apply a closed-loop control.
*/

#ifndef SPRINGMASSCONTROLDEMO_HPP
#define SPRINGMASSCONTROLDEMO_HPP

#include <vector>

class SpringMassControlDemo {
public:
    // Constructor with default parameters
    SpringMassControlDemo(double final_velocity = 10.0, 
                        double approach_distance = 50.0,
                        double final_distance = 60.0, 
                        double approach_offset = 10.0, // Distance BEFORE the target distance to target reaching final velocity
                        double travel_velocity = 80.0,
                        double acceleration = 200.0);

    // Create Velocity Profile based on user defined parameters
    void create_velocity_profile();

    // Closed Loop Control based on current mass position and velocity
    void velocity_control(double drive_velocity, double mass_position, double mass_velocity);

    // Setters for User Defined Parameters
    void set_final_velocity(double final_velocity);
    void set_target_distance(double target_distance);
    void set_final_distance(double final_distance);
    void set_approach_offset(double approach_offset);
    void set_travel_velocity(double travel_velocity);
    void set_acceleration(double acceleration);
    void set_approach_distance(double approach_distance); // Setter for approach distance

    // Getters for User Defined Parameters
    double get_final_velocity() const;
    double get_target_distance() const;
    double get_final_distance() const;
    double get_approach_offset() const;
    double get_travel_velocity() const;
    double get_acceleration() const;
    double get_approach_distance() const; // Getter for approach distance

    // Getters for System Parameters
    static constexpr double get_max_velocity() { return MAX_VELOCITY; }
    static constexpr double get_min_velocity() { return MIN_VELOCITY; }
    static constexpr double get_max_acceleration() { return MAX_ACCELERATION; }
    static constexpr double get_min_acceleration() { return MIN_ACCELERATION; }
    static constexpr double get_max_distance() { return MAX_DISTANCE; }
    static constexpr double get_min_distance() { return MIN_DISTANCE; }
    static constexpr double get_sampling_time() { return SAMPLING_TIME; }

    // Getters for Internal State Variables - Used for monitoring and graphing
    double get_control_velocity() const;
    double get_drive_position() const;
    double get_drive_velocity() const;
    double get_mass_position() const;
    double get_mass_velocity() const;

    // Getter for Velocity Profile
    const std::vector<double>& get_velocity_profile() const {
        return velocity_profile_;
    }

    virtual ~SpringMassControlDemo() = default;

private:

    // User Defined Parameters
    double final_velocity_; // Velocity the mass must be traveling after the approach distance till the final distance (mm/s)
    double approach_distance_; // Desired position achieve stable final velocity (mm)
    double final_distance_; // Final distance to stop at (mm)
    double approach_offset_; // Distance before the approach distance to target reaching approach velocity (mm)
    double travel_velocity_; // Maximum velocity during travel to approach distance (mm/s)
    double acceleration_; // Acceleration (mm/s^2)

    // Internal State Variables
    double control_velocity_; // Current set drive velocity (mm/s)
    double drive_position_; // Measured drive position after control (mm)
    double drive_velocity_; // Calculated from drive_position_ (mm/s)
    double mass_position_; // Measured mass position (mm)
    double mass_velocity_; // Calculated from mass_position_ (mm/s)

    // Velocity Profile
    std::vector<double> velocity_profile_;

    // System Parameters
    static constexpr double MAX_VELOCITY = 100.0; // Maximum allowable velocity (mm/s)
    static constexpr double MIN_VELOCITY = 10.0;  // Minimum allowable velocity (mm/s)
    static constexpr double MAX_ACCELERATION = 500.0; // Maximum allowable acceleration (mm/s^2)
    static constexpr double MIN_ACCELERATION = 50.0;  // Minimum allowable acceleration (mm/s^2)
    static constexpr double MAX_DISTANCE = 100.0;     // Maximum travel distance (mm)
    static constexpr double MIN_DISTANCE = 5.0;       // Minimum travel distance the user can set (mm)
    static constexpr double JERK = 1000.0;            // Jerk is fixed at 1000 mm/s^3 for S-curve profiles
    static constexpr double SAMPLING_TIME = 0.001;    // Sampling time for control loop (s)

    // Helper function for calculating the distance of an S-curve profile
    double s_curve_distance(double initial_velocity, double final_velocity, double acceleration, double jerk);
};

#endif // SPRINGMASSCONTROLDEMO_HPP