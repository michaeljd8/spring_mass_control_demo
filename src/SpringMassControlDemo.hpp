/*
This ControllerDemo Class will manage the control logic for the spring-mass system.
It will create a desired velocity profile based on user inputs and apply a closed-loop control.
*/

#ifndef SPRINGMASSCONTROLDEMO_HPP
#define SPRINGMASSCONTROLDEMO_HPP

#include <vector>
#include <utility> 
#include <memory>
#include <ruckig/ruckig.hpp>

class SpringMassControlDemo {
public:
    // Motion state enum for tracking extend/retract state
    enum class MotionState : uint8_t {
        Idle,
        Extending,
        Retracting
    };

    using MotionState = SpringMassControlDemo::MotionState;

    // Constructor with default parameters
    SpringMassControlDemo(double final_velocity = 10.0, // mm/s
                        double approach_distance = 55.0, // mm
                        double final_distance = 60.0, // mm
                        double approach_offset = 2.0, // mm
                        double travel_velocity = 80.0, // mm/s
                        double acceleration = 200.0); // mm/s^2

    // Closed Loop Control based on current mass position and velocity
    void velocity_control(double mass_position, double mass_velocity);

    // Setters for User Defined Parameters
    void set_final_velocity(double final_velocity);
    void set_target_distance(double target_distance);
    void set_final_distance(double final_distance);
    void set_approach_offset(double approach_offset);
    void set_travel_velocity(double travel_velocity);
    void set_acceleration(double acceleration);
    void set_approach_distance(double approach_distance); // Setter for approach distance

    // Setters for PID Controller Gains
    void set_pid_gains(double kp, double ki, double kd);
    void set_kp(double kp);
    void set_ki(double ki);
    void set_kd(double kd);
    void reset_pid(); // Reset integral and derivative terms
    void reset_trajectory(); // Reset Ruckig trajectory to initial state

    // Direction and motion control
    void start_extend();    // Configure and start extend motion (user parameters)
    void start_retract();   // Configure and start retract motion (max parameters)
    void set_direction(int8_t direction);  // +1 = extend, -1 = retract
    int8_t get_direction() const;
    MotionState get_motion_state() const;

    // Getters for User Defined Parameters
    double get_final_velocity() const;
    double get_target_distance() const;
    double get_final_distance() const;
    double get_approach_offset() const;
    double get_travel_velocity() const;
    double get_acceleration() const;
    double get_approach_distance() const; // Getter for approach distance

    // Getters for PID Controller Gains
    double get_kp() const;
    double get_ki() const;
    double get_kd() const;

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

    virtual ~SpringMassControlDemo() = default;

    // ==== Legacy Precomputed Velocity Profile Methods ====
    // Create Velocity Profile based on user defined parameters
    void create_velocity_profile();

    // Getter for Velocity Profile
    const std::vector<std::pair<double, double>>& get_velocity_profile() const {
        return velocity_profile_;
    }
    

private:

    // User Defined Parameters
    double final_velocity_; // Velocity the mass must be traveling after the approach distance till the final distance (mm/s)
    double approach_distance_; // Desired position achieve stable final velocity (mm)
    double final_distance_; // Final distance to stop at (mm)
    double approach_offset_; // Distance before the approach distance to target reaching approach velocity (mm)
    double travel_velocity_; // Maximum velocity during travel to approach distance (mm/s)
    double acceleration_; // Acceleration (mm/s^2)

    // Internal State Variables
    int8_t direction_ = 1;  // +1 = extend, -1 = retract
    MotionState motion_state_ = MotionState::Idle;  // Current motion state
    double control_velocity_; // Current set drive velocity (mm/s)
    double drive_position_; // Measured drive position after control (mm)
    double mass_position_; // Measured mass position (mm)
    double mass_velocity_; // Calculated from mass_position_ (mm/s)
    double time_counter_; // Time counter for tracking simulation time

    // PID Controller Variables
    double kp_;             // Proportional gain
    double ki_;             // Integral gain
    double kd_;             // Derivative gain
    double integral_error_; // Accumulated integral error
    double previous_error_; // Previous error for derivative calculation

    // Ruckig trajectory generator for real-time S-curve motion
    std::unique_ptr<ruckig::Ruckig<1>> ruckig_;
    ruckig::InputParameter<1> ruckig_input_;
    ruckig::OutputParameter<1> ruckig_output_;

    // System Parameters
    static constexpr double MAX_VELOCITY = 100.0; // Maximum allowable velocity (mm/s)
    static constexpr double MIN_VELOCITY = 10.0;  // Minimum allowable velocity (mm/s)
    static constexpr double MAX_ACCELERATION = 500.0; // Maximum allowable acceleration (mm/s^2)
    static constexpr double MIN_ACCELERATION = 50.0;  // Minimum allowable acceleration (mm/s^2)
    static constexpr double MAX_DISTANCE = 100.0;     // Maximum travel distance (mm)
    static constexpr double MIN_DISTANCE = 5.0;       // Minimum travel distance the user can set (mm)
    static constexpr double JERK = 5000.0;            // Jerk is fixed at 5000 mm/s^3 for S-curve profiles
    static constexpr double SAMPLING_TIME = 0.001;    // Sampling time for control loop (s)

    // ==== Legacy Precomputed Velocity Profile Members ====
    // Velocity Profile Pair for Distance and Velocity
    std::vector<std::pair<double, double>> velocity_profile_; // Velocity profile over time (mm/s)


};

#endif // SPRINGMASSCONTROLDEMO_HPP