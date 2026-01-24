#include "SpringMassControlDemo.hpp"
#include "PlantModel.hpp"
#include "save_to_csv.hpp"
#include <iostream>

using MotionState = SpringMassControlDemo::MotionState;

// Derived class for simulation that overrides virtual methods
// Owns the PlantModel internally with default parameters
class MILController : public SpringMassControlDemo {
public:
    // Default constructor creates plant with preset parameters
    // (mass=0.1kg, spring_constant=100 N/mm, damping=1.0 N*s/mm)
    MILController() : plant_(0.1, 100.0, 1.0) {
        last_position_ = plant_.get_position();
    }

    // Constructor with custom plant parameters
    MILController(double mass, double spring_constant, double damping)
        : plant_(mass, spring_constant, damping) {
        last_position_ = plant_.get_position();
    }
    
    double read_mass_position() override {
        double current_position = plant_.get_position();
        
        // Calculate velocity from position difference (numerical differentiation)
        double dt = get_sampling_time();
        if (dt > 0) {
            calculated_velocity_ = (current_position - last_position_) / dt;
        }
        last_position_ = current_position;
        
        return current_position;
    }

    // Override to provide mass velocity to state machine
    double read_mass_velocity() override {
        return plant_.get_velocity();  // Use actual velocity for accurate simulation
    }

    // Override to set motor velocity - called by state handlers
    void set_motor_velocity(double drive_velocity, int8_t direction) override {
        // In this simulation we update the plant model directly
        // Note: drive_velocity from controller already includes sign for direction
        plant_.update(drive_velocity, get_sampling_time());
    }

    // Reset the derived class state (call when starting new cycle)
    void reset_sensor_state() {
        last_position_ = plant_.get_position();
        calculated_velocity_ = 0.0;
        plant_.reset();
    }

    // Expose mass velocity for logging
    double get_current_mass_velocity() const {
        return plant_.get_velocity();
    }
    
private:
    PlantModel plant_;  // Owned by this class
    double last_position_ = 0.0;
    double calculated_velocity_ = 0.0;
};

// Log data structure using vectors for CSV export
struct LogData {
    std::vector<double> time;
    std::vector<double> drive_position;
    std::vector<double> drive_velocity;
    std::vector<double> desired_velocity;
    std::vector<double> mass_position;
    std::vector<double> mass_velocity;
    std::vector<int> motion_state;
    
    void clear() {
        time.clear();
        drive_position.clear();
        drive_velocity.clear();
        desired_velocity.clear();
        mass_position.clear();
        mass_velocity.clear();
        motion_state.clear();
    }

    void add(double t, double dp, double dv, double des_v, double mp, double mv, int state) {
        time.push_back(t);
        drive_position.push_back(dp);
        drive_velocity.push_back(dv);
        desired_velocity.push_back(des_v);
        mass_position.push_back(mp);
        mass_velocity.push_back(mv);
        motion_state.push_back(state);
    }
    
    bool save(const std::string& filename) const {
        // Convert motion_state to double for CSV export
        std::vector<double> state_double(motion_state.begin(), motion_state.end());
        return save_columns_to_csv(
            {time, drive_position, drive_velocity, desired_velocity, mass_position, mass_velocity, state_double},
            filename,
            "time,drive_position,drive_velocity,desired_velocity,mass_position,mass_velocity,motion_state"
        );
    }
};

// Helper function to convert MotionState to string for logging
const char* motion_state_to_string(MotionState state) {
    switch (state) {
        case MotionState::Home: return "Home";
        case MotionState::Extending: return "Extending";
        case MotionState::Final_Velocity: return "Final_Velocity";
        case MotionState::At_Final_Distance: return "At_Final_Distance";
        case MotionState::Retracting: return "Retracting";
        case MotionState::Manual_Stop: return "Manual_Stop";
        case MotionState::Error: return "Error";
        default: return "Unknown";
    }
}

int main() {

    // Print the program as started
    std::cout << "Starting SpringMassControlDemo...\n";

    double final_velocity = 15.0; // mm/s
    double approach_distance = 2.0; // mm
    double final_distance = 20.0; // mm
    double approach_offset = 5.0; // mm
    double travel_velocity = 100.0; // mm/s
    double acceleration = 250.0; // mm/s^2

    // Create MILController (plant model created internally with default params)
    MILController controller;
    controller.set_final_velocity(final_velocity);
    controller.set_approach_distance(approach_distance);
    controller.set_final_distance(final_distance);
    controller.set_approach_offset(approach_offset);
    controller.set_travel_velocity(travel_velocity);
    controller.set_acceleration(acceleration);

    // Set P only control for closed loop simulation
    controller.set_pid_gains(4.0, 0.0, 0.0);

    // Create log data structure
    LogData log;

    // Sampling time in seconds
    const double dt = controller.get_sampling_time(); // 0.001 seconds = 1 ms

    // Track total time across all cycles
    double total_time_s = 0.0;
    int step = 0;

    // Track state transitions
    MotionState last_state = controller.get_motion_state();
    std::cout << "Initial state: " << motion_state_to_string(last_state) << "\n";

    // main loop
    while (1) 
        controller.update();

    return 0;
}
