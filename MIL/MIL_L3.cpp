// Model in Loop Level 3 for SpringMassControlDemo.hpp
// Tests the update() state machine loop

#include "SpringMassControlDemo.hpp"
#include "PlantModel.hpp"
#include "save_to_csv.hpp"
#include <iostream>

using MotionState = SpringMassControlDemo::MotionState;

// Derived class for simulation that overrides virtual methods
class MILController : public SpringMassControlDemo {
public:
    MILController(PlantModel& plant) : plant_(plant) {
        // Initialize last_position_ to current plant position to avoid velocity spike on first call
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
    }

    // Expose mass velocity for logging
    double get_current_mass_velocity() const {
        return plant_.get_velocity();
    }
    
private:
    PlantModel& plant_;
    double last_position_ = 0.0;
    double calculated_velocity_ = 0.0;
};

// Log data structure using vectors for CSV export
struct LogData {
    std::vector<double> time;
    std::vector<double> drive_position;
    std::vector<double> drive_velocity;
    std::vector<double> mass_position;
    std::vector<double> mass_velocity;
    std::vector<int> motion_state;
    
    void clear() {
        time.clear();
        drive_position.clear();
        drive_velocity.clear();
        mass_position.clear();
        mass_velocity.clear();
        motion_state.clear();
    }
    
    void add(double t, double dp, double dv, double mp, double mv, int state) {
        time.push_back(t);
        drive_position.push_back(dp);
        drive_velocity.push_back(dv);
        mass_position.push_back(mp);
        mass_velocity.push_back(mv);
        motion_state.push_back(state);
    }
    
    bool save(const std::string& filename) const {
        // Convert motion_state to double for CSV export
        std::vector<double> state_double(motion_state.begin(), motion_state.end());
        return save_columns_to_csv(
            {time, drive_position, drive_velocity, mass_position, mass_velocity, state_double},
            filename,
            "time,drive_position,drive_velocity,mass_position,mass_velocity,motion_state"
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
    std::cout << "=== MIL Level 3: State Machine Update Loop Test ===\n\n";

    // Create PlantModel with light mass and stiff spring for responsive dynamics
    PlantModel plant(0.1, 100.0, 1.0);

    double final_velocity = 15.0; // mm/s
    double approach_distance = 78.0; // mm
    double final_distance = 80.0; // mm
    double approach_offset = 5.0; // mm
    double travel_velocity = 100.0; // mm/s
    double acceleration = 250.0; // mm/s^2

    // Create MILController with custom parameters
    MILController controller(plant);
    controller.set_final_velocity(final_velocity);
    controller.set_approach_distance(approach_distance);
    controller.set_final_distance(final_distance);
    controller.set_approach_offset(approach_offset);
    controller.set_travel_velocity(travel_velocity);
    controller.set_acceleration(acceleration);

    // Set P only control for closed loop simulation
    controller.set_pid_gains(4.0, 0.0, 0.0);

    // Set max time steps to avoid infinite loops
    const int max_steps = 20000; // max iterations per phase

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

    // Loop through 2 cycles: extend + retract
    for (int cycle = 0; cycle < 2; ++cycle) {
        std::cout << "\n--- Cycle " << cycle + 1 << " ---\n";

        // Reset plant and controller for new cycle (except first)
        if (cycle > 0) {
            plant.reset();
            controller.reset_trajectory();
            controller.reset_sensor_state();
            last_state = controller.get_motion_state();
        }

        // Start extend phase - triggers state transition Home -> Extending
        controller.start_extend();
        std::cout << "Command: start_extend()\n";

        // Main loop using update() - runs until At_Final_Distance
        while (controller.get_motion_state() != MotionState::At_Final_Distance && step < max_steps) {
            // Call the state machine update - this handles everything internally
            controller.update();

            // Check for state transition and log it
            MotionState current_state = controller.get_motion_state();
            if (current_state != last_state) {
                std::cout << "State transition: " << motion_state_to_string(last_state) 
                          << " -> " << motion_state_to_string(current_state)
                          << " at t=" << total_time_s << "s, pos=" << controller.get_mass_position() << "mm\n";
                last_state = current_state;
            }

            // Log data
            log.add(total_time_s, 
                    controller.get_drive_position(), 
                    controller.get_control_velocity(), 
                    controller.get_mass_position(), 
                    controller.get_current_mass_velocity(),
                    static_cast<int>(current_state));

            ++step;
            total_time_s += dt;
        }

        std::cout << "Extend complete at position " << controller.get_mass_position() 
                  << " mm (step " << step << ")\n";

        // Dwell to let spring settle
        const int dwell_steps = 500; // 500 ms dwell
        std::cout << "Dwelling for " << dwell_steps << " ms...\n";
        
        for (int dwell = 0; dwell < dwell_steps; ++dwell) {
            // Call update() even during dwell - At_Final_Distance handler manages zero velocity
            controller.update();

            // Log data during dwell
            log.add(total_time_s, 
                    controller.get_drive_position(), 
                    controller.get_control_velocity(), 
                    controller.get_mass_position(), 
                    controller.get_current_mass_velocity(),
                    static_cast<int>(controller.get_motion_state()));
            
            total_time_s += dt;
        }

        // Start retract phase - triggers state transition At_Final_Distance -> Retracting
        controller.start_retract();
        std::cout << "Command: start_retract()\n";

        // Check for state transition
        MotionState current_state = controller.get_motion_state();
        if (current_state != last_state) {
            std::cout << "State transition: " << motion_state_to_string(last_state) 
                      << " -> " << motion_state_to_string(current_state) << "\n";
            last_state = current_state;
        }

        // Main loop using update() - runs until Home
        while (controller.get_motion_state() != MotionState::Home && step < max_steps * 2) {
            // Call the state machine update
            controller.update();

            // Check for state transition
            current_state = controller.get_motion_state();
            if (current_state != last_state) {
                std::cout << "State transition: " << motion_state_to_string(last_state) 
                          << " -> " << motion_state_to_string(current_state)
                          << " at t=" << total_time_s << "s, pos=" << controller.get_mass_position() << "mm\n";
                last_state = current_state;
            }

            // Log data
            log.add(total_time_s, 
                    controller.get_drive_position(), 
                    controller.get_control_velocity(), 
                    controller.get_mass_position(), 
                    controller.get_current_mass_velocity(),
                    static_cast<int>(current_state));

            ++step;
            total_time_s += dt;
        }

        std::cout << "Retract complete at position " << controller.get_mass_position() 
                  << " mm (step " << step << ")\n";
    }

    // Print summary
    std::cout << "\n=== MIL Level 3 Simulation Summary ===\n";
    std::cout << "Total simulation time: " << total_time_s << " s\n";
    std::cout << "Total steps: " << step << "\n";

    // Save log to CSV
    if (log.save("MIL3_simulation.csv")) {
        std::cout << "Data saved to MIL3_simulation.csv\n";
    } else {
        std::cerr << "Error: Failed to save data to CSV\n";
    }

    return 0;
}
