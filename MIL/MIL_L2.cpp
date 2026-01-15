// Model in Loop Level 2 for SpringMassControlDemo.hpp

#include "SpringMassControlDemo.hpp"
#include "PlantModel.hpp"
#include "save_to_csv.hpp"
#include <iostream>

using MotionState = SpringMassControlDemo::MotionState;

// Derived class for simulation that overrides read_mass_position
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
            mass_velocity_ = (current_position - last_position_) / dt;
        }
        last_position_ = current_position;
        
        return current_position;
    }

    // Getter for mass velocity - placed in derived class so filtering can be applied in HAL
    double read_mass_velocity() {
        // For simulation, we can cheat and use actual plant velocity for comparison
        // In real HAL, this would return the calculated mass_velocity_
        return plant_.get_velocity();  // Use actual velocity for accurate simulation
    }

    // Override to set motor velocity
    void set_motor_velocity(double drive_velocity, int8_t direction) override {
        // In this simulation we update the plant model directly
        // Note: drive_velocity from controller already includes sign for direction
        // (Ruckig generates negative velocities for retract)
        plant_.update(drive_velocity, get_sampling_time());
    }

    // Reset the derived class state (call when starting new cycle)
    void reset_sensor_state() {
        last_position_ = plant_.get_position();
        mass_velocity_ = 0.0;
    }
    
private:
    PlantModel& plant_;
    double last_position_ = 0.0;
    double mass_velocity_ = 0.0;
};

// Log data structure using vectors for CSV export
struct LogData {
    std::vector<double> time;
    std::vector<double> drive_position;
    std::vector<double> drive_velocity;
    std::vector<double> mass_position;
    std::vector<double> mass_velocity;
    
    void clear() {
        time.clear();
        drive_position.clear();
        drive_velocity.clear();
        mass_position.clear();
        mass_velocity.clear();
    }
    
    void add(double t, double dp, double dv, double mp, double mv) {
        time.push_back(t);
        drive_position.push_back(dp);
        drive_velocity.push_back(dv);
        mass_position.push_back(mp);
        mass_velocity.push_back(mv);
    }
    
    bool save(const std::string& filename) const {
        return save_columns_to_csv(
            {time, drive_position, drive_velocity, mass_position, mass_velocity},
            filename,
            "time,drive_position,drive_velocity,mass_position,mass_velocity"
        );
    }
};

int main () {
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

    // Loop through 2 cycles: extend + retract
    for (int cycle = 0; cycle < 2; ++cycle) {
        int step = 0;
        double mass_position = 0.0;
        double mass_velocity = 0.0;

        // Reset plant and controller for new cycle (except first)
        if (cycle > 0) {
            plant.reset();
            controller.reset_trajectory();
            controller.reset_sensor_state();
        }

        // Start extend phase
        controller.start_extend();
        std::cout << "Cycle " << cycle + 1 << ": Starting extend phase\n";

        // Extend loop - each iteration is 1 ms
        while (controller.get_motion_state() != MotionState::At_Final_Distance && step < max_steps) {
            mass_position = controller.read_mass_position();
            mass_velocity = controller.read_mass_velocity();

            controller.velocity_control(mass_position, mass_velocity);
            controller.set_motor_velocity(controller.get_control_velocity(), controller.get_direction());

            // Log data (time in seconds for plotting)
            log.add(total_time_s, controller.get_drive_position(), controller.get_control_velocity(), mass_position, mass_velocity);

            ++step;
            total_time_s += dt;
        }

        std::cout << "Cycle " << cycle + 1 << ": Extend complete at position " << mass_position << " mm (step " << step << ")\n";

        // Dwell to let spring settle - continue updating plant with zero drive velocity
        const int dwell_steps = 500; // 500 ms dwell
        for (int dwell = 0; dwell < dwell_steps; ++dwell) {
            // Update plant with zero drive velocity so spring can settle
            // Use set_motor_velocity to keep consistent with main loop
            controller.set_motor_velocity(0.0, 1);

            mass_position = controller.read_mass_position();
            mass_velocity = controller.read_mass_velocity();
            
            // Log data during dwell
            log.add(total_time_s, controller.get_drive_position(), 0.0, mass_position, mass_velocity);
            total_time_s += dt;
        }

        // Start retract phase
        controller.start_retract();
        std::cout << "Cycle " << cycle + 1 << ": Starting retract phase\n";

        // Retract loop - each iteration is 1 ms
        while (controller.get_motion_state() != MotionState::Home && step < max_steps * 2) {
            mass_position = controller.read_mass_position();
            mass_velocity = controller.read_mass_velocity();

            controller.velocity_control(mass_position, mass_velocity);
            controller.set_motor_velocity(controller.get_control_velocity(), controller.get_direction());

            // Log data (time in seconds for plotting)
            log.add(total_time_s, controller.get_drive_position(), controller.get_control_velocity(), mass_position, mass_velocity);

            ++step;
            total_time_s += dt;
        }

        std::cout << "Cycle " << cycle + 1 << ": Retract complete at position " << mass_position << " mm (step " << step << ")\n";
    }

    // Save log to CSV
    if (log.save("MIL2_simulation.csv")) {
        std::cout << "Data saved to MIL2_simulation.csv\n";
    } else {
        std::cerr << "Error: Failed to save data to CSV\n";
    }

    return 0;
}