/**
 * Model in Loop Level 1 for SpringMassControlDemo.hpp
 * 
 * PlantModel to simulate the spring-mass-damper system.
 * SpringMassControlDemo to manage the control logic.
 */

#include "PlantModel.hpp"
#include "SpringMassControlDemo.hpp"
#include "save_to_csv.hpp"
#include <iostream>
#include <cmath>
#include <vector>

// Type alias for cleaner MotionState access
using MotionState = SpringMassControlDemo::MotionState;

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

// Function to simulate actuation cycle (extend + retract)
void run_actuation_cycle(PlantModel& plant, SpringMassControlDemo& controller, LogData& log, int max_steps) {
    int steps = 0;
    
    // Create time variable
    double time = 0.0;

    // Start extend phase
    controller.start_extend();

    // Extend loop
    while (controller.get_motion_state() != MotionState::At_Final_Distance && steps < max_steps) {
        double mass_position = plant.get_position();
        double mass_velocity = plant.get_velocity();
        double dt = controller.get_sampling_time();

        controller.velocity_control(mass_position, mass_velocity);
        plant.update(controller.get_control_velocity(), dt);

        time += dt;
        log.add(time, controller.get_drive_position(), controller.get_control_velocity(), mass_position, mass_velocity);
        ++steps;
    }

    // Start retract phase
    controller.start_retract();

    // Retract loop
    while (controller.get_motion_state() != MotionState::Home && steps < max_steps) {
        double mass_position = plant.get_position();
        double mass_velocity = plant.get_velocity();
        double dt = controller.get_sampling_time();

        controller.velocity_control(mass_position, mass_velocity);
        plant.update(controller.get_control_velocity(), dt);

        time += dt;
        log.add(time, controller.get_drive_position(), controller.get_control_velocity(), mass_position, mass_velocity);
        ++steps;
    }

    if (steps >= max_steps) {
        std::cerr << "Warning: reached max steps (" << max_steps << ") before completing actuation cycle.\n";
    }
}


int main() {
    // Create PlantModel with light mass and stiff spring for responsive dynamics
    PlantModel plant(0.1, 100.0, 1.0);

    double final_velocity = 15.0; // mm/s
    double approach_distance = 78.0; // mm
    double final_distance = 80.0; // mm
    double approach_offset = 5.0; // mm
    double travel_velocity = 100.0; // mm/s
    double acceleration = 250.0; // mm/s^2

    // Create SpringMassControlDemo with custom parameters
    SpringMassControlDemo controller(final_velocity, approach_distance, final_distance, approach_offset, travel_velocity, acceleration);
    
    // Set PID gains to 0 for open loop benchmark
    controller.set_pid_gains(0.0, 0.0, 0.0);

    // Set maximum simulation steps to avoid infinite loops
    const int max_steps = 20000;

    // Container to store data for graphing
    LogData log;

    // Run actuation cycle (extend + retract)
    run_actuation_cycle(plant, controller, log, max_steps);

    // Save open loop data to CSV
    log.save("MIL1_open_loop_simulation.csv");
    std::cout << "Data saved to MIL1_open_loop_simulation.csv\n";

    // Set P only control for closed loop simulation
    controller.set_pid_gains(4.0, 0.0, 0.0);

    // Clear log
    log.clear();

    // Run actuation cycle (extend + retract)
    run_actuation_cycle(plant, controller, log, max_steps);

    log.save("MIL1_closed_loop_simulation.csv");
    std::cout << "Data saved to MIL1_closed_loop_simulation.csv\n";

    return 0;
}