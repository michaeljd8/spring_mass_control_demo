/**
 * Model in Loop Level 1 for SpringMassControlDemo.hpp
 * 
 * PlantModel to simulate the spring-mass-damper system.
 * SpringMassControlDemo to manage the control logic.
 */

#include "PlantModel.hpp"
#include "SpringMassControlDemo.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

// Include util for saving CSV
#include "utils/save_to_csv.hpp"

// Type alias for cleaner MotionState access
using MotionState = SpringMassControlDemo::MotionState;

// Data point type
struct DataPoint {
    double time;
    double drive_position;
    double drive_velocity;
    double mass_position;
    double mass_velocity;
};

// Function to save log data to CSV file
void save_to_csv(const std::vector<DataPoint>& log, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return;
    }

    // Write CSV header
    file << "time,drive_position,drive_velocity,mass_position,mass_velocity\n";

    // Write data rows
    for (const auto& point : log) {
        file << point.time << ","
             << point.drive_position << ","
             << point.drive_velocity << ","
             << point.mass_position << ","
             << point.mass_velocity << "\n";
    }

    file.close();
    std::cout << "Data saved to " << filename << "\n";
}

// Function to simulate actuation cycle (extend + retract)
void run_actuation_cycle(PlantModel& plant, SpringMassControlDemo& controller, std::vector<DataPoint>& log, int max_steps) {
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
        log.push_back({ time, controller.get_drive_position(), controller.get_control_velocity(), mass_position, mass_velocity });
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
        log.push_back({ time, controller.get_drive_position(), controller.get_control_velocity(), mass_position, mass_velocity });
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
    std::vector<DataPoint> log;

    // Run actuation cycle (extend + retract)
    run_actuation_cycle(plant, controller, log, max_steps);

    // Save open loop data to CSV
    save_to_csv(log, "MIL1_open_loop_simulation.csv");

    // Set P only control for closed loop simulation
    controller.set_pid_gains(4.0, 0.0, 0.0);

    // Clear log
    log.clear();

    // Run actuation cycle (extend + retract)
    run_actuation_cycle(plant, controller, log, max_steps);

    save_to_csv(log, "MIL1_closed_loop_simulation.csv");

    return 0;
}