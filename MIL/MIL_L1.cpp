/**
 * Model in Loop Level 1 for SpringMassControlDemo.hpp
 * 
 * PlantModel to simulate the spring-mass-damper system.
 * SpringMassControlDemo to manage the control logic.
 */

#include "../PlantModel/PlantModel.hpp"
#include "../src/SpringMassControlDemo.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

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

// Function to run extend simulation - stops when mass reaches approach distance
void run_extend_simulation(PlantModel& plant, SpringMassControlDemo& controller, std::vector<DataPoint>& log, int max_steps, double time_offset = 0.0) {
    int steps = 0;
    double control_velocity = controller.get_control_velocity();
    double target_distance = controller.get_approach_distance() - controller.get_approach_offset();

    while (plant.get_position() < target_distance && steps < max_steps) {
        double mass_position = plant.get_position();
        double mass_velocity = plant.get_velocity();
        double dt = controller.get_sampling_time();

        controller.velocity_control(control_velocity, mass_position, mass_velocity);
        control_velocity = controller.get_control_velocity();
        plant.update(control_velocity, dt);

        double drive_position = controller.get_drive_position();
        double time = time_offset + steps * dt;
        log.push_back({ time, drive_position, control_velocity, mass_position, mass_velocity });

        ++steps;
    }

    if (steps >= max_steps) {
        std::cerr << "Warning: reached max steps (" << max_steps << ") before completing extend.\n";
    }
}

// Function to run retract simulation - stops when motion state becomes Idle
void run_retract_simulation(PlantModel& plant, SpringMassControlDemo& controller, std::vector<DataPoint>& log, int max_steps, double time_offset = 0.0) {
    int steps = 0;
    double control_velocity = controller.get_control_velocity();

    while (controller.get_motion_state() != MotionState::Idle && steps < max_steps) {
        double mass_position = plant.get_position();
        double mass_velocity = plant.get_velocity();
        double dt = controller.get_sampling_time();

        controller.velocity_control(control_velocity, mass_position, mass_velocity);
        control_velocity = controller.get_control_velocity();
        plant.update(control_velocity, dt);

        double drive_position = controller.get_drive_position();
        double time = time_offset + steps * dt;
        log.push_back({ time, drive_position, control_velocity, mass_position, mass_velocity });

        ++steps;
    }

    if (steps >= max_steps) {
        std::cerr << "Warning: reached max steps (" << max_steps << ") before completing retract.\n";
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

    const int max_steps = 20000;

    // Container to store data for graphing
    std::vector<DataPoint> log;

    // ===== EXTEND PHASE =====
    std::cout << "Starting EXTEND phase...\n";
    controller.start_extend();
    
    // Run the extend simulation
    run_extend_simulation(plant, controller, log, max_steps);

    std::cout << "Extend completed at position: " << plant.get_position() << " mm\n";
    std::cout << "Extend completed at velocity: " << plant.get_velocity() << " mm/s\n";

    // ===== RETRACT PHASE =====
    std::cout << "\nStarting RETRACT phase...\n";
    controller.start_retract();

    // Continue simulation for retract (append to same log for continuous data)
    double time_offset = log.empty() ? 0.0 : log.back().time + controller.get_sampling_time();
    
    run_retract_simulation(plant, controller, log, max_steps, time_offset);

    // Output combined extend/retract log data to CSV file
    save_to_csv(log, "MIL1_open_loop_simulation.csv");

    // Delete and create new instances of the plant and controller
    plant = PlantModel(0.1, 100.0, 1.0);
    // Create a new controller instance for the closed-loop run instead of assigning to the existing one
    SpringMassControlDemo controller_closed(final_velocity, approach_distance, final_distance, approach_offset, travel_velocity, acceleration);
    
    // Set PID gains for closed loop control - P only for demo
    controller_closed.set_pid_gains(10.0, 0.0, 0.0);

    // Clear log for closed loop data
    log.clear();

    // ===== EXTEND PHASE (Closed Loop) =====
    std::cout << "\nStarting EXTEND phase (Closed Loop)...\n";
    controller_closed.start_extend();
    run_extend_simulation(plant, controller_closed, log, max_steps);
    std::cout << "Extend completed at position: " << plant.get_position() << " mm\n";
    std::cout << "Extend completed at velocity: " << plant.get_velocity() << " mm/s\n";

    // ===== RETRACT PHASE (Closed Loop) =====
    std::cout << "\nStarting RETRACT phase (Closed Loop)...\n";
    controller_closed.start_retract();
    time_offset = log.empty() ? 0.0 : log.back().time + controller_closed.get_sampling_time();
    run_retract_simulation(plant, controller_closed, log, max_steps, time_offset);
    save_to_csv(log, "MIL1_closed_loop_simulation.csv");


    return 0;
}