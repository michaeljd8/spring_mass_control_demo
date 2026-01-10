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

int main() {
    // Create PlantModel with light mass and stiff spring for responsive dynamics
    PlantModel plant(0.1, 100.0, 1.0);

    double final_velocity = 15.0; // mm/s
    double approach_distance = 70.0; // mm
    double final_distance = 80.0; // mm
    double approach_offset = 15.0; // mm
    double travel_velocity = 100.0; // mm/s
    double acceleration = 250.0; // mm/s^2

    // Create SpringMassControlDemo with custom parameters
    SpringMassControlDemo controller(final_velocity, approach_distance, final_distance, approach_offset, travel_velocity, acceleration);

    // Set PID values to 0 for open loop test
    controller.set_pid_gains(0.0, 0.0, 0.0);
    /*
    Simulate the spring-mass-damper system
    Read mass position and velocity from PlantModel
    Use SpringMassControlDemo to compute drive velocity
    Save graph data for analysis
        Columns: time, drive_position, drive_velocity, mass_position, mass_velocity
    Loop through until mass velocity reaches final distance
    */
    const int max_steps = 10000;
    int steps = 0;

    // Container to store data for graphing
    std::vector<DataPoint> log;

    // Track drive position by integrating drive velocity
    double drive_position = 0.0;

    // Get Initial control velocity
    double control_velocity = controller.get_control_velocity();

    while (plant.get_position() < controller.get_final_distance() && steps < max_steps) { 
        double mass_position = plant.get_position();
        double mass_velocity = plant.get_velocity();

        double dt = controller.get_sampling_time();

        // Update the control velocity using the PID controller
        controller.velocity_control(control_velocity, mass_position, mass_velocity);

        // Get the updated control velocity
        control_velocity = controller.get_control_velocity();

        // Update PlantModel with computed control velocity
        plant.update(control_velocity, dt);

        // Update drive position by integrating control velocity
        drive_position += control_velocity * dt;

        // compute current time and append a new sample
        double time = steps * dt;


        log.push_back({ time, drive_position, control_velocity, mass_position, mass_velocity });


        ++steps;
    }

    if (steps >= max_steps) {
        std::cerr << "Warning: reached max steps (" << max_steps << ") before reaching target distance.\n";
    }

    // Output log data to CSV file for graphing
    save_to_csv(log, "MIL1_simulation_results.csv");

    std::cout << "Simulation completed in " << steps << " steps.\n";
    std::cout << "Final mass position: " << plant.get_position() << " mm\n";
    std::cout << "Final mass velocity: " << plant.get_velocity() << " mm/s\n";

    return 0;
}