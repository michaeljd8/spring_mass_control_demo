/**
 * Model in Loop Level 1 for SpringMassControlDemo.hpp
 * 
 * PlantModel to simulate the spring-mass-damper system.
 * SpringMassControlDemo to manage the control logic.
 */

#include "PlantModel.hpp"
#include "SpringMassControlDemo.hpp"
#include <iostream>
#include <cmath>
#include <vector>


int main() {
    // Create PlantModel with default parameters
    PlantModel plant;

    // Create SpringMassControlDemo with default parameters
    SpringMassControlDemo controller;

    /*
    Simulate the spring-mass-damper system
    Read mass position and velocity from PlantModel
    Use SpringMassControlDemo to compute drive velocity
    Save graph data for analysis
        Columns: time, drive_position, drive_velocity, mass_position, mass_velocity
    Loop through until mass velocity reaches final distance
    */
    while (plant.get_position() < controller.get_final_distance() / 1000.) { // Convert mm to m
        double mass_position = plant.get_position();
        double mass_velocity = plant.get_velocity();

        // Compute drive velocity using controller
        double drive_velocity = 0.0;
        controller.velocity_control(drive_velocity, mass_position, mass_velocity);

        // Update PlantModel with computed drive velocity
        plant.update(drive_velocity, SpringMassControlDemo::get_sampling_time());

        // Log data for graphing (not implemented here)
    }



}