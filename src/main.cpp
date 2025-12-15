#include "PlantModel.h"
#include <iostream>

int main() {
    // Example usage of the PlantModel
    PlantModel plant;

    // Initialize the plant model with parameters
    plant.setMass(10.0); // Mass in kg
    plant.setSpringConstant(100.0); // Spring constant in N/m

    // Simulate the plant model
    double controlVelocity = 5.0; // Control velocity in m/s
    plant.update(controlVelocity, 0.01); // Update with a time step of 0.01 seconds

    // Output the results
    std::cout << "Cart Velocity: " << plant.getCartVelocity() << " m/s" << std::endl;
    std::cout << "Cart Position: " << plant.getCartPosition() << " m" << std::endl;

    return 0;
}