/*
Spring-Mass System Simulation

Simulates a spring-mass-damper system with friction.
Input: displacement at the spring
Output: position of the mass

Equation: m * x_ddot + c * x_dot + k * x = k * x_in + c * x_in_dot
*/

#ifndef PLANTMODEL_HPP
#define PLANTMODEL_HPP

#include <cmath>
#include <algorithm>


class PlantModel {
public:

    // Constructor with default parameters
    PlantModel(double m = 1.0, double k = 10.0, double c = 0.5,
               double b = 0.1, double fc = 0.5);

    // Reset the system state to initial conditions
    void reset();

    // Update the system state using Euler integration
    void update(double drive_velocity, double dt);

    // Get current position
    double get_position() const;

    // Get current velocity
    double get_velocity() const;

private:
    // System parameters
    double mass;              // Mass (kg)
    double spring_constant;   // Spring stiffness k (N/mm)
    double damping;           // Damping coefficient c (N*s/mm)
    double viscous_friction;  // Viscous friction coefficient b (N*s/mm)
    double coulomb_friction;  // Coulomb friction force fc (N)

    // State variables
    double drive_position;        // Drive position x_in (mm) - Default to 1 mm/s to ensure movement at start
    double mass_position;          // Mass position x (mm)
    double mass_velocity;          // Mass velocity x_dot (mm/s)

    // Compute acceleration given current state and input
    double compute_acceleration(double drive_position, double drive_velocity) const;

    // Helper function to determine the sign of a value
    static double sign(double x);
};

#endif // PLANTMODEL_HPP
