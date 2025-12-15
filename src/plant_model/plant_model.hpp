#ifndef PLANT_MODEL_HPP
#define PLANT_MODEL_HPP

#include <cmath>
#include <algorithm>

/**
 * Spring-Mass System Simulation
 * 
 * Simulates a spring-mass-damper system with friction.
 * Input: displacement at the spring
 * Output: position of the mass
 * 
 * Equation: m * x_ddot + c * x_dot + k * x = k * x_in + c * x_in_dot
 */
class PlantModel {
public:
    // System parameters
    double mass;              // Mass (kg)
    double spring_constant;   // Spring stiffness k (N/m)
    double damping;           // Damping coefficient c (N*s/m)
    double viscous_friction;  // Viscous friction coefficient b (N*s/m)
    double coulomb_friction;  // Coulomb friction force fc (N)

    // State variables
    double position;          // Mass position x (m)
    double velocity;          // Mass velocity x_dot (m/s)

    /**
     * Constructor with default parameters
     */
    PlantModel(double m = 1.0, double k = 10.0, double c = 0.5,
               double b = 0.1, double fc = 0.5);

    /**
     * Reset the system state to initial conditions
     */
    void reset(double initial_position = 0.0, double initial_velocity = 0.0);

    /**
     * Compute acceleration given current state and input
     * 
     * @param x_in Input displacement (m)
     * @param x_in_dot Input velocity (m/s)
     * @return Acceleration of the mass (m/s^2)
     */
    double compute_acceleration(double x_in, double x_in_dot) const;

    /**
     * Update the system state using Euler integration
     * 
     * @param x_in Input displacement (m)
     * @param x_in_dot Input velocity (m/s)
     * @param dt Time step (s)
     */
    void update(double x_in, double x_in_dot, double dt);

    /**
     * Update using 4th-order Runge-Kutta integration (more accurate)
     * 
     * @param x_in Input displacement (m)
     * @param x_in_dot Input velocity (m/s)
     * @param dt Time step (s)
     */
    void update_rk4(double x_in, double x_in_dot, double dt);

    /**
     * Get current position
     */
    double get_position() const;

    /**
     * Get current velocity
     */
    double get_velocity() const;

private:
    static double sign(double x);
};

#endif // PLANT_MODEL_HPP
