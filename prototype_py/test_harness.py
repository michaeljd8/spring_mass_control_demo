"""
Test Harness for Spring-Mass System and PID Controller

This script integrates the SpringMassSystem and PID Controller classes to test the performance of the controller in driving the cart to the target velocity.
"""

import numpy as np
import matplotlib.pyplot as plt
from controller import Controller
from plant_model import SpringMassSystem

# Initialize the plant model and controller
mass = 1.0
spring_constant = 10.0
damping = 0.5
viscous_friction = 0.1
coulomb_friction = 0.5

plant = SpringMassSystem(mass, spring_constant, damping, viscous_friction, coulomb_friction)

# Define target parameters
target_velocity = 2.0  # m/s
controller = Controller(target_velocity, kp=0, ki=0., kd=0)

# Simulation parameters
time_step = 0.01  # seconds
total_time = 20.0  # seconds
t_span = np.arange(0, total_time, time_step)

# Initialize state variables
current_velocity = 0.0
current_distance = 0.0

# Data storage for plotting
control_velocities = []
cart_velocities = []
cart_positions = []

# Ensure the control velocity starts at the target velocity for open-loop control
control_velocity = target_velocity

# Ensure the input function provides both displacement and velocity
# Initialize the input displacement and velocity
input_displacement = 0.0
input_velocity = target_velocity

# Simulation loop
for t in t_span:
    # Get velocity adjustment from the PID controller
    velocity_adjustment = controller.update(current_velocity, time_step)

    # Calculate the new control velocity by applying the adjustment
    control_velocity += velocity_adjustment

    # Update the input displacement and velocity for the plant model
    input_displacement += input_velocity * time_step
    input_velocity = control_velocity

    # Simulate the plant model for one time step
    def input_func(_):
        return input_displacement, input_velocity

    result = plant.simulate([t, t + time_step], input_func)

    # Update current state
    current_velocity = result['velocity'][-1]
    current_distance += current_velocity * time_step

    # Store data for plotting
    control_velocities.append(control_velocity)
    cart_velocities.append(current_velocity)
    cart_positions.append(current_distance)

# Plot results
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax1.plot(t_span, control_velocities, label='Control Velocity', color='blue')
ax1.set_ylabel('Control Velocity (m/s)')
ax1.legend()
ax1.grid(True)

ax2.plot(t_span, cart_velocities, label='Cart Velocity', color='green')
ax2.set_ylabel('Cart Velocity (m/s)')
ax2.legend()
ax2.grid(True)

ax3.plot(t_span, cart_positions, label='Cart Position', color='red')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Cart Position (m)')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.show()
