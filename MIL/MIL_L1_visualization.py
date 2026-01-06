"""
Visualization script for MIL Level 1 simulation results.
Plots the drive velocity, mass position, and mass velocity over time.
"""

import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# Determine the CSV file path
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
else:
    # Default to build directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, "../build/MIL1_simulation_results.csv")

# Check if file exists
if not os.path.exists(csv_file):
    print(f"Error: CSV file not found: {csv_file}")
    sys.exit(1)

# Read the CSV file
data = pd.read_csv(csv_file)

# Extract columns
time = data['time']
drive_position = data['drive_position']
drive_velocity = data['drive_velocity']
mass_position = data['mass_position']
mass_velocity = data['mass_velocity']

# Create figure with subplots
fig, axes = plt.subplots(2, 1, figsize=(12, 10))
fig.suptitle('MIL Level 1 Simulation Results', fontsize=16, fontweight='bold')

# Plot 1: Drive Position and Mass Position
axes[0].plot(time, drive_position, label='Drive Position', color='blue', linewidth=2)
axes[0].plot(time, mass_position, label='Mass Position', color='orange', linewidth=2)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Position (mm)')
axes[0].set_title('Position vs Time')
axes[0].grid(True, alpha=0.3)
axes[0].legend()

# Plot 2: Drive Velocity and Mass Velocity
axes[1].plot(time, drive_velocity, label='Drive Velocity', color='blue', linewidth=2)
axes[1].plot(time, mass_velocity, label='Mass Velocity', color='green', linewidth=2)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Velocity (mm/s)')
axes[1].set_title('Velocities vs Time')
axes[1].grid(True, alpha=0.3)
axes[1].legend()


plt.tight_layout()

# Print summary statistics
print("\n=== Simulation Summary ===")
print(f"Total simulation time: {time.iloc[-1]:.3f} s")
print(f"Total steps: {len(time)}")
print(f"Final drive position: {drive_position.iloc[-1]:.6f} mm")
print(f"Final mass position: {mass_position.iloc[-1]:.6f} mm")
print(f"Final mass velocity: {mass_velocity.iloc[-1]:.6e} mm/s")
print(f"Max mass velocity: {mass_velocity.max():.6f} mm/s")

plt.show()
