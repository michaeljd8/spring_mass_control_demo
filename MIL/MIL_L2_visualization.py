"""
Visualization script for MIL Level 2 simulation results.
Plots the drive position, drive velocity, mass position, and mass velocity over time.
Shows multiple extend/retract cycles using the virtual function override pattern.
"""

import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# Determine the CSV file path
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_file = os.path.join(script_dir, "../build/MIL2_simulation.csv")

# Check if file exists
if not os.path.exists(csv_file):
    print(f"Error: CSV file not found: {csv_file}")
    print("Run ./MIL_2_test first to generate the data.")
    sys.exit(1)

# Read the CSV file
data = pd.read_csv(csv_file)

# Extract data
time = data['time']
drive_position = data['drive_position']
drive_velocity = data['drive_velocity']
mass_position = data['mass_position']
mass_velocity = data['mass_velocity']

# Create the plots - single figure with 2 subplots (stacked vertically)
fig, axs = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle('MIL Level 2: Virtual Function Override Pattern - Extend/Retract Cycles', fontsize=14)

# Plot 1: Positions over time (top)
axs[0].plot(time, drive_position, label='Drive Position', color='blue')
axs[0].plot(time, mass_position, label='Mass Position', color='orange')
axs[0].set_title('Positions Over Time')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Position (mm)')
axs[0].legend()
axs[0].grid()
axs[0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

# Plot 2: Velocities over time (bottom)
axs[1].plot(time, drive_velocity, label='Drive Velocity', color='blue')
axs[1].plot(time, mass_velocity, label='Mass Velocity', color='orange')
axs[1].set_title('Velocities Over Time')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Velocity (mm/s)')
axs[1].legend()
axs[1].grid()
axs[1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

plt.tight_layout()
plt.show()

# Print summary statistics
print("\n=== MIL Level 2 Simulation Summary ===")
print(f"Total simulation time: {time.iloc[-1]:.3f} s")
print(f"Number of data points: {len(time)}")
print(f"\nPosition Statistics:")
print(f"  Max drive position: {drive_position.max():.2f} mm")
print(f"  Max mass position: {mass_position.max():.2f} mm")
print(f"\nVelocity Statistics:")
print(f"  Max drive velocity: {drive_velocity.max():.2f} mm/s")
print(f"  Min drive velocity: {drive_velocity.min():.2f} mm/s")
print(f"  Max mass velocity: {mass_velocity.max():.2f} mm/s")
print(f"  Min mass velocity: {mass_velocity.min():.2f} mm/s")
