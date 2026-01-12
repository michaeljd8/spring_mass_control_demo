"""
Visualization script for MIL Level 1 simulation results.
Plots the drive velocity, mass position, and mass velocity over time.
Supports multiple simulation runs (e.g., open loop vs PID control).
"""

import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# Determine the CSV file paths
script_dir = os.path.dirname(os.path.abspath(__file__))
open_loop_csv = os.path.join(script_dir, "../build/MIL1_open_loop_simulation_results.csv")
closed_loop_csv = os.path.join(script_dir, "../build/MIL1_closed_loop_simulation_results.csv")

# Check if files exist
if not os.path.exists(open_loop_csv):
    print(f"Error: Open loop CSV file not found: {open_loop_csv}")
    sys.exit(1)
if not os.path.exists(closed_loop_csv):
    print(f"Error: Closed loop CSV file not found: {closed_loop_csv}")
    sys.exit(1)

# Read the CSV files
open_loop_data = pd.read_csv(open_loop_csv)
closed_loop_data = pd.read_csv(closed_loop_csv)

# Extract data for plotting
time_ol = open_loop_data['time']
drive_position_ol = open_loop_data['drive_position']
drive_velocity_ol = open_loop_data['drive_velocity']
mass_position_ol = open_loop_data['mass_position']
mass_velocity_ol = open_loop_data['mass_velocity']

# Extract data for closed loop
time_cl = closed_loop_data['time']
drive_position_cl = closed_loop_data['drive_position']
drive_velocity_cl = closed_loop_data['drive_velocity']
mass_position_cl = closed_loop_data['mass_position']
mass_velocity_cl = closed_loop_data['mass_velocity']

# Create the plots
fig, axs = plt.subplots(2, 2, figsize=(10, 12))

# Plot 1: Open Loop Positions
axs[0, 0].plot(time_ol, drive_position_ol, label='Drive Position (Open Loop)', color='blue')
axs[0, 0].plot(time_ol, mass_position_ol, label='Mass Position (Open Loop)', color='orange')
axs[0, 0].set_title('Open Loop: Drive and Mass Positions')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Position (mm)')
axs[0, 0].legend()
axs[0, 0].grid()

# Plot 2: Open Loop Velocities
axs[1, 0].plot(time_ol, drive_velocity_ol, label='Drive Velocity (Open Loop)', color='blue')
axs[1, 0].plot(time_ol, mass_velocity_ol, label='Mass Velocity (Open Loop)', color='orange')
axs[1, 0].set_title('Open Loop: Drive and Mass Velocities')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Velocity (mm/s)')
axs[1, 0].legend()
axs[1, 0].grid()

# Plot 3: Closed Loop Positions
axs[0, 1].plot(time_cl, drive_position_cl, label='Drive Position (Closed Loop)', color='blue')
axs[0, 1].plot(time_cl, mass_position_cl, label='Mass Position (Closed Loop)', color='orange')
axs[0, 1].set_title('Closed Loop: Drive and Mass Positions')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Position (mm)')
axs[0, 1].legend()
axs[0, 1].grid()

# Plot 4: Closed Loop Velocities
axs[1, 1].plot(time_cl, drive_velocity_cl, label='Drive Velocity (Closed Loop)', color='blue')
axs[1, 1].plot(time_cl, mass_velocity_cl, label='Mass Velocity (Closed Loop)', color='orange')
axs[1, 1].set_title('Closed Loop: Drive and Mass Velocities')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Velocity (mm/s)')
axs[1, 1].legend()
axs[1, 1].grid()

plt.show()
