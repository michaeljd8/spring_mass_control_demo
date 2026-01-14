"""
Visualization script for MIL Level 1 simulation results.
Plots the drive velocity, mass position, and mass velocity over time.
Compares open loop vs closed loop control in a single figure.
"""

import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# Determine the CSV file paths
script_dir = os.path.dirname(os.path.abspath(__file__))
open_loop_csv = os.path.join(script_dir, "../build/MIL1_open_loop_simulation.csv")
closed_loop_csv = os.path.join(script_dir, "../build/MIL1_closed_loop_simulation.csv")

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

# Extract data for open loop
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

# Create the plots - single figure with 2x2 subplots
fig, axs = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('MIL Level 1: Open Loop vs Closed Loop Comparison', fontsize=14)

# Plot 1: Open Loop Positions (top-left)
axs[0, 0].plot(time_ol, drive_position_ol, label='Drive Position', color='blue')
axs[0, 0].plot(time_ol, mass_position_ol, label='Mass Position', color='orange')
axs[0, 0].set_title('Open Loop: Positions')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Position (mm)')
axs[0, 0].legend()
axs[0, 0].grid()
axs[0, 0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

# Plot 2: Closed Loop Positions (top-right)
axs[0, 1].plot(time_cl, drive_position_cl, label='Drive Position', color='blue')
axs[0, 1].plot(time_cl, mass_position_cl, label='Mass Position', color='orange')
axs[0, 1].set_title('Closed Loop: Positions')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Position (mm)')
axs[0, 1].legend()
axs[0, 1].grid()
axs[0, 1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

# Plot 3: Open Loop Velocities (bottom-left)
axs[1, 0].plot(time_ol, drive_velocity_ol, label='Drive Velocity', color='blue')
axs[1, 0].plot(time_ol, mass_velocity_ol, label='Mass Velocity', color='orange')
axs[1, 0].set_title('Open Loop: Velocities')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Velocity (mm/s)')
axs[1, 0].legend()
axs[1, 0].grid()
axs[1, 0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

# Plot 4: Closed Loop Velocities (bottom-right)
axs[1, 1].plot(time_cl, drive_velocity_cl, label='Drive Velocity', color='blue')
axs[1, 1].plot(time_cl, mass_velocity_cl, label='Mass Velocity', color='orange')
axs[1, 1].set_title('Closed Loop: Velocities')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Velocity (mm/s)')
axs[1, 1].legend()
axs[1, 1].grid()
axs[1, 1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

plt.tight_layout()
plt.show()
