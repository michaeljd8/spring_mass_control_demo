"""
Visualization script for MIL Level 3 simulation results.
Plots the drive position, drive velocity, mass position, mass velocity, and motion state over time.
Tests the update() state machine loop with state transition tracking.
"""

import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

# Determine the CSV file path
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_file = os.path.join(script_dir, "../build/MIL3_simulation.csv")

# Check if file exists
if not os.path.exists(csv_file):
    print(f"Error: CSV file not found: {csv_file}")
    print("Run ./MIL_3_test first to generate the data.")
    sys.exit(1)

# Read the CSV file
data = pd.read_csv(csv_file)

# Extract data
time = data['time']
drive_position = data['drive_position']
drive_velocity = data['drive_velocity']
mass_position = data['mass_position']
mass_velocity = data['mass_velocity']
motion_state = data['motion_state']

# Motion state enum mapping
STATE_NAMES = {
    0: 'Home',
    1: 'Extending',
    2: 'Final_Velocity',
    3: 'At_Final_Distance',
    4: 'Retracting',
    5: 'Manual_Stop',
    6: 'Error'
}

STATE_COLORS = {
    0: 'green',        # Home
    1: 'blue',         # Extending
    2: 'cyan',         # Final_Velocity
    3: 'purple',       # At_Final_Distance
    4: 'orange',       # Retracting
    5: 'red',          # Manual_Stop
    6: 'darkred'       # Error
}

# Create the plots - single figure with 3 subplots (stacked vertically)
fig, axs = plt.subplots(3, 1, figsize=(14, 10))
fig.suptitle('MIL Level 3: State Machine Update Loop Test - Extend/Retract Cycles', fontsize=14)

# Plot 1: Positions over time with state coloring (top)
axs[0].plot(time, drive_position, label='Drive Position', color='blue', alpha=0.7)
axs[0].plot(time, mass_position, label='Mass Position', color='orange', alpha=0.7)
axs[0].set_title('Positions Over Time')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Position (mm)')
axs[0].legend()
axs[0].grid()
axs[0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
axs[0].axhline(y=80, color='red', linestyle='--', alpha=0.3, label='Final Distance')

# Plot 2: Velocities over time (middle)
axs[1].plot(time, drive_velocity, label='Drive Velocity', color='blue', alpha=0.7)
axs[1].plot(time, mass_velocity, label='Mass Velocity', color='orange', alpha=0.7)
axs[1].set_title('Velocities Over Time')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Velocity (mm/s)')
axs[1].legend()
axs[1].grid()
axs[1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
axs[1].axhline(y=15, color='green', linestyle='--', alpha=0.3, label='Final Velocity')

# Plot 3: Motion State over time (bottom)
# Create colored regions for each state
prev_state = motion_state.iloc[0]
prev_time = time.iloc[0]

for i in range(1, len(time)):
    if motion_state.iloc[i] != prev_state or i == len(time) - 1:
        # Draw region for previous state
        state = int(prev_state)
        color = STATE_COLORS.get(state, 'gray')
        axs[2].axvspan(prev_time, time.iloc[i], alpha=0.3, color=color)
        
        prev_state = motion_state.iloc[i]
        prev_time = time.iloc[i]

# Plot state as line
axs[2].plot(time, motion_state, 'k-', linewidth=0.5)
axs[2].set_title('Motion State Over Time')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('State')
axs[2].set_yticks(list(STATE_NAMES.keys()))
axs[2].set_yticklabels(list(STATE_NAMES.values()))
axs[2].grid(axis='x')

# Add legend for states
from matplotlib.patches import Patch
legend_elements = [Patch(facecolor=STATE_COLORS[k], alpha=0.3, label=STATE_NAMES[k]) 
                   for k in sorted(STATE_NAMES.keys()) if k in motion_state.values]
axs[2].legend(handles=legend_elements, loc='upper right', ncol=3)

plt.tight_layout()
plt.show()

# Print summary statistics
print("\n=== MIL Level 3 Simulation Summary ===")
print(f"Total simulation time: {time.iloc[-1]:.3f} s")
print(f"Number of data points: {len(time)}")

print(f"\nPosition Statistics:")
print(f"  Max drive position: {drive_position.max():.2f} mm")
print(f"  Max mass position: {mass_position.max():.2f} mm")
print(f"  Min drive position: {drive_position.min():.2f} mm")
print(f"  Min mass position: {mass_position.min():.2f} mm")

print(f"\nVelocity Statistics:")
print(f"  Max drive velocity: {drive_velocity.max():.2f} mm/s")
print(f"  Min drive velocity: {drive_velocity.min():.2f} mm/s")
print(f"  Max mass velocity: {mass_velocity.max():.2f} mm/s")
print(f"  Min mass velocity: {mass_velocity.min():.2f} mm/s")

print(f"\nState Transitions:")
state_changes = motion_state.diff().fillna(0) != 0
transitions = data[state_changes]
for idx, row in transitions.iterrows():
    prev_idx = idx - 1 if idx > 0 else 0
    prev_state = int(data.iloc[prev_idx]['motion_state'])
    new_state = int(row['motion_state'])
    print(f"  t={row['time']:.3f}s: {STATE_NAMES.get(prev_state, '?')} -> {STATE_NAMES.get(new_state, '?')}")
