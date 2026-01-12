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

# Determine the CSV file path
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
else:
    # Default to build directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, "../build/MIL1_closed_loop_simulation_results.csv")

# Check if file exists
if not os.path.exists(csv_file):
    print(f"Error: CSV file not found: {csv_file}")
    sys.exit(1)

# Read the CSV file
data = pd.read_csv(csv_file)

# Check if run_id column exists for multiple runs
if 'run_id' in data.columns:
    runs = data['run_id'].unique()
    num_runs = len(runs)
else:
    # Single run, add a default run_id
    data['run_id'] = 0
    runs = [0]
    num_runs = 1

# Define colors and labels for each run
run_colors = {0: {'drive': 'blue', 'mass': 'orange', 'vel': 'green'},
              1: {'drive': 'red', 'mass': 'purple', 'vel': 'brown'}}
run_labels = {0: 'Open Loop (No PID)', 1: 'PID Control'}

# Create figure with subplots
fig, axes = plt.subplots(2, 1, figsize=(14, 10))
fig.suptitle('MIL Level 1 Simulation Results - Open Loop vs PID Control', fontsize=16, fontweight='bold')

# Plot each run
for run_id in runs:
    run_data = data[data['run_id'] == run_id]
    time = run_data['time']
    drive_position = run_data['drive_position']
    drive_velocity = run_data['drive_velocity']
    mass_position = run_data['mass_position']
    mass_velocity = run_data['mass_velocity']
    
    label_suffix = run_labels.get(run_id, f'Run {run_id}')
    colors = run_colors.get(run_id, {'drive': 'gray', 'mass': 'black', 'vel': 'cyan'})
    linestyle = '-' if run_id == 0 else '--'
    
    # Plot 1: Drive Position and Mass Position
    axes[0].plot(time, drive_position, label=f'Drive Position ({label_suffix})', 
                 color=colors['drive'], linewidth=2, linestyle=linestyle)
    axes[0].plot(time, mass_position, label=f'Mass Position ({label_suffix})', 
                 color=colors['mass'], linewidth=2, linestyle=linestyle)
    
    # Plot 2: Drive Velocity and Mass Velocity
    axes[1].plot(time, drive_velocity, label=f'Drive Velocity ({label_suffix})', 
                 color=colors['drive'], linewidth=2, linestyle=linestyle)
    axes[1].plot(time, mass_velocity, label=f'Mass Velocity ({label_suffix})', 
                 color=colors['vel'], linewidth=2, linestyle=linestyle)

# Configure Plot 1
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Position (mm)')
axes[0].set_title('Position vs Time')
axes[0].grid(True, alpha=0.3)
axes[0].legend(loc='best')

# Configure Plot 2
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Velocity (mm/s)')
axes[1].set_title('Velocities vs Time')
axes[1].grid(True, alpha=0.3)
axes[1].legend(loc='best')

plt.tight_layout()

# Print summary statistics for each run
print("\n=== Simulation Summary ===")
for run_id in runs:
    run_data = data[data['run_id'] == run_id]
    label = run_labels.get(run_id, f'Run {run_id}')
    print(f"\n--- {label} ---")
    print(f"  Total simulation time: {run_data['time'].iloc[-1]:.3f} s")
    print(f"  Total steps: {len(run_data)}")
    print(f"  Final drive position: {run_data['drive_position'].iloc[-1]:.6f} mm")
    print(f"  Final mass position: {run_data['mass_position'].iloc[-1]:.6f} mm")
    print(f"  Final mass velocity: {run_data['mass_velocity'].iloc[-1]:.6e} mm/s")
    print(f"  Max mass velocity: {run_data['mass_velocity'].max():.6f} mm/s")
    
    # Get PID gains from the data if available
    if 'kp' in run_data.columns and 'ki' in run_data.columns and 'kd' in run_data.columns:
        kp = run_data['kp'].iloc[0]
        ki = run_data['ki'].iloc[0]
        kd = run_data['kd'].iloc[0]
        print(f"  PID Gains: Kp={kp}, Ki={ki}, Kd={kd}")
    else:
        print("  PID Gains: Not available in the data")

plt.show()
