import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
# Updated to use simulation_data.csv
data = pd.read_csv("../build/plant_model_log.csv")

# Columns: time, drive_velocity, mass_position, mass_velocity
time = data['time']
drive_velocity = data['drive_velocity']
mass_position = data['mass_position']
mass_velocity = data['mass_velocity']

# Create the plots
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(time, drive_velocity, label='Drive Velocity', color='blue', marker='o')
plt.plot(time, mass_velocity, label='Mass Velocity', color='green')
plt.title('Velocities vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (mm/s)')
plt.grid()
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, mass_position, label='Mass Position', color='orange')
plt.title('Mass Position vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Mass Position (mm)')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()

# ============ Back and Forth Motion Visualization ============
data_back_forth = pd.read_csv("../build/back_and_forth_motion.csv")

time_bf = data_back_forth['time']
drive_velocity_bf = data_back_forth['drive_velocity']
drive_position_bf = data_back_forth['drive_position']
mass_position_bf = data_back_forth['mass_position']
mass_velocity_bf = data_back_forth['mass_velocity']

plt.figure(figsize=(12, 10))

plt.subplot(3, 1, 1)
plt.plot(time_bf, drive_velocity_bf, label='Drive Velocity', color='blue')
plt.plot(time_bf, mass_velocity_bf, label='Mass Velocity', color='green')
plt.title('Back and Forth Motion - Velocities vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (mm/s)')
plt.grid()
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_bf, drive_position_bf, label='Drive Position', color='blue')
plt.plot(time_bf, mass_position_bf, label='Mass Position', color='orange')
plt.title('Back and Forth Motion - Positions vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (mm)')
plt.grid()
plt.legend()

plt.subplot(3, 1, 3)
position_error = drive_position_bf - mass_position_bf
plt.plot(time_bf, position_error, label='Position Error (Drive - Mass)', color='red')
plt.title('Back and Forth Motion - Position Error vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (mm)')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()