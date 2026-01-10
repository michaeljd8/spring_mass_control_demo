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