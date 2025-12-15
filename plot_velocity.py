import matplotlib
matplotlib.use('TkAgg')

import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
# Updated to use simulation_data.csv
data = pd.read_csv("simulation_data.csv")

# Plot Position and Velocity
plt.figure(figsize=(10, 6))
plt.plot(data["time"], data["position"], label="Position", linestyle="--")
plt.plot(data["time"], data["velocity"], label="Velocity")

# Add labels and legend
plt.title("Position and Velocity vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Values")
plt.legend()
plt.grid()

# Save the plot
plt.savefig("velocity_plot.png")
plt.show()