import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    # Load the CSV file (columns are: distance, velocity)
    # Use absolute path based on script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, "..", "build", "velocity_profile.csv")
    data = pd.read_csv(file_path, header=None, names=["Distance", "Velocity"])

    # Sampling time from SpringMassControlDemo (1 ms = 0.001 s)
    SAMPLING_TIME = 0.001  # seconds

    # Create time column based on sample index
    data['Time'] = data.index * SAMPLING_TIME

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot Distance vs Time
    ax1.plot(data['Time'], data['Distance'], 'b-', linewidth=1.5, label='Distance')
    ax1.set_ylabel('Distance (mm)')
    ax1.set_title('Distance and Velocity Over Time')
    ax1.legend(loc='upper left')
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Plot Velocity vs Time
    ax2.plot(data['Time'], data['Velocity'], 'r-', linewidth=1.5, label='Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (mm/s)')
    ax2.legend(loc='upper left')
    ax2.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()

    # Print some statistics
    print(f"Total samples: {len(data)}")
    print(f"Total time: {data['Time'].iloc[-1]:.3f} s")
    print(f"Final distance: {data['Distance'].iloc[-1]:.3f} mm")
    print(f"Final velocity: {data['Velocity'].iloc[-1]:.3f} mm/s")
    print(f"Max velocity: {data['Velocity'].max():.3f} mm/s")

if __name__ == "__main__":
    main()
