import pandas as pd
import matplotlib.pyplot as plt

def main():
    # Load the CSV file
    file_path = "./build/velocity_profile.csv"  # Adjusted path to locate the CSV file correctly
    data = pd.read_csv(file_path, header=None, names=["Time", "Velocity"])

    # Add a time column (assuming time is the first column already)
    data['Time'] = data.index * 0.01  # Assuming a time step of 0.01 seconds

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(data['Time'], data['Velocity'], label='Velocity Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity Profile Over Time')
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()