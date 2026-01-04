#include "../src/SpringMassControlDemo.hpp"
#include <iostream>
#include <cassert>
#include <fstream>

void test_velocity_profile() {
    // Create an object of SpringMassControlDemo with custom parameters
    SpringMassControlDemo demo(20.0, 100.0, 120.0, 10.0, 50.0, 300.0, 300.0);

    // Retrieve the velocity profile
    const std::vector<double>& velocity_profile = demo.get_velocity_profile();

        // Retrieve the SAMPLING_TIME constant
    double sampling_time = SpringMassControlDemo::get_sampling_time();

    // Save the velocity profile to a CSV file
    std::ofstream csv_file("velocity_profile.csv");
    if (csv_file.is_open()) {
        csv_file << "Time,Velocity\n";
        double time = 0.0;
        for (double velocity : velocity_profile) {
            csv_file << time << "," << velocity << "\n";
            time += sampling_time;
        }
        csv_file.close();
        std::cout << "Velocity profile saved to velocity_profile.csv.\n";
    } else {
        std::cerr << "Failed to open velocity_profile.csv for writing.\n";
    }

    // Check that the velocity profile is not empty
    assert(!velocity_profile.empty());
    std::cout << "Velocity profile is not empty.\n";

    // Check that the velocity profile starts at 0
    assert(velocity_profile.front() == 0.0);
    std::cout << "Velocity profile starts at 0.\n";

    // Check that the velocity profile ends at the final velocity
    assert(velocity_profile.back() == demo.get_final_velocity());
    std::cout << "Velocity profile ends at the final velocity.\n";





    // Print the velocity profile for debugging
    std::cout << "Velocity Profile: ";
    for (double velocity : velocity_profile) {
        std::cout << velocity << " ";
    }
    std::cout << std::endl;
}

int main() {
    test_velocity_profile();
    std::cout << "All test cases passed!\n";
    return 0;
}