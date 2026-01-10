#include <gtest/gtest.h>
#include "../src/SpringMassControlDemo.hpp"
#include <fstream>

TEST(SpringMassControlDemoTest, VelocityProfileGeneration) {
    // Create an instance of SpringMassControlDemo with custom parameters
    double final_velocity = 15.0; // mm/s
    double approach_distance = 70.0; // mm
    double final_distance = 80.0; // mm
    double approach_offset = 15.0; // mm
    double travel_velocity = 100.0; // mm/s
    double acceleration = 250.0; // mm/s^2

    SpringMassControlDemo demo(final_velocity, approach_distance, final_distance, approach_offset, travel_velocity, acceleration);

    // Call the method to generate the velocity profile
    demo.create_velocity_profile();

    // Helper function to save the velocity profile to a file (for debugging purposes)
    auto save_velocity_profile_to_file = [](const std::vector<std::pair<double, double>>& profile, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            for (const auto& point : profile) {
                file << point.first << "," << point.second << "\n"; // Use .first and .second for std::pair
            }
            file.close();
        }
    };

    // Retrieve the generated velocity profile
    auto& velocity_profile = demo.get_velocity_profile();

    save_velocity_profile_to_file(velocity_profile, "velocity_profile.csv");

}