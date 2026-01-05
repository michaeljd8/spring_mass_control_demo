#include <gtest/gtest.h>
#include "../src/SpringMassControlDemo.hpp"
#include <fstream>

TEST(SpringMassControlDemoTest, VelocityProfileGeneration) {
    // Create an instance of SpringMassControlDemo with default parameters
    SpringMassControlDemo demo;

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