#include <gtest/gtest.h>
#include "../src/SpringMassControlDemo.hpp"
#include <fstream>

TEST(SpringMassControlDemoTest, VelocityProfileGeneration) {
    // Initialize parameters for the test
    double final_velocity = 2.0;
    double approach_distance = 10.0;
    double final_distance = 1.0;
    double approach_offset = 1.0;
    double travel_velocity = 5.0;
    double acceleration = 2.0;

    // Create an instance of SpringMassControlDemo
    SpringMassControlDemo demo(final_velocity, approach_distance, final_distance, approach_offset, travel_velocity, acceleration);

    // Call the method to generate the velocity profile
    demo.create_velocity_profile();

    // Helper function to save the velocity profile to a file (for debugging purposes)
    auto save_velocity_profile_to_file = [](const std::vector<double>& profile, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            for (double velocity : profile) {
                file << velocity << "\n";
            }
            file.close();
        }
    };



    // Retrieve the generated velocity profile
    const std::vector<double>& velocity_profile = demo.get_velocity_profile();

    save_velocity_profile_to_file(velocity_profile, "velocity_profile.csv");

    // Verify the velocity profile (example check: size and values)
    ASSERT_FALSE(velocity_profile.empty()) << "Velocity profile should not be empty.";

    for (double velocity : velocity_profile) {
        ASSERT_NEAR(velocity, travel_velocity, 0.01) << "Velocity profile values should match the travel velocity.";
    }
}