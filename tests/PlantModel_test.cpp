#include <gtest/gtest.h>
#include "../PlantModel/PlantModel.hpp"
#include <fstream>

// Test reset sets position and velocity
TEST(PlantModelTest, ResetSetsState) {
    PlantModel pm; // use defaults
    pm.reset(5.0, 2.0);
    EXPECT_DOUBLE_EQ(pm.get_position(), 5.0);
    EXPECT_DOUBLE_EQ(pm.get_velocity(), 2.0);
}

// Test that update with a positive drive velocity moves the mass forward
TEST(PlantModelTest, UpdateMovesMassTowardsDrive) {
    PlantModel pm;
    pm.reset(0.0, 0.0);

    // Take several small steps with a positive drive velocity
    double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        pm.update(10.0, dt);
    }

    // After driving, mass position and velocity should be positive
    EXPECT_GT(pm.get_position(), 0.0);
    EXPECT_GT(pm.get_velocity(), 0.0);
}

// Test that large Coulomb friction prevents motion when net force is below threshold
TEST(PlantModelTest, CoulombFrictionPreventsSmallForces) {
    // Set spring and damper small but Coulomb friction large
    PlantModel pm(1.0, 0.1, 0.1, 0.0, 5.0);
    pm.reset(0.0, 0.0);

    // Small drive velocity (so drive_position stays near default 1.0)
    pm.update(0.0, 0.1);

    // With the chosen parameters the net force should be clamped by Coulomb friction
    EXPECT_NEAR(pm.get_velocity(), 0.0, 1e-9);
    EXPECT_NEAR(pm.get_position(), 0.0, 1e-9);
}

// Basic unit test saving to a csv for visualization 
TEST(PlantModelTest, BasicSimulationLogging) {
    PlantModel pm(0.1, 100.0, 1.0);
    pm.reset(0.0, 0.0);

    std::ofstream logfile("plant_model_log.csv");
    logfile << "time,drive_position,drive_velocity,mass_position,mass_velocity\n";

    double drive_position = 0.0;
    double dt = 0.01; // s
    int steps = 500;

    // Create a constant acceleration drive velocity
    double drive_accel = 50.0; // mm/s^2
    double drive_velocity = 0.0;

    for (int i = 0; i < steps; ++i) {
        double time = i * dt;
        pm.update(drive_velocity, dt);
        drive_position += drive_velocity * dt;

        // Update the drive velocity with constant acceleration
        drive_velocity += drive_accel * dt;

        logfile << time << "," << drive_position << "," << drive_velocity << ","
                << pm.get_position() << "," << pm.get_velocity() << "\n";
    }

    logfile.close();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
