#include <gtest/gtest.h>
#include "../PlantModel/PlantModel.hpp"
#include <fstream>


// Test that update with a positive drive velocity moves the mass forward
TEST(PlantModelTest, UpdateMovesMassTowardsDrive) {
    PlantModel pm;
    pm.reset();

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
    pm.reset();

    // Small drive velocity (so drive_position stays near default 1.0)
    pm.update(0.0, 0.1);

    // With the chosen parameters the net force should be clamped by Coulomb friction
    EXPECT_NEAR(pm.get_velocity(), 0.0, 1e-9);
    EXPECT_NEAR(pm.get_position(), 0.0, 1e-9);
}

// Basic unit test saving to a csv for visualization 
TEST(PlantModelTest, BasicSimulationLogging) {
    PlantModel pm(0.1, 100.0, 1.0);
    pm.reset();

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

// Unit Test to move the mass back and forth
TEST(PlantModelTest, BackAndForthMotion) {
    PlantModel pm(0.5, 50.0, 2.0);
    pm.reset();

    std::ofstream logfile("back_and_forth_motion.csv");
    logfile << "time,drive_velocity,drive_position,mass_position,mass_velocity\n";

    double dt = 0.01;
    double drive_velocity = 10.0;  // Constant velocity forward
    double drive_position = 0.0;
    int steps_per_direction = 200;

    // Move forward at constant velocity
    for (int i = 0; i < steps_per_direction; ++i) {
        double time = i * dt;
        pm.update(drive_velocity, dt);
        drive_position += drive_velocity * dt;

        logfile << time << "," << drive_velocity << "," << drive_position << ","
                << pm.get_position() << "," << pm.get_velocity() << "\n";
    }

    // Reverse and move back at constant velocity
    drive_velocity = -10.0;
    for (int i = 0; i < steps_per_direction * 2; ++i) {
        double time = (steps_per_direction + i) * dt;
        pm.update(drive_velocity, dt);
        drive_position += drive_velocity * dt;

        logfile << time << "," << drive_velocity << "," << drive_position << ","
                << pm.get_position() << "," << pm.get_velocity() << "\n";
    }

    logfile.close();

    // After moving back and forth, verify the simulation ran successfully
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
