#include "../PlantModel/PlantModel.hpp"
#include <gtest/gtest.h>
#include <fstream> // For file output
#include <vector>

// Test fixture for PlantModel
class PlantModelTest : public ::testing::Test {
protected:
    PlantModel plant;

    void SetUp() override {
        // Initialize PlantModel with default parameters
        plant = PlantModel();
    }

    // Helper function to save simulation data to CSV
    void save_to_csv(const std::string& filename, const std::vector<double>& time,
                     const std::vector<double>& drive_velocity,
                     const std::vector<double>& position, const std::vector<double>& velocity) {
        std::ofstream file(filename);
        file << "time,drive_velocity,mass_position,mass_velocity\n";
        for (size_t i = 0; i < time.size(); ++i) {
            file << time[i] << "," << drive_velocity[i] << "," << position[i] << "," << velocity[i] << "\n";
        }
        file.close();
    }
};

// Test default initialization
TEST_F(PlantModelTest, DefaultInitialization) {
    EXPECT_DOUBLE_EQ(plant.get_position(), 0.0);
    EXPECT_DOUBLE_EQ(plant.get_velocity(), 0.0);
}

// Test reset functionality
TEST_F(PlantModelTest, ResetState) {
    plant.reset(5.0, -2.0);
    EXPECT_DOUBLE_EQ(plant.get_position(), 5.0);
    EXPECT_DOUBLE_EQ(plant.get_velocity(), -2.0);
}


// Test update with Euler integration
TEST_F(PlantModelTest, UpdateEulerIntegration) {
    double x_in_dot = 0.0;
    double dt = 0.1;

    plant.reset(0.0, 0.0);
    plant.update(x_in_dot, dt);

}

// Test friction effects
TEST_F(PlantModelTest, FrictionEffects) {
    double drive_velocity = 0.0;
    double dt = 0.1;

    plant.reset(1.0, -1.0);
    plant.update(drive_velocity, dt);

    EXPECT_LT(plant.get_velocity(), -0.9); // Velocity should decrease due to friction
}

// Test to save simulation data (time, position, velocity) to a CSV file for plotting
TEST_F(PlantModelTest, SaveSimulationData) {
    double constant_velocity = 1.0;
    double dt = 0.01;
    int steps = 1000;

    plant.reset(0.0, 0.0);

    // Compute velocity profile
    std::vector<double> drive_velocity(steps, constant_velocity);

    std::vector<double> time;
    std::vector<double> position;
    std::vector<double> velocity;

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        time.push_back(t);
        position.push_back(plant.get_position());
        velocity.push_back(plant.get_velocity());
        plant.update(constant_velocity, dt);
    }

    save_to_csv("build/simulation_data.csv", time, drive_velocity, position, velocity);
    SUCCEED(); // Ensure the test passes
}

// Test acceleration to a constant velocity
TEST_F(PlantModelTest, ConstantVelocity) {
    double dt = 0.01;
    int steps = 1000;

    plant.reset(0.0, 0.0);

    // Compute velocity profile
    double constant_velocity = 1.0; // Constant velocity input
    double acceleration_time = 5.0; // Time to reach constant velocity
    double acceleration = constant_velocity / acceleration_time;
    
    std::vector<double> drive_velocity;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        if (t < acceleration_time) {
            drive_velocity.push_back(acceleration * t); // Ramp-up phase
        } else {
            drive_velocity.push_back(constant_velocity); // Constant velocity phase
        }
    }

    std::vector<double> time;
    std::vector<double> position;
    std::vector<double> velocity;

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        time.push_back(t);
        position.push_back(plant.get_position());
        velocity.push_back(plant.get_velocity());
        plant.update(drive_velocity[i], dt);
    }

    // Save the data for visualization
    save_to_csv("build/velocity_data.csv", time, drive_velocity, position, velocity);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}