#include "../src/plant_model/plant_model.hpp"
#include <gtest/gtest.h>
#include <fstream> // For file output
#include <vector>

// Test fixture for PlantModel
class PlantModelTest : public ::testing::Test {
protected:
    PlantModel plant;

    void SetUp() override {
        // Initialize PlantModel with default parameters
        plant = PlantModel(1.0, 10.0, 0.5, 0.1, 0.5);
    }

    // Helper function to save simulation data to CSV
    void save_to_csv(const std::string& filename, const std::vector<double>& time,
                     const std::vector<double>& position, const std::vector<double>& velocity) {
        std::ofstream file(filename);
        file << "time,position,velocity\n";
        for (size_t i = 0; i < time.size(); ++i) {
            file << time[i] << "," << position[i] << "," << velocity[i] << "\n";
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

// Test compute_acceleration with no friction
TEST_F(PlantModelTest, ComputeAccelerationNoFriction) {
    plant.viscous_friction = 0.0;
    plant.coulomb_friction = 0.0;

    double x_in = 1.0;
    double x_in_dot = 0.0;
    plant.reset(0.0, 0.0);

    double acceleration = plant.compute_acceleration(x_in, x_in_dot);
    EXPECT_DOUBLE_EQ(acceleration, 10.0); // Spring force only
}

// Test update with Euler integration
TEST_F(PlantModelTest, UpdateEulerIntegration) {
    double x_in = 1.0;
    double x_in_dot = 0.0;
    double dt = 0.1;

    plant.reset(0.0, 0.0);
    plant.update(x_in, x_in_dot, dt);

    EXPECT_NEAR(plant.get_position(), 0.05, 1e-2);
    EXPECT_NEAR(plant.get_velocity(), 1.0, 1e-2);
}

// Test update with Runge-Kutta integration
TEST_F(PlantModelTest, UpdateRungeKuttaIntegration) {
    double x_in = 1.0;
    double x_in_dot = 0.0;
    double dt = 0.1;

    plant.reset(0.0, 0.0);
    plant.update_rk4(x_in, x_in_dot, dt);

    EXPECT_NEAR(plant.get_position(), 0.05, 1e-2);
    EXPECT_NEAR(plant.get_velocity(), 1.0, 1e-2);
}

// Test friction effects
TEST_F(PlantModelTest, FrictionEffects) {
    double x_in = 0.0;
    double x_in_dot = 0.0;
    double dt = 0.1;

    plant.reset(1.0, -1.0);
    plant.update(x_in, x_in_dot, dt);

    EXPECT_LT(plant.get_velocity(), -0.9); // Velocity should decrease due to friction
}

// Test to save simulation data (time, position, velocity) to a CSV file for plotting
TEST_F(PlantModelTest, SaveSimulationData) {
    double x_in = 1.0;
    double x_in_dot = 0.0;
    double dt = 0.01;
    int steps = 1000;

    plant.reset(0.0, 0.0);

    std::vector<double> time;
    std::vector<double> position;
    std::vector<double> velocity;

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        time.push_back(t);
        position.push_back(plant.get_position());
        velocity.push_back(plant.get_velocity());
        plant.update_rk4(x_in, x_in_dot, dt);
    }

    save_to_csv("simulation_data.csv", time, position, velocity);
    SUCCEED(); // Ensure the test passes
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}