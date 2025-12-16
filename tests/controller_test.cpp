#include "../src/control/controller.hpp"
#include <gtest/gtest.h>

// Test fixture for Controller
class ControllerTest : public ::testing::Test {
protected:
    Controller controller;

    void SetUp() override {
        // Initialize Controller with default parameters
        controller = Controller(5.0, 10.0);
    }
};

// Test default initialization
TEST_F(ControllerTest, DefaultInitialization) {
    EXPECT_DOUBLE_EQ(controller.get_target_velocity(), 5.0);
    EXPECT_DOUBLE_EQ(controller.get_target_distance(), 10.0);
}

// Test setters
TEST_F(ControllerTest, Setters) {
    controller.set_target_velocity(7.5);
    controller.set_target_distance(15.0);

    EXPECT_DOUBLE_EQ(controller.get_target_velocity(), 7.5);
    EXPECT_DOUBLE_EQ(controller.get_target_distance(), 15.0);
}

// Test compute_control_signal
TEST_F(ControllerTest, ComputeControlSignal) {
    double current_velocity = 3.0;
    double current_position = 4.0;

    double control_signal = controller.compute_control_signal(current_velocity, current_position);

    // Expected control signal based on placeholder logic
    double expected_signal = 0.1 * (5.0 - 3.0) + 0.01 * (10.0 - 4.0);
    EXPECT_DOUBLE_EQ(control_signal, expected_signal);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}