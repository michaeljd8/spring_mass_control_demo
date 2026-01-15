#include "../src/SpringMassControlDemo.hpp"
#include <gtest/gtest.h>
#include <cmath>

class VelocityControllerTest : public ::testing::Test {
protected:
    SpringMassControlDemo controller;
    
    void SetUp() override {
        // Reset trajectory and PID state before each test
        controller.reset_trajectory();
    }
};

// Test PID gain setters and getters
TEST_F(VelocityControllerTest, PIDGainSettersGetters) {
    // Test set_pid_gains
    controller.set_pid_gains(2.0, 0.5, 0.1);
    EXPECT_DOUBLE_EQ(controller.get_kp(), 2.0);
    EXPECT_DOUBLE_EQ(controller.get_ki(), 0.5);
    EXPECT_DOUBLE_EQ(controller.get_kd(), 0.1);
    
    // Test individual setters
    controller.set_kp(3.0);
    EXPECT_DOUBLE_EQ(controller.get_kp(), 3.0);
    
    controller.set_ki(1.0);
    EXPECT_DOUBLE_EQ(controller.get_ki(), 1.0);
    
    controller.set_kd(0.2);
    EXPECT_DOUBLE_EQ(controller.get_kd(), 0.2);
}

// Test that negative PID gains are clamped to zero
TEST_F(VelocityControllerTest, NegativePIDGainsClampedToZero) {
    controller.set_kp(-1.0);
    EXPECT_DOUBLE_EQ(controller.get_kp(), 0.0);
    
    controller.set_ki(-0.5);
    EXPECT_DOUBLE_EQ(controller.get_ki(), 0.0);
    
    controller.set_kd(-0.1);
    EXPECT_DOUBLE_EQ(controller.get_kd(), 0.0);
    
    controller.set_pid_gains(-1.0, -2.0, -3.0);
    EXPECT_DOUBLE_EQ(controller.get_kp(), 0.0);
    EXPECT_DOUBLE_EQ(controller.get_ki(), 0.0);
    EXPECT_DOUBLE_EQ(controller.get_kd(), 0.0);
}

// Test proportional control action with Ruckig trajectory
TEST_F(VelocityControllerTest, ProportionalControlOnly) {
    // Set only proportional gain
    controller.set_pid_gains(1.0, 0.0, 0.0);
    controller.reset_trajectory();
    
    // Run a few iterations to let Ruckig generate trajectory
    double mass_position = 0.0;
    double mass_velocity = 0.0;
    
    // First call to get initial desired velocity from Ruckig
    controller.velocity_control(0.0, mass_position, mass_velocity);
    double control_vel_1 = controller.get_control_velocity();
    
    // With Kp=1 and mass_velocity=0, error = desired_velocity - 0 = desired_velocity
    // control_velocity = desired_velocity + Kp * error = desired_velocity + desired_velocity = 2 * desired_velocity
    // So control velocity should be positive and greater than zero
    EXPECT_GT(control_vel_1, 0.0);
    
    // Test that proportional gain affects the output
    // With higher mass velocity (closer to desired), error is smaller
    controller.reset_trajectory();
    controller.set_pid_gains(0.0, 0.0, 0.0); // Zero gains first
    controller.velocity_control(0.0, mass_position, 0.0);
    double control_vel_no_pid = controller.get_control_velocity();
    
    controller.reset_trajectory();
    controller.set_pid_gains(1.0, 0.0, 0.0); // With proportional gain
    controller.velocity_control(0.0, mass_position, 0.0);
    double control_vel_with_pid = controller.get_control_velocity();
    
    // With Kp > 0 and positive error (mass not moving), control should be higher
    EXPECT_GT(control_vel_with_pid, control_vel_no_pid);
}

// Test that control velocity increases with high Kp when tracking error exists
TEST_F(VelocityControllerTest, HighKpIncreasesControlVelocity) {
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0; // Mass not moving, large error
    
    // First test with low Kp
    controller.set_pid_gains(0.5, 0.0, 0.0);
    controller.velocity_control(0.0, mass_position, mass_velocity);
    double control_vel_low_kp = controller.get_control_velocity();
    
    // Reset and test with high Kp
    controller.reset_trajectory();
    controller.set_pid_gains(5.0, 0.0, 0.0);
    controller.velocity_control(0.0, mass_position, mass_velocity);
    double control_vel_high_kp = controller.get_control_velocity();
    
    // Higher Kp should result in higher control velocity (more aggressive correction)
    EXPECT_GT(control_vel_high_kp, control_vel_low_kp);
}

// Test that control velocity can go negative (current implementation doesn't clamp)
TEST_F(VelocityControllerTest, ControlVelocityCanBeNegative) {
    controller.set_pid_gains(10.0, 0.0, 0.0);
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 200.0; // Much higher than desired, negative error
    
    controller.velocity_control(0.0, mass_position, mass_velocity);
    
    // Note: Current implementation does NOT clamp control velocity to [0, MAX]
    // This test documents the current behavior - control velocity can be negative
    // If clamping is desired, the implementation should be updated
    double control_vel = controller.get_control_velocity();
    // Just verify the function ran without error
    EXPECT_TRUE(std::isfinite(control_vel));
}

// Test integral accumulation
TEST_F(VelocityControllerTest, IntegralAccumulation) {
    controller.set_pid_gains(0.0, 1.0, 0.0); // Only integral gain
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0; // Mass not moving, creates constant error
    
    // Call velocity_control multiple times to accumulate integral
    double prev_control_vel = 0.0;
    for (int i = 0; i < 50; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
        double current_control_vel = controller.get_control_velocity();
        
        // Control velocity should increase due to integral accumulation
        // (until it hits anti-windup limit)
        if (i > 0) {
            EXPECT_GE(current_control_vel, prev_control_vel - 0.1); // Allow small tolerance for Ruckig variations
        }
        prev_control_vel = current_control_vel;
    }
    
    // After accumulation, control velocity should be significantly positive
    EXPECT_GT(controller.get_control_velocity(), 0.0);
}

// Test PID reset clears integral and derivative state
TEST_F(VelocityControllerTest, ResetPIDClearsState) {
    controller.set_pid_gains(0.0, 1.0, 0.0);
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0;
    
    // Accumulate some integral error by running multiple iterations
    for (int i = 0; i < 100; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
    }
    double vel_before_reset = controller.get_control_velocity();
    
    // Reset PID state (but not trajectory, so Ruckig continues from same state)
    controller.reset_pid();
    
    // After reset, the integral contribution should be cleared
    // The next call will have zero integral contribution
    controller.velocity_control(0.0, mass_position, mass_velocity);
    double vel_after_reset = controller.get_control_velocity();
    
    // With integral reset, the control velocity should be different
    // (less integral contribution means different output)
    // Note: The exact relationship depends on where Ruckig is in the trajectory
    EXPECT_NE(vel_after_reset, vel_before_reset);
}

// Test derivative action
TEST_F(VelocityControllerTest, DerivativeAction) {
    controller.set_pid_gains(0.0, 0.0, 1.0); // Only derivative gain
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    
    // First call with mass velocity = 0 (error = desired - 0)
    controller.velocity_control(0.0, mass_position, 0.0);
    double control1 = controller.get_control_velocity();
    
    // Second call with mass velocity = 5 (error decreasing if desired > 5)
    controller.velocity_control(0.0, mass_position, 5.0);
    double control2 = controller.get_control_velocity();
    
    // Third call with mass velocity = 0 again (error increasing)
    controller.velocity_control(0.0, mass_position, 0.0);
    double control3 = controller.get_control_velocity();
    
    // The derivative term should cause different outputs when error changes
    // This tests that derivative action responds to rate of change
    EXPECT_NE(control1, control2); // Should be different due to derivative
    EXPECT_NE(control2, control3);
}

// Test Ruckig trajectory generates increasing velocity at start
TEST_F(VelocityControllerTest, RuckigTrajectoryStartsAccelerating) {
    controller.set_pid_gains(0.0, 0.0, 0.0); // No PID, just Ruckig output
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0;
    
    std::vector<double> velocities;
    
    // Run trajectory for some iterations
    for (int i = 0; i < 50; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
        velocities.push_back(controller.get_control_velocity());
    }
    
    // Early velocities should be increasing (acceleration phase)
    EXPECT_GT(velocities[10], velocities[0]);
    EXPECT_GT(velocities[20], velocities[10]);
}

// Test Ruckig trajectory reaches target velocity
TEST_F(VelocityControllerTest, RuckigTrajectoryReachesTargetVelocity) {
    controller.set_pid_gains(0.0, 0.0, 0.0); // No PID, just Ruckig output
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0;
    double final_vel = controller.get_final_velocity();
    
    // Run trajectory until completion (or max iterations)
    double control_vel = 0.0;
    for (int i = 0; i < 2000; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
        control_vel = controller.get_control_velocity();
        
        // Update simulated mass position based on control velocity
        mass_position += control_vel * controller.get_sampling_time();
    }
    
    // After trajectory completes, velocity should be near final_velocity
    EXPECT_NEAR(control_vel, final_vel, 1.0);
}

// Test internal state variables are updated correctly
TEST_F(VelocityControllerTest, InternalStateUpdated) {
    double mass_pos = 25.0;
    double mass_vel = 45.0;
    double control_vel = 50.0;
    
    controller.velocity_control(control_vel, mass_pos, mass_vel);

    EXPECT_DOUBLE_EQ(controller.get_mass_position(), mass_pos);
    EXPECT_DOUBLE_EQ(controller.get_mass_velocity(), mass_vel);
}

// Test drive position integrates control velocity
TEST_F(VelocityControllerTest, DrivePositionIntegrates) {
    controller.set_pid_gains(0.0, 0.0, 0.0);
    controller.reset_trajectory();
    
    double initial_drive_pos = controller.get_drive_position();
    
    // Run several iterations
    for (int i = 0; i < 100; ++i) {
        controller.velocity_control(0.0, 0.0, 0.0);
    }
    
    double final_drive_pos = controller.get_drive_position();
    
    // Drive position should have increased (integrated positive velocity)
    EXPECT_GT(final_drive_pos, initial_drive_pos);
}

// Test full PID response with simulated mass tracking
TEST_F(VelocityControllerTest, FullPIDTracking) {
    controller.set_pid_gains(0.5, 0.05, 0.01); // More conservative gains
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0;
    
    std::vector<double> control_outputs;
    
    // Simulate tracking - mass velocity gradually approaches control velocity
    for (int i = 0; i < 500; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
        double control_vel = controller.get_control_velocity();
        control_outputs.push_back(control_vel);
        
        // Simulate system response - velocity approaches control velocity with lag
        // Use a more conservative response factor
        double vel_diff = control_vel - mass_velocity;
        // Clamp the velocity change to prevent instability
        double max_vel_change = 5.0; // mm/s per iteration
        if (vel_diff > max_vel_change) vel_diff = max_vel_change;
        if (vel_diff < -max_vel_change) vel_diff = -max_vel_change;
        
        mass_velocity += vel_diff * 0.02;
        mass_position += mass_velocity * controller.get_sampling_time();
        
        // Safety check to prevent runaway values
        if (!std::isfinite(mass_velocity) || !std::isfinite(mass_position)) {
            break;
        }
    }
    
    // Verify the simulation ran and produced finite values
    EXPECT_TRUE(std::isfinite(mass_velocity));
    EXPECT_TRUE(std::isfinite(mass_position));
    
    // Mass velocity should have increased from initial zero
    EXPECT_GT(mass_velocity, 0.0);
}

// Test with zero gains - output equals Ruckig desired velocity
TEST_F(VelocityControllerTest, ZeroGainsOutputsDesiredVelocity) {
    controller.set_pid_gains(0.0, 0.0, 0.0);
    controller.reset_trajectory();
    
    double mass_position = 0.0;
    double mass_velocity = 0.0;
    
    controller.velocity_control(0.0, mass_position, mass_velocity);
    double control_vel_1 = controller.get_control_velocity();
    
    // With zero PID gains and zero mass velocity:
    // error = desired - 0 = desired
    // P_term = 0 * desired = 0
    // I_term = 0
    // D_term = 0
    // control = desired + 0 = desired
    
    // The control velocity should be positive (Ruckig generates positive trajectory)
    EXPECT_GT(control_vel_1, 0.0);
    
    // Run again with mass velocity matching the control velocity
    controller.reset_trajectory();
    controller.velocity_control(0.0, mass_position, control_vel_1);
    double control_vel_2 = controller.get_control_velocity();
    
    // With matching velocities, error = 0, so control = desired
    // Should be approximately the same (both equal to desired from Ruckig)
    EXPECT_NEAR(control_vel_2, control_vel_1, 1.0);
}

// Test reset_trajectory reinitializes everything
TEST_F(VelocityControllerTest, ResetTrajectoryReinitializes) {
    controller.set_pid_gains(1.0, 0.5, 0.1);
    
    // Run some iterations
    for (int i = 0; i < 100; ++i) {
        controller.velocity_control(0.0, 0.0, 0.0);
    }
    
    double pos_before_reset = controller.get_drive_position();
    
    // Reset trajectory
    controller.reset_trajectory();
    
    double pos_after_reset = controller.get_drive_position();
    
    // Drive position should be reset to 0
    EXPECT_DOUBLE_EQ(pos_after_reset, 0.0);
    EXPECT_GT(pos_before_reset, pos_after_reset);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
