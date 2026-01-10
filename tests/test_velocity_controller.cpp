#include "../src/SpringMassControlDemo.hpp"
#include <gtest/gtest.h>
#include <cmath>

class VelocityControllerTest : public ::testing::Test {
protected:
    SpringMassControlDemo controller;
    
    void SetUp() override {
        // Reset PID state before each test
        controller.reset_pid();
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

// Test proportional control only
TEST_F(VelocityControllerTest, ProportionalControlOnly) {
    // Set only proportional gain
    controller.set_pid_gains(1.0, 0.0, 0.0);
    
    // At a position where desired velocity is known (e.g., final velocity region)
    double final_vel = controller.get_final_velocity();
    double approach_dist = controller.get_approach_distance();
    
    // Position beyond approach distance should use final velocity
    double mass_position = approach_dist + 5.0;
    double mass_velocity = final_vel - 5.0; // 5 mm/s below desired
    
    controller.velocity_control(0.0, mass_position, mass_velocity);
    
    // With Kp=1, error=5, P_term=5
    // Control velocity = desired + P_term = final_vel + 5
    double expected = final_vel + 5.0;
    EXPECT_NEAR(controller.get_control_velocity(), expected, 0.1);
}

// Test that control velocity is clamped to MAX_VELOCITY
TEST_F(VelocityControllerTest, ControlVelocityClampedToMax) {
    controller.set_pid_gains(10.0, 0.0, 0.0); // High Kp to force high output
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double mass_velocity = 0.0; // Large error will result in high output
    
    controller.velocity_control(0.0, mass_position, mass_velocity);
    
    // Control velocity should be clamped to MAX_VELOCITY
    EXPECT_LE(controller.get_control_velocity(), controller.get_max_velocity());
}

// Test that control velocity is clamped to minimum (0)
TEST_F(VelocityControllerTest, ControlVelocityClampedToMin) {
    controller.set_pid_gains(10.0, 0.0, 0.0);
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double mass_velocity = 200.0; // Much higher than desired, negative error
    
    controller.velocity_control(0.0, mass_position, mass_velocity);
    
    // Control velocity should be clamped to 0 (not negative)
    EXPECT_GE(controller.get_control_velocity(), 0.0);
}

// Test integral accumulation
TEST_F(VelocityControllerTest, IntegralAccumulation) {
    controller.set_pid_gains(0.0, 1.0, 0.0); // Only integral gain
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double final_vel = controller.get_final_velocity();
    double mass_velocity = final_vel - 10.0; // Constant error of 10
    
    // Call velocity_control multiple times to accumulate integral
    double prev_control_vel = 0.0;
    for (int i = 0; i < 100; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
        double current_control_vel = controller.get_control_velocity();
        
        // Control velocity should increase due to integral accumulation
        // (until it hits anti-windup limit or MAX_VELOCITY)
        if (i > 0) {
            EXPECT_GE(current_control_vel, prev_control_vel - 0.01); // Allow small tolerance
        }
        prev_control_vel = current_control_vel;
    }
}

// Test PID reset clears integral and derivative state
TEST_F(VelocityControllerTest, ResetPIDClearsState) {
    controller.set_pid_gains(0.0, 1.0, 0.0);
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double final_vel = controller.get_final_velocity();
    double mass_velocity = final_vel - 10.0;
    
    // Accumulate some integral error
    for (int i = 0; i < 50; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
    }
    double vel_before_reset = controller.get_control_velocity();
    
    // Reset PID state
    controller.reset_pid();
    
    // After reset, first call should have no integral contribution
    controller.velocity_control(0.0, mass_position, mass_velocity);
    double vel_after_reset = controller.get_control_velocity();
    
    // The velocity after reset should be less than before (less integral contribution)
    EXPECT_LT(vel_after_reset, vel_before_reset);
}

// Test derivative action
TEST_F(VelocityControllerTest, DerivativeAction) {
    controller.set_pid_gains(0.0, 0.0, 1.0); // Only derivative gain
    controller.reset_pid();
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double final_vel = controller.get_final_velocity();
    
    // First call with error = 10
    controller.velocity_control(0.0, mass_position, final_vel - 10.0);
    double control1 = controller.get_control_velocity();
    
    // Second call with error = 5 (error decreasing)
    controller.velocity_control(0.0, mass_position, final_vel - 5.0);
    double control2 = controller.get_control_velocity();
    
    // Third call with error = 15 (error increasing)
    controller.velocity_control(0.0, mass_position, final_vel - 15.0);
    double control3 = controller.get_control_velocity();
    
    // When error decreases, derivative term is negative, so control should be lower
    // When error increases, derivative term is positive, so control should be higher
    // This tests that derivative action responds to rate of change
    EXPECT_NE(control1, control2); // Should be different due to derivative
    EXPECT_NE(control2, control3);
}

// Test velocity profile lookup - position before profile start
TEST_F(VelocityControllerTest, VelocityProfileLookupBeforeStart) {
    controller.set_pid_gains(0.0, 0.0, 0.0); // No PID contribution
    
    auto& profile = controller.get_velocity_profile();
    ASSERT_FALSE(profile.empty());
    
    double first_profile_vel = profile.front().second;
    
    // Position before profile starts (negative position)
    controller.velocity_control(0.0, -1.0, first_profile_vel);
    
    // Control velocity should equal the first profile velocity
    EXPECT_NEAR(controller.get_control_velocity(), first_profile_vel, 0.01);
}

// Test velocity profile lookup - position beyond profile end
TEST_F(VelocityControllerTest, VelocityProfileLookupBeyondEnd) {
    controller.set_pid_gains(0.0, 0.0, 0.0); // No PID contribution
    
    auto& profile = controller.get_velocity_profile();
    ASSERT_FALSE(profile.empty());
    
    double final_vel = controller.get_final_velocity();
    double beyond_position = profile.back().first + 10.0;
    
    // Position beyond profile should use final velocity
    controller.velocity_control(0.0, beyond_position, final_vel);
    
    EXPECT_NEAR(controller.get_control_velocity(), final_vel, 0.01);
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

// Test full PID response to steady-state error
TEST_F(VelocityControllerTest, FullPIDSteadyStateTracking) {
    controller.set_pid_gains(1.0, 0.5, 0.1);
    controller.reset_pid();
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double final_vel = controller.get_final_velocity();
    
    // Simulate tracking with decreasing error
    double mass_velocity = final_vel - 20.0;
    std::vector<double> control_outputs;
    
    for (int i = 0; i < 500; ++i) {
        controller.velocity_control(0.0, mass_position, mass_velocity);
        control_outputs.push_back(controller.get_control_velocity());
        
        // Simulate system response - velocity approaches control velocity
        mass_velocity += (controller.get_control_velocity() - mass_velocity) * 0.02;
    }
    
    // After many iterations, mass velocity should approach the desired velocity
    // Use a tolerance of 3 mm/s to account for the simple simulation model
    EXPECT_NEAR(mass_velocity, final_vel, 3.0);
}

// Test with zero gains (no control action)
TEST_F(VelocityControllerTest, ZeroGainsNoControlAction) {
    controller.set_pid_gains(0.0, 0.0, 0.0);
    
    double mass_position = controller.get_approach_distance() + 5.0;
    double final_vel = controller.get_final_velocity();
    double mass_velocity = final_vel - 20.0; // Large error
    
    controller.velocity_control(0.0, mass_position, mass_velocity);
    
    // With zero gains, control velocity should equal desired velocity (no PID correction)
    EXPECT_NEAR(controller.get_control_velocity(), final_vel, 0.01);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
