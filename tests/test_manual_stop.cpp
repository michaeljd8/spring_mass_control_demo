#include "../src/SpringMassControlDemo.hpp"
#include <gtest/gtest.h>

// Test that manual_stop sets the motion state to Manual_Stop
TEST(ManualStopTest, SetsMotionStateToManualStop) {
    SpringMassControlDemo demo;
    
    // Start extending motion first
    demo.start_extend();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Extending);
    
    // Call manual_stop
    demo.manual_stop();
    
    // Verify motion state is Manual_Stop
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
}

// Test that manual_stop sets control velocity to zero
TEST(ManualStopTest, SetsControlVelocityToZero) {
    SpringMassControlDemo demo;
    
    // Start extending and run a few control cycles to build up velocity
    demo.start_extend();
    for (int i = 0; i < 100; i++) {
        demo.velocity_control(0.0, 0.0);
    }
    
    // Verify we have some control velocity
    EXPECT_GT(demo.get_control_velocity(), 0.0);
    
    // Call manual_stop
    demo.manual_stop();
    
    // Verify control velocity is zero
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
}

// Test that manual_stop works during retract motion
TEST(ManualStopTest, StopsDuringRetract) {
    SpringMassControlDemo demo;
    
    // Start retracting
    demo.start_retract();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Retracting);
    
    // Call manual_stop
    demo.manual_stop();
    
    // Verify motion state is Manual_Stop
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
}

// Test that manual_stop can be called from Home state
TEST(ManualStopTest, StopsFromHomeState) {
    SpringMassControlDemo demo;
    
    // Initial state should be Home (after reset_trajectory or construction)
    demo.reset_trajectory();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Home);
    
    // Call manual_stop
    demo.manual_stop();
    
    // Verify motion state is Manual_Stop
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
}

// Test that manual_stop works during Final_Velocity state
TEST(ManualStopTest, StopsDuringFinalVelocity) {
    SpringMassControlDemo demo(10.0, 20.0, 60.0, 2.0, 80.0, 200.0);
    
    // Start extending and run until we reach Final_Velocity state
    demo.start_extend();
    double position = 0.0;
    double velocity = 0.0;
    
    // Run control loop until we reach Final_Velocity or timeout
    for (int i = 0; i < 5000; i++) {
        demo.velocity_control(position, velocity);
        position += demo.get_control_velocity() * 0.001; // Simulate position update
        velocity = demo.get_control_velocity();
        
        if (demo.get_motion_state() == SpringMassControlDemo::MotionState::Final_Velocity) {
            break;
        }
    }
    
    // Verify we reached Final_Velocity state
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Final_Velocity);
    
    // Call manual_stop
    demo.manual_stop();
    
    // Verify motion state is Manual_Stop and velocity is zero
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
}

// Test that calling manual_stop multiple times is safe (idempotent)
TEST(ManualStopTest, MultipleCallsAreSafe) {
    SpringMassControlDemo demo;
    
    demo.start_extend();
    
    // Call manual_stop multiple times
    demo.manual_stop();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
    
    demo.manual_stop();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
    
    demo.manual_stop();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    EXPECT_EQ(demo.get_control_velocity(), 0.0);
}

// Test that motion can be restarted after manual_stop
TEST(ManualStopTest, CanRestartAfterManualStop) {
    SpringMassControlDemo demo;
    
    // Start, stop, then restart
    demo.start_extend();
    demo.manual_stop();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Manual_Stop);
    
    // Restart extending
    demo.start_extend();
    EXPECT_EQ(demo.get_motion_state(), SpringMassControlDemo::MotionState::Extending);
    
    // Run a control cycle and verify motion resumes
    demo.velocity_control(0.0, 0.0);
    EXPECT_GT(demo.get_control_velocity(), 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
