#include "../src/SpringMassControlDemo.hpp"
#include <cassert>
#include <iostream>
#include <gtest/gtest.h>
#include <vector>

void test_getters_and_setters() {
    SpringMassControlDemo demo;

    // Test final_velocity
    demo.set_final_velocity(50.0);
    assert(demo.get_final_velocity() == 50.0);
    demo.set_final_velocity(5.0); // Below MIN_VELOCITY
    assert(demo.get_final_velocity() == 10.0);
    demo.set_final_velocity(150.0); // Above MAX_VELOCITY
    assert(demo.get_final_velocity() == 100.0);

    // Test target_distance
    demo.set_target_distance(30.0);
    assert(demo.get_target_distance() == 30.0);
    demo.set_target_distance(2.0); // Below MIN_DISTANCE
    assert(demo.get_target_distance() == 5.0);
    demo.set_target_distance(150.0); // Above MAX_DISTANCE
    assert(demo.get_target_distance() == 100.0);

    // Test final_distance
    demo.set_final_distance(40.0);
    assert(demo.get_final_distance() == 40.0);
    demo.set_final_distance(2.0); // Below MIN_DISTANCE
    assert(demo.get_final_distance() == 5.0);
    demo.set_final_distance(150.0); // Above MAX_DISTANCE
    assert(demo.get_final_distance() == 100.0);

    // Test approach_offset
    demo.set_approach_offset(5.0);
    assert(demo.get_approach_offset() == 5.0);
    demo.set_approach_offset(-1.0); // Below 0
    assert(demo.get_approach_offset() == 0.0);
    demo.set_approach_offset(200.0); // Above target_distance
    assert(demo.get_approach_offset() == demo.get_target_distance());

    // Test travel_velocity
    demo.set_travel_velocity(70.0);
    assert(demo.get_travel_velocity() == 70.0);
    demo.set_travel_velocity(5.0); // Below MIN_VELOCITY
    assert(demo.get_travel_velocity() == 10.0);
    demo.set_travel_velocity(150.0); // Above MAX_VELOCITY
    assert(demo.get_travel_velocity() == 100.0);

    // Test acceleration
    demo.set_acceleration(300.0);
    assert(demo.get_acceleration() == 300.0);
    demo.set_acceleration(20.0); // Below MIN_ACCELERATION
    assert(demo.get_acceleration() == 50.0);
    demo.set_acceleration(600.0); // Above MAX_ACCELERATION
    assert(demo.get_acceleration() == 500.0);

    std::cout << "All getter and setter tests passed!" << std::endl;
}



int main() {
    test_getters_and_setters();
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}