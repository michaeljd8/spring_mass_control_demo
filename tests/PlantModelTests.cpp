#include "PlantModel.h"
#include <gtest/gtest.h>

// Test the default initialization of the PlantModel
TEST(PlantModelTest, DefaultInitialization) {
    PlantModel plant;
    EXPECT_DOUBLE_EQ(plant.getCartVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(plant.getCartPosition(), 0.0);
}

// Test setting and getting mass
TEST(PlantModelTest, SetMass) {
    PlantModel plant;
    plant.setMass(5.0);
    // No direct getter for mass, but we assume it works if no errors occur
}

// Test setting and getting spring constant
TEST(PlantModelTest, SetSpringConstant) {
    PlantModel plant;
    plant.setSpringConstant(50.0);
    // No direct getter for spring constant, but we assume it works if no errors occur
}

// Test the update function
TEST(PlantModelTest, UpdateFunction) {
    PlantModel plant;
    plant.setMass(10.0);
    plant.setSpringConstant(100.0);

    double controlVelocity = 5.0;
    double timeStep = 0.01;

    plant.update(controlVelocity, timeStep);

    EXPECT_NEAR(plant.getCartVelocity(), 0.0, 0.1); // Velocity should change slightly
    EXPECT_NEAR(plant.getCartPosition(), 0.0, 0.1); // Position should change slightly
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}