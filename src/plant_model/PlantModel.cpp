#include "PlantModel.h"

PlantModel::PlantModel() : mass(1.0), springConstant(1.0), cartVelocity(0.0), cartPosition(0.0) {}

void PlantModel::setMass(double m) {
    mass = m;
}

void PlantModel::setSpringConstant(double k) {
    springConstant = k;
}

double PlantModel::getCartVelocity() const {
    return cartVelocity;
}

double PlantModel::getCartPosition() const {
    return cartPosition;
}

void PlantModel::update(double controlVelocity, double timeStep) {
    // Calculate spring force
    double springForce = -springConstant * cartPosition;

    // Calculate acceleration (F = ma)
    double acceleration = springForce / mass;

    // Update velocity and position using discrete integration
    cartVelocity += acceleration * timeStep;
    cartPosition += cartVelocity * timeStep;
}