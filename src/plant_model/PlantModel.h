#ifndef PLANT_MODEL_H
#define PLANT_MODEL_H

class PlantModel {
private:
    double mass; // Mass of the cart
    double springConstant; // Spring constant
    double cartVelocity; // Current velocity of the cart
    double cartPosition; // Current position of the cart

public:
    PlantModel();

    // Setters for parameters
    void setMass(double m);
    void setSpringConstant(double k);

    // Getters for outputs
    double getCartVelocity() const;
    double getCartPosition() const;

    // Update the plant model with control input and time step
    void update(double controlVelocity, double timeStep);
};

#endif // PLANT_MODEL_H