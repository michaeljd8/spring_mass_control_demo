// Model in Loop Level 2 for SpringMassControlDemo.hpp

#include "../src/SpringMassControlDemo.hpp"
#include "../PlantModel/PlantModel.hpp"
#include <iostream>

// Derived class for simulation that overrides read_mass_position
class MILController : public SpringMassControlDemo {
public:
    MILController(PlantModel& plant) : plant_(plant) {}
protected:
    double read_mass_position() override {
        return plant_.get_position();
    }
private:
    PlantModel& plant_;
};