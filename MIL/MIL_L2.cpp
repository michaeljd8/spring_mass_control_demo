// Model in Loop Level 2 for SpringMassControlDemo.hpp

#include "SpringMassControlDemo.hpp"
#include "PlantModel.hpp"
#include "save_to_csv.hpp"

using MotionState = SpringMassControlDemo::MotionState;

// Derived class for simulation that overrides read_mass_position
class MILController : public SpringMassControlDemo {
public:
    MILController(PlantModel& plant) : plant_(plant) {}
    
    double read_mass_position() override {
        double current_position = plant_.get_position();
        
        // Calculate velocity from position difference
        double dt = get_sampling_time();
        mass_velocity_ = (current_position - last_position_) / dt;
        last_position_ = current_position;
        
        return current_position;
    }

    // Getter for mass velocity - placed in derived class so filtering can be applied in HAL
    double read_mass_velocity() {
        return mass_velocity_;
    }
    
private:
    PlantModel& plant_;
    double last_position_ = 0.0;
    double mass_velocity_ = 0.0;
};

// Log data structure using vectors for CSV export
struct LogData {
    std::vector<double> time;
    std::vector<double> drive_position;
    std::vector<double> drive_velocity;
    std::vector<double> mass_position;
    std::vector<double> mass_velocity;
    
    void clear() {
        time.clear();
        drive_position.clear();
        drive_velocity.clear();
        mass_position.clear();
        mass_velocity.clear();
    }
    
    void add(double t, double dp, double dv, double mp, double mv) {
        time.push_back(t);
        drive_position.push_back(dp);
        drive_velocity.push_back(dv);
        mass_position.push_back(mp);
        mass_velocity.push_back(mv);
    }
    
    bool save(const std::string& filename) const {
        return save_columns_to_csv(
            {time, drive_position, drive_velocity, mass_position, mass_velocity},
            filename,
            "time,drive_position,drive_velocity,mass_position,mass_velocity"
        );
    }
};

int main () {
    // Create PlantModel with light mass and stiff spring for responsive dynamics
    PlantModel plant(0.1, 100.0, 1.0);

    double final_velocity = 15.0; // mm/s
    double approach_distance = 78.0; // mm
    double final_distance = 80.0; // mm
    double approach_offset = 5.0; // mm
    double travel_velocity = 100.0; // mm/s
    double acceleration = 250.0; // mm/s^2

    // Create MILController with custom parameters
    MILController controller(plant);
    controller.set_final_velocity(final_velocity);
    controller.set_approach_distance(approach_distance);
    controller.set_final_distance(final_distance);
    controller.set_approach_offset(approach_offset);
    controller.set_travel_velocity(travel_velocity);
    controller.set_acceleration(acceleration);

    // Set max time steps to avoid infinite loops
    const int max_time = 20000; // ms

    // Loop through 2 cycles: extend + retract
    for (int cycle = 0; cycle < 2; ++cycle) {
        int time = 0;
        double dt = controller.get_sampling_time();
        double mass_position = 0.0;
        double mass_velocity = 0.0;

        // Start extend phase
        controller.start_extend();

        // Extend loop
        while (controller.get_motion_state() != MotionState::At_Final_Distance && time < max_time) {

        }
    
    

    return 0;
}