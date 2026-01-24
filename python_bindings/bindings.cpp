/*
 * pybind11 bindings for SpringMassControlDemo simulation
 * Exposes MILController class to Python for GUI interaction
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "SpringMassControlDemo.hpp"
#include "PlantModel.hpp"

namespace py = pybind11;

// Derived class for simulation that owns the PlantModel
class MILController : public SpringMassControlDemo {
public:
    // Default constructor with preset plant parameters
    MILController() : plant_(0.1, 100.0, 1.0) {
        last_position_ = plant_.get_position();
    }

    // Constructor with custom plant parameters
    MILController(double mass, double spring_constant, double damping)
        : plant_(mass, spring_constant, damping) {
        last_position_ = plant_.get_position();
    }
    
    double read_mass_position() override {
        double current_position = plant_.get_position();
        double dt = get_sampling_time();
        if (dt > 0) {
            calculated_velocity_ = (current_position - last_position_) / dt;
        }
        last_position_ = current_position;
        return current_position;
    }

    double read_mass_velocity() override {
        return plant_.get_velocity();
    }

    void set_motor_velocity(double drive_velocity, int8_t direction) override {
        plant_.update(drive_velocity, get_sampling_time());
    }

    void reset_sensor_state() {
        last_position_ = plant_.get_position();
        calculated_velocity_ = 0.0;
        plant_.reset();
    }

    double get_current_mass_velocity() const {
        return plant_.get_velocity();
    }

    // Get all current data as a dict-friendly struct for GUI
    py::dict get_state_data() const {
        py::dict data;
        data["time"] = time_s_;
        data["drive_position"] = get_drive_position();
        data["drive_velocity"] = get_control_velocity();
        data["desired_velocity"] = get_desired_velocity();
        data["mass_position"] = get_mass_position();
        data["mass_velocity"] = plant_.get_velocity();
        data["motion_state"] = static_cast<int>(get_motion_state());
        data["motion_state_name"] = motion_state_to_string(get_motion_state());
        return data;
    }

    // Run one simulation step and increment time
    void step() {
        update();
        time_s_ += get_sampling_time();
    }

    // Reset everything including time
    void full_reset() {
        reset_trajectory();
        reset_sensor_state();
        time_s_ = 0.0;
    }

    double get_time() const { return time_s_; }

private:
    PlantModel plant_;
    double last_position_ = 0.0;
    double calculated_velocity_ = 0.0;
    double time_s_ = 0.0;

    static const char* motion_state_to_string(MotionState state) {
        switch (state) {
            case MotionState::Home: return "Home";
            case MotionState::Extending: return "Extending";
            case MotionState::Final_Velocity: return "Final_Velocity";
            case MotionState::At_Final_Distance: return "At_Final_Distance";
            case MotionState::Retracting: return "Retracting";
            case MotionState::Manual_Stop: return "Manual_Stop";
            case MotionState::Error: return "Error";
            default: return "Unknown";
        }
    }
};

PYBIND11_MODULE(spring_mass_sim, m) {
    m.doc() = "Spring Mass Control Simulation Module";

    // Expose MotionState enum
    py::enum_<SpringMassControlDemo::MotionState>(m, "MotionState")
        .value("Home", SpringMassControlDemo::MotionState::Home)
        .value("Extending", SpringMassControlDemo::MotionState::Extending)
        .value("Final_Velocity", SpringMassControlDemo::MotionState::Final_Velocity)
        .value("At_Final_Distance", SpringMassControlDemo::MotionState::At_Final_Distance)
        .value("Retracting", SpringMassControlDemo::MotionState::Retracting)
        .value("Manual_Stop", SpringMassControlDemo::MotionState::Manual_Stop)
        .value("Error", SpringMassControlDemo::MotionState::Error)
        .export_values();

    // Expose MILController class
    py::class_<MILController>(m, "MILController")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("mass"), py::arg("spring_constant"), py::arg("damping"))
        
        // Commands
        .def("start_extend", &MILController::start_extend)
        .def("start_retract", &MILController::start_retract)
        .def("manual_stop", &MILController::manual_stop)
        .def("reset_trajectory", &MILController::reset_trajectory)
        .def("full_reset", &MILController::full_reset)
        .def("step", &MILController::step)
        .def("update", &MILController::update)
        
        // Setters for motion parameters
        .def("set_final_velocity", &MILController::set_final_velocity)
        .def("set_approach_distance", &MILController::set_approach_distance)
        .def("set_final_distance", &MILController::set_final_distance)
        .def("set_approach_offset", &MILController::set_approach_offset)
        .def("set_travel_velocity", &MILController::set_travel_velocity)
        .def("set_acceleration", &MILController::set_acceleration)
        .def("set_pid_gains", &MILController::set_pid_gains)
        
        // Getters for motion parameters
        .def("get_final_velocity", &MILController::get_final_velocity)
        .def("get_approach_distance", &MILController::get_approach_distance)
        .def("get_final_distance", &MILController::get_final_distance)
        .def("get_approach_offset", &MILController::get_approach_offset)
        .def("get_travel_velocity", &MILController::get_travel_velocity)
        .def("get_acceleration", &MILController::get_acceleration)
        
        // Getters for state
        .def("get_motion_state", &MILController::get_motion_state)
        .def("get_state_data", &MILController::get_state_data)
        .def("get_time", &MILController::get_time)
        .def("get_drive_position", &MILController::get_drive_position)
        .def("get_control_velocity", &MILController::get_control_velocity)
        .def("get_desired_velocity", &MILController::get_desired_velocity)
        .def("get_mass_position", &MILController::get_mass_position)
        .def("get_current_mass_velocity", &MILController::get_current_mass_velocity)
        .def("get_sampling_time", &MILController::get_sampling_time)
        
        // Static getters for limits
        .def_static("get_max_velocity", &MILController::get_max_velocity)
        .def_static("get_min_velocity", &MILController::get_min_velocity)
        .def_static("get_max_distance", &MILController::get_max_distance)
        .def_static("get_min_distance", &MILController::get_min_distance);
}
