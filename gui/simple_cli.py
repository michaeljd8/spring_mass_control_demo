"""
Spring Mass Control Simulation - Simple CLI
Prints position and velocity at every step for debugging
"""

import sys
import os

# Add build directory to path for the compiled module
build_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'build')
sys.path.insert(0, build_dir)

try:
    import spring_mass_sim as sim
except ImportError as e:
    print(f"Error importing spring_mass_sim module: {e}")
    print(f"Make sure to build the module first:")
    print(f"  cd build && cmake .. && make spring_mass_sim")
    sys.exit(1)


def main():
    print("=== Spring Mass Control Simulation ===\n")
    
    # Create controller
    controller = sim.MILController()
    
    # Configure parameters
    controller.set_final_velocity(15.0)
    controller.set_approach_distance(78.0)
    controller.set_final_distance(80.0)
    controller.set_approach_offset(5.0)
    controller.set_travel_velocity(100.0)
    controller.set_acceleration(250.0)
    controller.set_pid_gains(4.0, 0.0, 0.0)
    
    print(f"Initial state: {controller.get_state_data()['motion_state_name']}")
    print(f"Target: extend to {controller.get_final_distance()}mm, then retract to home\n")
    
    # Start extend
    print("--- STARTING EXTEND ---")
    controller.start_extend()
    
    # Run extend phase - print every 100 steps
    step = 0
    print_interval = 100
    max_steps = 20000
    
    while controller.get_motion_state() != sim.MotionState.At_Final_Distance and step < max_steps:
        controller.step()
        step += 1
        
        if step % print_interval == 0:
            data = controller.get_state_data()
            print(f"step={step:5d} | t={data['time']:.3f}s | "
                  f"mass_pos={data['mass_position']:7.2f}mm | "
                  f"mass_vel={data['mass_velocity']:7.2f}mm/s | "
                  f"state={data['motion_state_name']}")
    
    # Print final extend position
    data = controller.get_state_data()
    print(f"\n--- EXTEND COMPLETE ---")
    print(f"step={step:5d} | t={data['time']:.3f}s | "
          f"mass_pos={data['mass_position']:7.2f}mm | "
          f"state={data['motion_state_name']}\n")
    
    # Dwell for 500 steps
    print("--- DWELLING ---")
    for _ in range(500):
        controller.step()
        step += 1
    
    data = controller.get_state_data()
    print(f"step={step:5d} | t={data['time']:.3f}s | "
          f"mass_pos={data['mass_position']:7.2f}mm | "
          f"mass_vel={data['mass_velocity']:7.2f}mm/s\n")
    
    # Start retract
    print("--- STARTING RETRACT ---")
    controller.start_retract()
    
    # Run retract phase
    while controller.get_motion_state() != sim.MotionState.Home and step < max_steps * 2:
        controller.step()
        step += 1
        
        if step % print_interval == 0:
            data = controller.get_state_data()
            print(f"step={step:5d} | t={data['time']:.3f}s | "
                  f"mass_pos={data['mass_position']:7.2f}mm | "
                  f"mass_vel={data['mass_velocity']:7.2f}mm/s | "
                  f"state={data['motion_state_name']}")
    
    # Print final state
    data = controller.get_state_data()
    print(f"\n--- RETRACT COMPLETE ---")
    print(f"step={step:5d} | t={data['time']:.3f}s | "
          f"mass_pos={data['mass_position']:7.2f}mm | "
          f"state={data['motion_state_name']}")
    
    print(f"\n=== CYCLE COMPLETE ===")
    print(f"Total steps: {step}")
    print(f"Total time: {data['time']:.3f}s")
    print(f"Final position: {data['mass_position']:.4f}mm")


if __name__ == "__main__":
    main()
