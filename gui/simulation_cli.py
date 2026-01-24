"""
Spring Mass Control Simulation CLI
Interactive command-line interface for the C++ simulation backend via pybind11
"""

import sys
import os
import time
import threading
from collections import deque

# Add build directory to path for the compiled module
build_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'build')
sys.path.insert(0, build_dir)

try:
    import spring_mass_sim as sim
except ImportError as e:
    print(f"Error importing spring_mass_sim module: {e}")
    print(f"Make sure to build the module first:")
    print(f"  cd build && cmake .. && make spring_mass_sim")
    print(f"Looking in: {build_dir}")
    sys.exit(1)


# ANSI color codes for terminal output
class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"


STATE_COLORS = {
    "Home": Colors.GREEN,
    "Extending": Colors.BLUE,
    "Final_Velocity": Colors.CYAN,
    "At_Final_Distance": Colors.MAGENTA,
    "Retracting": Colors.YELLOW,
    "Manual_Stop": Colors.RED,
    "Error": Colors.RED + Colors.BOLD,
}


class SimulationCLI:
    def __init__(self):
        # Create controller with default plant parameters
        self.controller = sim.MILController()
        
        # Configure default motion parameters
        self.controller.set_final_velocity(15.0)
        self.controller.set_approach_distance(78.0)
        self.controller.set_final_distance(80.0)
        self.controller.set_approach_offset(5.0)
        self.controller.set_travel_velocity(100.0)
        self.controller.set_acceleration(250.0)
        self.controller.set_pid_gains(4.0, 0.0, 0.0)
        
        # Simulation state
        self.running = False
        self.sim_thread = None
        self.stop_event = threading.Event()
        
        # Data logging
        self.data_log = deque(maxlen=1000)
        self.log_interval = 0.05  # Log every 50ms of sim time
        self.last_log_time = 0.0
        
        # Display update rate
        self.display_interval = 0.1  # Update display every 100ms real time
        
    def print_header(self):
        """Print the CLI header"""
        print(f"\n{Colors.BOLD}{'='*60}")
        print("   Spring Mass Control Simulation - Interactive CLI")
        print(f"{'='*60}{Colors.RESET}\n")
        
    def print_help(self):
        """Print available commands"""
        print(f"\n{Colors.BOLD}Available Commands:{Colors.RESET}")
        print(f"  {Colors.GREEN}e{Colors.RESET} / {Colors.GREEN}extend{Colors.RESET}    - Start extend motion (from Home)")
        print(f"  {Colors.YELLOW}r{Colors.RESET} / {Colors.YELLOW}retract{Colors.RESET}   - Start retract motion (from At_Final_Distance)")
        print(f"  {Colors.RED}s{Colors.RESET} / {Colors.RED}stop{Colors.RESET}      - Emergency stop")
        print(f"  {Colors.CYAN}reset{Colors.RESET}         - Reset to initial state")
        print(f"  {Colors.BLUE}run{Colors.RESET}           - Run simulation continuously")
        print(f"  {Colors.BLUE}pause{Colors.RESET}         - Pause simulation")
        print(f"  {Colors.BLUE}step [n]{Colors.RESET}      - Run n steps (default: 100)")
        print(f"  {Colors.MAGENTA}status{Colors.RESET}        - Show current state")
        print(f"  {Colors.MAGENTA}params{Colors.RESET}        - Show motion parameters")
        print(f"  {Colors.MAGENTA}log{Colors.RESET}           - Show recent data log")
        print(f"  {Colors.WHITE}cycle{Colors.RESET}         - Run full extend/retract cycle")
        print(f"  {Colors.WHITE}help{Colors.RESET}          - Show this help")
        print(f"  {Colors.WHITE}quit{Colors.RESET} / {Colors.WHITE}q{Colors.RESET}     - Exit")
        print()
        
    def get_state_color(self, state_name):
        """Get ANSI color for state"""
        return STATE_COLORS.get(state_name, Colors.WHITE)
        
    def print_status(self):
        """Print current simulation status"""
        data = self.controller.get_state_data()
        state_name = data['motion_state_name']
        color = self.get_state_color(state_name)
        
        print(f"\n{Colors.BOLD}Current Status:{Colors.RESET}")
        print(f"  Motion State:     {color}{state_name}{Colors.RESET}")
        print(f"  Time:             {data['time']:.3f} s")
        print(f"  Drive Position:   {data['drive_position']:.2f} mm")
        print(f"  Drive Velocity:   {data['drive_velocity']:.2f} mm/s")
        print(f"  Desired Velocity: {data['desired_velocity']:.2f} mm/s")
        print(f"  Mass Position:    {data['mass_position']:.2f} mm")
        print(f"  Mass Velocity:    {data['mass_velocity']:.2f} mm/s")
        print(f"  Simulation:       {'RUNNING' if self.running else 'PAUSED'}")
        print()
        
    def print_params(self):
        """Print motion parameters"""
        print(f"\n{Colors.BOLD}Motion Parameters:{Colors.RESET}")
        print(f"  Final Velocity:     {self.controller.get_final_velocity():.1f} mm/s")
        print(f"  Approach Distance:  {self.controller.get_approach_distance():.1f} mm")
        print(f"  Final Distance:     {self.controller.get_final_distance():.1f} mm")
        print(f"  Approach Offset:    {self.controller.get_approach_offset():.1f} mm")
        print(f"  Travel Velocity:    {self.controller.get_travel_velocity():.1f} mm/s")
        print(f"  Acceleration:       {self.controller.get_acceleration():.1f} mm/s²")
        print(f"\n{Colors.BOLD}System Limits:{Colors.RESET}")
        print(f"  Max Velocity:       {sim.MILController.get_max_velocity():.1f} mm/s")
        print(f"  Min Velocity:       {sim.MILController.get_min_velocity():.1f} mm/s")
        print(f"  Max Distance:       {sim.MILController.get_max_distance():.1f} mm")
        print(f"  Min Distance:       {sim.MILController.get_min_distance():.1f} mm")
        print(f"  Sampling Time:      {self.controller.get_sampling_time()*1000:.1f} ms")
        print()
        
    def print_log(self, n=10):
        """Print recent data log entries"""
        if not self.data_log:
            print(f"\n{Colors.YELLOW}No data logged yet. Run the simulation first.{Colors.RESET}\n")
            return
            
        print(f"\n{Colors.BOLD}Recent Data Log (last {min(n, len(self.data_log))} entries):{Colors.RESET}")
        print(f"{'Time':>8}  {'State':<18}  {'Drive Pos':>10}  {'Mass Pos':>10}  {'Mass Vel':>10}")
        print("-" * 65)
        
        entries = list(self.data_log)[-n:]
        for data in entries:
            state_name = data['motion_state_name']
            color = self.get_state_color(state_name)
            print(f"{data['time']:>8.3f}  {color}{state_name:<18}{Colors.RESET}  "
                  f"{data['drive_position']:>10.2f}  {data['mass_position']:>10.2f}  "
                  f"{data['mass_velocity']:>10.2f}")
        print()
        
    def log_data(self):
        """Log current data if interval has passed"""
        data = self.controller.get_state_data()
        if data['time'] - self.last_log_time >= self.log_interval:
            self.data_log.append(data)
            self.last_log_time = data['time']
            
    def run_steps(self, n=100, show_progress=True):
        """Run n simulation steps"""
        start_state = self.controller.get_state_data()['motion_state_name']
        
        for i in range(n):
            self.controller.step()
            self.log_data()
            
        end_state = self.controller.get_state_data()['motion_state_name']
        
        if show_progress:
            data = self.controller.get_state_data()
            print(f"  Ran {n} steps | t={data['time']:.3f}s | "
                  f"pos={data['mass_position']:.2f}mm | "
                  f"state: {self.get_state_color(end_state)}{end_state}{Colors.RESET}")
            
            if start_state != end_state:
                print(f"  {Colors.CYAN}State changed: {start_state} → {end_state}{Colors.RESET}")
                
    def simulation_loop(self):
        """Background simulation loop"""
        last_display = time.time()
        
        while not self.stop_event.is_set():
            # Run simulation steps
            for _ in range(10):  # 10 steps per iteration
                self.controller.step()
                self.log_data()
                
            # Periodic status update
            now = time.time()
            if now - last_display >= self.display_interval:
                data = self.controller.get_state_data()
                state_name = data['motion_state_name']
                color = self.get_state_color(state_name)
                # Print on same line (overwrite)
                print(f"\r  t={data['time']:.3f}s | pos={data['mass_position']:.2f}mm | "
                      f"vel={data['mass_velocity']:.2f}mm/s | "
                      f"state: {color}{state_name:<18}{Colors.RESET}", end="", flush=True)
                last_display = now
                
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
            
    def start_simulation(self):
        """Start background simulation"""
        if self.running:
            print(f"{Colors.YELLOW}Simulation already running{Colors.RESET}")
            return
            
        self.running = True
        self.stop_event.clear()
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_thread.start()
        print(f"{Colors.GREEN}Simulation started{Colors.RESET}")
        
    def pause_simulation(self):
        """Pause background simulation"""
        if not self.running:
            print(f"{Colors.YELLOW}Simulation not running{Colors.RESET}")
            return
            
        self.stop_event.set()
        if self.sim_thread:
            self.sim_thread.join(timeout=1.0)
        self.running = False
        print(f"\n{Colors.YELLOW}Simulation paused{Colors.RESET}")
        
    def run_cycle(self):
        """Run a complete extend/retract cycle"""
        print(f"\n{Colors.BOLD}Running complete cycle...{Colors.RESET}")
        
        # Check starting state
        state = self.controller.get_motion_state()
        if state != sim.MotionState.Home:
            print(f"{Colors.RED}Error: Must be in Home state to run cycle{Colors.RESET}")
            return
            
        # Start extend
        print(f"  Starting extend...")
        self.controller.start_extend()
        
        # Run until At_Final_Distance
        steps = 0
        max_steps = 50000
        while self.controller.get_motion_state() != sim.MotionState.At_Final_Distance and steps < max_steps:
            self.controller.step()
            self.log_data()
            steps += 1
            if steps % 1000 == 0:
                data = self.controller.get_state_data()
                print(f"    t={data['time']:.3f}s | pos={data['mass_position']:.2f}mm | "
                      f"state: {data['motion_state_name']}")
                
        data = self.controller.get_state_data()
        print(f"  {Colors.GREEN}Extend complete{Colors.RESET} at pos={data['mass_position']:.2f}mm")
        
        # Dwell
        print(f"  Dwelling for 500ms...")
        for _ in range(500):
            self.controller.step()
            self.log_data()
            
        # Start retract
        print(f"  Starting retract...")
        self.controller.start_retract()
        
        # Run until Home
        while self.controller.get_motion_state() != sim.MotionState.Home and steps < max_steps:
            self.controller.step()
            self.log_data()
            steps += 1
            if steps % 1000 == 0:
                data = self.controller.get_state_data()
                print(f"    t={data['time']:.3f}s | pos={data['mass_position']:.2f}mm | "
                      f"state: {data['motion_state_name']}")
                
        data = self.controller.get_state_data()
        print(f"  {Colors.GREEN}Retract complete{Colors.RESET} at pos={data['mass_position']:.2f}mm")
        print(f"\n{Colors.BOLD}Cycle complete!{Colors.RESET} Total time: {data['time']:.3f}s, Steps: {steps}")
        print()
        
    def process_command(self, cmd):
        """Process a user command"""
        parts = cmd.strip().lower().split()
        if not parts:
            return True
            
        command = parts[0]
        args = parts[1:]
        
        # Pause simulation for most commands
        was_running = self.running
        if command not in ['status', 'params', 'log', 'help', 'quit', 'q']:
            if self.running:
                self.pause_simulation()
        
        if command in ['e', 'extend']:
            state = self.controller.get_motion_state()
            if state == sim.MotionState.Home:
                self.controller.start_extend()
                print(f"{Colors.GREEN}Started extend motion{Colors.RESET}")
            else:
                print(f"{Colors.RED}Error: Can only extend from Home state (current: {self.controller.get_state_data()['motion_state_name']}){Colors.RESET}")
                
        elif command in ['r', 'retract']:
            state = self.controller.get_motion_state()
            if state == sim.MotionState.At_Final_Distance:
                self.controller.start_retract()
                print(f"{Colors.YELLOW}Started retract motion{Colors.RESET}")
            else:
                print(f"{Colors.RED}Error: Can only retract from At_Final_Distance state (current: {self.controller.get_state_data()['motion_state_name']}){Colors.RESET}")
                
        elif command in ['s', 'stop']:
            self.controller.manual_stop()
            print(f"{Colors.RED}Emergency stop!{Colors.RESET}")
            
        elif command == 'reset':
            self.controller.full_reset()
            self.data_log.clear()
            self.last_log_time = 0.0
            print(f"{Colors.CYAN}Reset to initial state{Colors.RESET}")
            
        elif command == 'run':
            self.start_simulation()
            
        elif command == 'pause':
            self.pause_simulation()
            
        elif command == 'step':
            n = int(args[0]) if args else 100
            self.run_steps(n)
            
        elif command == 'status':
            self.print_status()
            
        elif command == 'params':
            self.print_params()
            
        elif command == 'log':
            n = int(args[0]) if args else 10
            self.print_log(n)
            
        elif command == 'cycle':
            self.run_cycle()
            
        elif command == 'help':
            self.print_help()
            
        elif command in ['quit', 'q', 'exit']:
            if self.running:
                self.pause_simulation()
            print(f"{Colors.WHITE}Goodbye!{Colors.RESET}")
            return False
            
        else:
            print(f"{Colors.RED}Unknown command: {command}{Colors.RESET}")
            print(f"Type 'help' for available commands")
            
        return True
        
    def run(self):
        """Main CLI loop"""
        self.print_header()
        self.print_help()
        self.print_status()
        
        while True:
            try:
                # Show prompt
                state = self.controller.get_state_data()['motion_state_name']
                color = self.get_state_color(state)
                prompt = f"{color}[{state}]{Colors.RESET}> "
                
                cmd = input(prompt)
                if not self.process_command(cmd):
                    break
                    
            except KeyboardInterrupt:
                print(f"\n{Colors.YELLOW}Interrupted{Colors.RESET}")
                if self.running:
                    self.pause_simulation()
                    
            except EOFError:
                break


def main():
    cli = SimulationCLI()
    cli.run()


if __name__ == "__main__":
    main()
