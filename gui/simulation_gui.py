"""
Spring Mass Control Simulation GUI
Tkinter-based GUI for interacting with the C++ simulation backend via pybind11
"""

import tkinter as tk
from tkinter import ttk
import sys
import os

# Add build directory to path for the compiled module
build_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'build')
sys.path.insert(0, build_dir)

try:
    import spring_mass_sim as sim
except ImportError as e:
    print(f"Error importing spring_mass_sim module: {e}")
    print(f"Make sure to build the module first: cd build && cmake .. && make")
    print(f"Looking in: {build_dir}")
    sys.exit(1)


class SimulationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Spring Mass Control Simulation")
        self.root.geometry("800x600")
        
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
        self.data_log = []
        
        # Steps per GUI update (run multiple sim steps per frame for speed)
        self.steps_per_update = 10
        
        self.create_widgets()
        
    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # === Control Buttons Frame ===
        btn_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        btn_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nw")
        
        self.btn_start_extend = ttk.Button(btn_frame, text="Start Extend", 
                                           command=self.on_start_extend)
        self.btn_start_extend.grid(row=0, column=0, padx=5, pady=5)
        
        self.btn_start_retract = ttk.Button(btn_frame, text="Start Retract", 
                                            command=self.on_start_retract)
        self.btn_start_retract.grid(row=0, column=1, padx=5, pady=5)
        
        self.btn_stop = ttk.Button(btn_frame, text="STOP", 
                                   command=self.on_stop, style="Danger.TButton")
        self.btn_stop.grid(row=1, column=0, padx=5, pady=5)
        
        self.btn_reset = ttk.Button(btn_frame, text="Reset", 
                                    command=self.on_reset)
        self.btn_reset.grid(row=1, column=1, padx=5, pady=5)
        
        # Simulation control
        self.btn_run = ttk.Button(btn_frame, text="Run Simulation", 
                                  command=self.toggle_simulation)
        self.btn_run.grid(row=2, column=0, columnspan=2, padx=5, pady=10)
        
        # Speed control
        ttk.Label(btn_frame, text="Sim Speed:").grid(row=3, column=0, padx=5, pady=2)
        self.speed_var = tk.IntVar(value=10)
        speed_spinbox = ttk.Spinbox(btn_frame, from_=1, to=100, 
                                    textvariable=self.speed_var, width=5)
        speed_spinbox.grid(row=3, column=1, padx=5, pady=2)
        
        # === State Display Frame ===
        state_frame = ttk.LabelFrame(main_frame, text="Current State", padding="10")
        state_frame.grid(row=0, column=1, padx=5, pady=5, sticky="new")
        
        # Motion state
        ttk.Label(state_frame, text="Motion State:").grid(row=0, column=0, sticky="e", padx=5)
        self.state_label = ttk.Label(state_frame, text="Home", font=("Arial", 12, "bold"))
        self.state_label.grid(row=0, column=1, sticky="w", padx=5)
        
        # Time
        ttk.Label(state_frame, text="Time (s):").grid(row=1, column=0, sticky="e", padx=5)
        self.time_label = ttk.Label(state_frame, text="0.000")
        self.time_label.grid(row=1, column=1, sticky="w", padx=5)
        
        # === Data Display Frame ===
        data_frame = ttk.LabelFrame(main_frame, text="Simulation Data", padding="10")
        data_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")
        main_frame.rowconfigure(1, weight=1)
        
        # Create data labels
        self.data_labels = {}
        data_items = [
            ("Drive Position (mm)", "drive_position"),
            ("Drive Velocity (mm/s)", "drive_velocity"),
            ("Desired Velocity (mm/s)", "desired_velocity"),
            ("Mass Position (mm)", "mass_position"),
            ("Mass Velocity (mm/s)", "mass_velocity"),
        ]
        
        for i, (label_text, key) in enumerate(data_items):
            ttk.Label(data_frame, text=f"{label_text}:").grid(row=i, column=0, sticky="e", padx=5, pady=2)
            self.data_labels[key] = ttk.Label(data_frame, text="0.00", width=15)
            self.data_labels[key].grid(row=i, column=1, sticky="w", padx=5, pady=2)
        
        # === Parameters Frame ===
        param_frame = ttk.LabelFrame(main_frame, text="Parameters", padding="10")
        param_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # Parameter display
        params = [
            ("Final Velocity", self.controller.get_final_velocity(), "mm/s"),
            ("Approach Distance", self.controller.get_approach_distance(), "mm"),
            ("Final Distance", self.controller.get_final_distance(), "mm"),
            ("Travel Velocity", self.controller.get_travel_velocity(), "mm/s"),
        ]
        
        for i, (name, value, unit) in enumerate(params):
            col = i % 2 * 2
            row = i // 2
            ttk.Label(param_frame, text=f"{name}:").grid(row=row, column=col, sticky="e", padx=5)
            ttk.Label(param_frame, text=f"{value:.1f} {unit}").grid(row=row, column=col+1, sticky="w", padx=5)
        
        # === Data Log Frame ===
        log_frame = ttk.LabelFrame(main_frame, text="Data Log (last 10 entries)", padding="10")
        log_frame.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")
        main_frame.rowconfigure(3, weight=2)
        
        # Create treeview for data log
        columns = ("time", "state", "drive_pos", "mass_pos", "mass_vel")
        self.log_tree = ttk.Treeview(log_frame, columns=columns, show="headings", height=8)
        
        self.log_tree.heading("time", text="Time (s)")
        self.log_tree.heading("state", text="State")
        self.log_tree.heading("drive_pos", text="Drive Pos")
        self.log_tree.heading("mass_pos", text="Mass Pos")
        self.log_tree.heading("mass_vel", text="Mass Vel")
        
        for col in columns:
            self.log_tree.column(col, width=100)
        
        self.log_tree.grid(row=0, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_tree.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_tree.configure(yscrollcommand=scrollbar.set)
        
        # Initial update
        self.update_display()
        
    def on_start_extend(self):
        state = self.controller.get_motion_state()
        if state == sim.MotionState.Home:
            self.controller.start_extend()
            self.update_display()
            
    def on_start_retract(self):
        state = self.controller.get_motion_state()
        if state == sim.MotionState.At_Final_Distance:
            self.controller.start_retract()
            self.update_display()
            
    def on_stop(self):
        self.controller.manual_stop()
        self.running = False
        self.btn_run.config(text="Run Simulation")
        self.update_display()
        
    def on_reset(self):
        self.running = False
        self.btn_run.config(text="Run Simulation")
        self.controller.full_reset()
        self.data_log.clear()
        # Clear log tree
        for item in self.log_tree.get_children():
            self.log_tree.delete(item)
        self.update_display()
        
    def toggle_simulation(self):
        self.running = not self.running
        if self.running:
            self.btn_run.config(text="Pause Simulation")
            self.run_simulation_loop()
        else:
            self.btn_run.config(text="Run Simulation")
            
    def run_simulation_loop(self):
        if not self.running:
            return
            
        # Run multiple steps per GUI update for speed
        steps = self.speed_var.get()
        for _ in range(steps):
            self.controller.step()
            
        # Log data periodically (every 100 steps = 100ms of sim time)
        data = self.controller.get_state_data()
        
        # Update log every ~50ms of simulation time
        if len(self.data_log) == 0 or (data['time'] - self.data_log[-1]['time']) >= 0.05:
            self.data_log.append(data)
            self.add_log_entry(data)
        
        self.update_display()
        
        # Schedule next update (16ms = ~60fps GUI update)
        self.root.after(16, self.run_simulation_loop)
        
    def update_display(self):
        data = self.controller.get_state_data()
        
        # Update state label with color
        state_name = data['motion_state_name']
        self.state_label.config(text=state_name)
        
        # Color coding for state
        state_colors = {
            "Home": "green",
            "Extending": "blue",
            "Final_Velocity": "cyan",
            "At_Final_Distance": "purple",
            "Retracting": "orange",
            "Manual_Stop": "red",
            "Error": "darkred"
        }
        self.state_label.config(foreground=state_colors.get(state_name, "black"))
        
        # Update time
        self.time_label.config(text=f"{data['time']:.3f}")
        
        # Update data labels
        self.data_labels["drive_position"].config(text=f"{data['drive_position']:.2f}")
        self.data_labels["drive_velocity"].config(text=f"{data['drive_velocity']:.2f}")
        self.data_labels["desired_velocity"].config(text=f"{data['desired_velocity']:.2f}")
        self.data_labels["mass_position"].config(text=f"{data['mass_position']:.2f}")
        self.data_labels["mass_velocity"].config(text=f"{data['mass_velocity']:.2f}")
        
        # Update button states based on motion state
        state = self.controller.get_motion_state()
        self.btn_start_extend.config(state="normal" if state == sim.MotionState.Home else "disabled")
        self.btn_start_retract.config(state="normal" if state == sim.MotionState.At_Final_Distance else "disabled")
        
    def add_log_entry(self, data):
        # Keep only last 100 entries in tree
        children = self.log_tree.get_children()
        if len(children) > 100:
            self.log_tree.delete(children[0])
            
        self.log_tree.insert("", "end", values=(
            f"{data['time']:.3f}",
            data['motion_state_name'],
            f"{data['drive_position']:.2f}",
            f"{data['mass_position']:.2f}",
            f"{data['mass_velocity']:.2f}"
        ))
        # Auto-scroll to bottom
        self.log_tree.yview_moveto(1.0)


def main():
    root = tk.Tk()
    app = SimulationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
