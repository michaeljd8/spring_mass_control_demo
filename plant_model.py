"""
Spring-Mass System Simulation

Simulates a spring-mass-damper system with input displacement at the spring.
Equation: m * x_ddot + c * x_dot + k * x = k * x_in + c * x_in_dot
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt


class SpringMassSystem:
    """A spring-mass-damper system simulation with friction."""
    
    def __init__(self, mass=1.0, spring_constant=10.0, damping=0.5, 
                 viscous_friction=0.1, coulomb_friction=0.5):
        self.m = mass
        self.k = spring_constant
        self.c = damping
        self.b = viscous_friction      # Viscous friction coefficient (N*s/m)
        self.fc = coulomb_friction     # Coulomb friction force (N)
    
    def simulate(self, t_span, x_in_func):
        """Simulate the system response to an input displacement function."""
        def derivatives(state, t):
            x, x_dot = state
            x_in, x_in_dot = x_in_func(t)
            
            # Spring and damper forces
            spring_force = self.k * (x_in - x)
            damper_force = self.c * (x_in_dot - x_dot)
            
            # Friction force on mass (opposes motion)
            # Viscous: proportional to velocity
            # Coulomb: constant magnitude, opposes motion direction
            viscous_force = -self.b * x_dot
            if abs(x_dot) > 1e-6:
                coulomb_force = -self.fc * np.sign(x_dot)
            else:
                # Static friction: oppose net force up to fc magnitude
                net_force = spring_force + damper_force
                coulomb_force = -np.clip(net_force, -self.fc, self.fc)
            
            x_ddot = (spring_force + damper_force + viscous_force + coulomb_force) / self.m
            return [x_dot, x_ddot]
        
        solution = odeint(derivatives, [0.0, 0.0], t_span)
        return {
            'time': t_span,
            'position': solution[:, 0],
            'velocity': solution[:, 1],
            'input': np.array([x_in_func(t)[0] for t in t_span])
        }


def constant_velocity_input(velocity=1.0):
    """Input that moves at constant velocity: x_in = v*t"""
    return lambda t: (velocity * t, velocity)


def step_input(amplitude=1.0):
    """Step input: x_in jumps to amplitude at t=0"""
    return lambda t: (amplitude, 0.0)


if __name__ == "__main__":
    # Use step input to see damping effects clearly
    # Try damping=0.5 (underdamped), 2.0 (critically damped), 10.0 (overdamped)
    system = SpringMassSystem(mass=1.0, spring_constant=10.0, damping=0.5)
    t = np.linspace(0, 10, 1000)
    results = system.simulate(t, constant_velocity_input(velocity=1.0))
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax1.plot(t, results['input'], 'b--', label='Input')
    ax1.plot(t, results['position'], 'r-', label='Mass Position')
    ax1.set_ylabel('Position (m)')
    ax1.legend()
    ax1.grid(True)
    
    ax2.plot(t, results['velocity'], 'g-', label='Mass Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('step_response.png', dpi=150)
    plt.show()
