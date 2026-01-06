#!/usr/bin/env python3
"""
Visualization script for the PID velocity controller unit tests.
This script demonstrates the PID controller behavior with different gain configurations.
"""

import matplotlib.pyplot as plt
import numpy as np

# Simulation parameters (matching C++ implementation)
SAMPLING_TIME = 0.001  # 1ms
MAX_VELOCITY = 100.0
FINAL_VELOCITY = 10.0

def simulate_pid_controller(kp, ki, kd, desired_velocity, initial_velocity, num_steps=2000):
    """
    Simulate PID velocity controller behavior.
    
    Returns:
        times: array of time values
        desired_velocities: array of desired velocity values
        actual_velocities: array of actual (mass) velocity values
        control_velocities: array of control output values
        errors: array of error values
    """
    times = np.arange(num_steps) * SAMPLING_TIME
    desired_velocities = np.full(num_steps, desired_velocity)
    actual_velocities = np.zeros(num_steps)
    control_velocities = np.zeros(num_steps)
    errors = np.zeros(num_steps)
    
    # PID state
    integral_error = 0.0
    previous_error = 0.0
    
    # Initial conditions
    mass_velocity = initial_velocity
    
    for i in range(num_steps):
        # Calculate error
        error = desired_velocity - mass_velocity
        errors[i] = error
        
        # Proportional term
        p_term = kp * error
        
        # Integral term with anti-windup
        integral_error += error * SAMPLING_TIME
        max_integral = MAX_VELOCITY / (ki + 1e-6)
        integral_error = np.clip(integral_error, -max_integral, max_integral)
        i_term = ki * integral_error
        
        # Derivative term
        derivative = (error - previous_error) / SAMPLING_TIME
        d_term = kd * derivative
        previous_error = error
        
        # PID output
        pid_output = p_term + i_term + d_term
        
        # Control velocity
        control_velocity = desired_velocity + pid_output
        control_velocity = np.clip(control_velocity, 0.0, MAX_VELOCITY)
        
        # Store values
        actual_velocities[i] = mass_velocity
        control_velocities[i] = control_velocity
        
        # Simulate system response (first-order approximation)
        # The mass velocity approaches the control velocity with some dynamics
        tau = 0.05  # Time constant (50ms)
        mass_velocity += (control_velocity - mass_velocity) * (SAMPLING_TIME / tau)
    
    return times, desired_velocities, actual_velocities, control_velocities, errors


def plot_pid_response():
    """Create visualization of PID controller responses."""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('PID Velocity Controller Visualization', fontsize=14, fontweight='bold')
    
    # Test configurations
    configs = [
        {'kp': 1.0, 'ki': 0.0, 'kd': 0.0, 'name': 'P-only (Kp=1.0)'},
        {'kp': 0.0, 'ki': 10.0, 'kd': 0.0, 'name': 'I-only (Ki=10.0)'},
        {'kp': 0.0, 'ki': 0.0, 'kd': 0.01, 'name': 'D-only (Kd=0.01)'},
        {'kp': 2.0, 'ki': 20.0, 'kd': 0.01, 'name': 'PID (Kp=2.0, Ki=20.0, Kd=0.01)'},
        {'kp': 5.0, 'ki': 50.0, 'kd': 0.02, 'name': 'Aggressive PID'},
        {'kp': 0.5, 'ki': 5.0, 'kd': 0.005, 'name': 'Conservative PID'},
    ]
    
    desired_velocity = FINAL_VELOCITY
    initial_velocity = 0.0  # Start from rest
    
    for ax, config in zip(axes.flatten(), configs):
        times, desired, actual, control, errors = simulate_pid_controller(
            config['kp'], config['ki'], config['kd'],
            desired_velocity, initial_velocity, num_steps=3000
        )
        
        # Convert to milliseconds for display
        times_ms = times * 1000
        
        ax.plot(times_ms, desired, 'g--', label='Desired', linewidth=2)
        ax.plot(times_ms, actual, 'b-', label='Actual (Mass)', linewidth=1.5)
        ax.plot(times_ms, control, 'r-', label='Control Output', linewidth=1, alpha=0.7)
        
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Velocity (mm/s)')
        ax.set_title(config['name'])
        ax.legend(loc='lower right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlim([0, times_ms[-1]])
        ax.set_ylim([0, max(MAX_VELOCITY * 0.3, desired_velocity * 1.5)])
        
        # Add settling info
        final_error = abs(actual[-1] - desired_velocity)
        ax.text(0.02, 0.98, f'Final Error: {final_error:.3f} mm/s', 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig('pid_controller_response.png', dpi=150, bbox_inches='tight')
    print("Saved: pid_controller_response.png")
    plt.show()


def plot_step_response_comparison():
    """Compare step responses with different PID tunings."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Configuration for well-tuned PID
    kp, ki, kd = 2.0, 30.0, 0.01
    
    # Different step sizes
    step_tests = [
        {'initial': 0.0, 'target': 10.0, 'name': 'Step: 0 → 10 mm/s'},
        {'initial': 10.0, 'target': 50.0, 'name': 'Step: 10 → 50 mm/s'},
        {'initial': 80.0, 'target': 20.0, 'name': 'Step: 80 → 20 mm/s (decel)'},
        {'initial': 50.0, 'target': 50.0, 'name': 'Steady State: 50 mm/s'},
    ]
    
    for ax, test in zip(axes.flatten(), step_tests):
        times, desired, actual, control, errors = simulate_pid_controller(
            kp, ki, kd, test['target'], test['initial'], num_steps=2000
        )
        
        times_ms = times * 1000
        
        ax.plot(times_ms, desired, 'g--', label='Desired', linewidth=2)
        ax.plot(times_ms, actual, 'b-', label='Actual', linewidth=1.5)
        ax.plot(times_ms, control, 'r-', label='Control', linewidth=1, alpha=0.6)
        
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Velocity (mm/s)')
        ax.set_title(test['name'])
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Calculate metrics
        steady_state_error = abs(actual[-1] - test['target'])
        
        # Find rise time (10% to 90% of step)
        step_size = abs(test['target'] - test['initial'])
        if step_size > 0.1:
            threshold_10 = test['initial'] + 0.1 * (test['target'] - test['initial'])
            threshold_90 = test['initial'] + 0.9 * (test['target'] - test['initial'])
            
            if test['target'] > test['initial']:
                t10_idx = np.where(actual >= threshold_10)[0]
                t90_idx = np.where(actual >= threshold_90)[0]
            else:
                t10_idx = np.where(actual <= threshold_10)[0]
                t90_idx = np.where(actual <= threshold_90)[0]
            
            if len(t10_idx) > 0 and len(t90_idx) > 0:
                rise_time = (t90_idx[0] - t10_idx[0]) * SAMPLING_TIME * 1000
                ax.text(0.02, 0.98, f'Rise Time: {rise_time:.1f} ms\nSS Error: {steady_state_error:.3f}', 
                        transform=ax.transAxes, fontsize=8, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    fig.suptitle(f'Step Response Comparison (Kp={kp}, Ki={ki}, Kd={kd})', fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig('pid_step_response.png', dpi=150, bbox_inches='tight')
    print("Saved: pid_step_response.png")
    plt.show()


def plot_error_analysis():
    """Visualize error components and PID terms."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    kp, ki, kd = 2.0, 20.0, 0.01
    desired_velocity = 10.0
    initial_velocity = 0.0
    num_steps = 2000
    
    times = np.arange(num_steps) * SAMPLING_TIME * 1000  # in ms
    
    # Track all components
    p_terms = np.zeros(num_steps)
    i_terms = np.zeros(num_steps)
    d_terms = np.zeros(num_steps)
    errors = np.zeros(num_steps)
    actual_velocities = np.zeros(num_steps)
    
    integral_error = 0.0
    previous_error = 0.0
    mass_velocity = initial_velocity
    
    for i in range(num_steps):
        error = desired_velocity - mass_velocity
        errors[i] = error
        
        p_terms[i] = kp * error
        
        integral_error += error * SAMPLING_TIME
        max_integral = MAX_VELOCITY / (ki + 1e-6)
        integral_error = np.clip(integral_error, -max_integral, max_integral)
        i_terms[i] = ki * integral_error
        
        derivative = (error - previous_error) / SAMPLING_TIME
        d_terms[i] = kd * derivative
        previous_error = error
        
        pid_output = p_terms[i] + i_terms[i] + d_terms[i]
        control_velocity = np.clip(desired_velocity + pid_output, 0.0, MAX_VELOCITY)
        
        actual_velocities[i] = mass_velocity
        
        tau = 0.05
        mass_velocity += (control_velocity - mass_velocity) * (SAMPLING_TIME / tau)
    
    # Plot 1: Velocity tracking
    axes[0].plot(times, np.full(num_steps, desired_velocity), 'g--', label='Desired', linewidth=2)
    axes[0].plot(times, actual_velocities, 'b-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Velocity (mm/s)')
    axes[0].set_title('Velocity Tracking')
    axes[0].legend(loc='right')
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Error
    axes[1].plot(times, errors, 'r-', linewidth=1.5)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Error (mm/s)')
    axes[1].set_title('Velocity Error (Desired - Actual)')
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: PID terms
    axes[2].plot(times, p_terms, 'r-', label=f'P-term (Kp={kp})', linewidth=1.5)
    axes[2].plot(times, i_terms, 'g-', label=f'I-term (Ki={ki})', linewidth=1.5)
    axes[2].plot(times, d_terms, 'b-', label=f'D-term (Kd={kd})', linewidth=1.5, alpha=0.7)
    axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[2].set_xlabel('Time (ms)')
    axes[2].set_ylabel('PID Term Value')
    axes[2].set_title('Individual PID Term Contributions')
    axes[2].legend(loc='right')
    axes[2].grid(True, alpha=0.3)
    
    fig.suptitle('PID Controller Error Analysis', fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig('pid_error_analysis.png', dpi=150, bbox_inches='tight')
    print("Saved: pid_error_analysis.png")
    plt.show()


if __name__ == '__main__':
    print("Generating PID Velocity Controller Visualizations...")
    print("=" * 50)
    
    plot_pid_response()
    plot_step_response_comparison()
    plot_error_analysis()
    
    print("=" * 50)
    print("All visualizations complete!")
