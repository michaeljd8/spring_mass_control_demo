# PID Controller Class with Adjustment Output

class Controller:
    """
    A PID controller class for the spring-mass system.
    This class calculates the adjustment to the current set velocity to drive the cart to the target velocity.
    If all gains are set to 0, the controller runs in open-loop mode.
    """

    def __init__(self, target_velocity, kp=0.1, ki=0.01, kd=0.05):
        """
        Initialize the PID controller with target velocity and PID gains.

        :param target_velocity: The desired velocity of the cart (m/s).
        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        """
        self.target_velocity = target_velocity
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, measured_velocity, dt):
        """
        Update the controller with the measured velocity and calculate the adjustment to the current set velocity.

        :param measured_velocity: The current velocity of the cart (m/s).
        :param dt: Time step (s).
        :return: The velocity adjustment to apply to the system.
        """
        
        # Calculate error
        error = self.target_velocity - measured_velocity

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt

        # PID output (adjustment to the current velocity)
        velocity_adjustment = proportional + integral + derivative

        # Update previous error
        self.previous_error = error

        return velocity_adjustment

    def reset(self):
        """
        Reset the controller state.
        """
        self.previous_error = 0.0
        self.integral = 0.0