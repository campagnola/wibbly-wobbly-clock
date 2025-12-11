import math

class PIDController:
    """Generic PID controller implementation with exponential integral averaging."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None, integral_time_constant=1.0):
        """Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output limits, or None for unlimited
            integral_time_constant: Time constant for exponential integral decay (seconds)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_time_constant = integral_time_constant

        self.integral = 0.0
        self.last_error = 0.0
        self.last_terms = (0.0, 0.0, 0.0)  # (P, I, D)
        self.last_output = 0.0

    def update(self, setpoint, measured_value, dt):
        """Update the PID controller.

        Args:
            setpoint: Desired value
            measured_value: Current measured value
            dt: Time step in seconds

        Returns:
            Control output
        """
        if dt <= 0:
            return 0.0

        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term with exponential decay
        # Decay old integral and add new error contribution
        decay = math.exp(-dt / self.integral_time_constant)
        self.integral = self.integral * decay + error * dt
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error

        # Calculate output
        output = p_term + i_term + d_term
        self.last_terms = (p_term, i_term, d_term)
        self.last_output = output

        # Apply output limits
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))

        return output

    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.last_error = 0.0

    def state_str(self):
        return (
            f"KP={self.kp:0.1f}, KI={self.ki:0.1f}, KD={self.kd:0.1f}, Integral={self.integral:0.1f}, "
            f"error={self.last_error:0.1f}, output={self.last_output:0.1f}, "
            f"terms(P,I,D)={self.last_terms[0]:0.1f},{self.last_terms[1]:0.1f},{self.last_terms[2]:0.1f}"
        )

