class DeltaPIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min, self.output_max = output_limits

        self.prev_error = 0.0
        self.prev_prev_error = 0.0
        self.output = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.prev_prev_error = 0.0
        self.output = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement

        # Delta PID update
        delta_output = (
            self.kp * (error - self.prev_error)
            + self.ki * error
            + self.kd * (error - 2 * self.prev_error + self.prev_prev_error)
        )

        self.output += delta_output

        # Clamp output
        if self.output_min is not None:
            self.output = max(self.output_min, self.output)
        if self.output_max is not None:
            self.output = min(self.output_max, self.output)

        # Update history
        self.prev_prev_error = self.prev_error
        self.prev_error = error

        return self.output

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None), integral_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits

        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement

        # Integral term with anti-windup
        self.integral += error
        if self.integral_min is not None:
            self.integral = max(self.integral_min, self.integral)
        if self.integral_max is not None:
            self.integral = min(self.integral_max, self.integral)

        # Derivative term
        derivative = error - self.prev_error

        # PID computation
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        # Clamp output
        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        # Save error for next step
        self.prev_error = error

        return output


if __name__ == "__main__":
    test_pid = PIDController(kp=-0.03,ki=0,kd=0, output_limits=(-5.0, 5.0))
    error = 356
    print(f"PID output for error {error}: {test_pid.compute(0, error)}")
