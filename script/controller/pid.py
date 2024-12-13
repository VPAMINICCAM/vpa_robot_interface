class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, output_limits=(None, None), integral_limits=(None, None), smoothing_factor=0.1):
        """
        Initialize the PID controller.
        :param Kp: Proportional gain
        :param Ki: Integral gain
        :param Kd: Derivative gain
        :param output_limits: Tuple (min, max) for output limiting
        :param integral_limits: Tuple (min, max) for integral term limiting
        :param smoothing_factor: Factor for exponential smoothing (0.0 to 1.0, where 1.0 means no smoothing)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.smoothing_factor = smoothing_factor

        self.setpoint = 0
        self._prev_error = 0
        self._integral = 0
        self._last_output = 0  # To apply smoothing

    def compute(self, setpoint, measurement):
        """
        Compute the PID output.
        :param setpoint: Desired value
        :param measurement: Current value
        :return: Smoothed PID output
        """
        error = setpoint - measurement
        self._integral += error

        # Apply integral saturation limits
        if self.integral_limits[0] is not None:
            self._integral = max(self.integral_limits[0], self._integral)
        if self.integral_limits[1] is not None:
            self._integral = min(self.integral_limits[1], self._integral)

        derivative = error - self._prev_error

        # PID output
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)

        # Apply output limits
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        # Apply smoothing
        smoothed_output = self._last_output + self.smoothing_factor * (output - self._last_output)
        self._last_output = smoothed_output

        self._prev_error = error
        return smoothed_output
