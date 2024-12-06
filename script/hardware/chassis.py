class CHASSIS:
    def __init__(self, wheel_diameter: float):
        self.wheel_diameter = wheel_diameter

    def calculate_wheel_speeds(self, linear_x: float):
        """Calculate wheel speeds in rps."""
        if linear_x != 0:
            return linear_x / (3.14 * self.wheel_diameter)
        return 0
