class CHASSIS:
    def __init__(self, wheel_diameter: float, wheelbase: float):
        """Initialize the chassis with wheel diameter and wheelbase."""
        self.wheel_diameter = wheel_diameter  # Diameter of the wheels
        self.wheelbase = wheelbase  # Distance between the two wheels

        self.trim = 0

    def calculate_wheel_speeds(self, linear_x: float, angular_z: float):
        """Calculate left (A) and right (B) wheel speeds in radians per second (rps) based on the linear and angular velocity from cmd_vel."""
        if not linear_x == 0: 
            r_left = linear_x - (self.wheelbase * angular_z) 
            r_right = linear_x + (self.wheelbase * angular_z)

            omega_left = ((r_left) / (self.wheel_diameter * 3.14)) * (1 - self.trim)
            omega_right = (r_right) / (self.wheel_diameter * 3.14) * (1 + self.trim)
        else:
            omega_left  = 0
            omega_right = 0
        return omega_left, omega_right
        