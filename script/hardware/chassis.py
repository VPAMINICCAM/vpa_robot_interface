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
        
    def caculate_wheel_speeds_rps(self,linear_x:float,angular_z:float):
        """Calculate left (A) and right (B) wheel speeds in revolutions per second (rps) based on the linear and angular velocity from cmd_vel."""
        if not linear_x == 0:
            r_left = linear_x - (self.wheelbase/2 * angular_z) * (1 - self.trim)
            r_right = linear_x + (self.wheelbase/2 * angular_z) * (1 + self.trim)
            omega_left = r_left / (self.wheel_diameter * 3.14)
            omega_right = r_right / (self.wheel_diameter * 3.14)
        else:
            omega_left = 0
            omega_right = 0
        return omega_left, omega_right
    
    def calculate_yaw_rate_from_wheelspd(self, omega_left: float, omega_right: float):
        """Calculate the yaw rate based on the left and right wheel speeds in revolutions per second (rps)."""
        # Convert wheel speeds from rps to linear velocities
        v_left = omega_left * self.wheel_diameter * 3.14
        v_right = omega_right * self.wheel_diameter * 3.14
        # Calculate yaw rate
        return (v_right - v_left) / self.wheelbase