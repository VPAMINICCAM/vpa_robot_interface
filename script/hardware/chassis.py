class CHASSIS:
    def __init__(self, wheel_diameter: float):
        self.wheel_diameter = wheel_diameter

    def calculate_wheel_speeds(self, linear_x: float):
        """Calculate wheel speeds in rps."""
        if linear_x != 0:
            return linear_x / (3.14 * self.wheel_diameter)
        return 0


    def yaw2steerratio(self, linear_x: float, yaw_demand: float) -> float:
        """
        Calculates the steering ratio based on linear velocity and yaw demand.

        Args:
            linear_x (float): The linear velocity.
            yaw_demand (float): The yaw demand.

        Returns:
            float: The steering ratio, constrained between -1 and 1.
        """
        if linear_x == 0:
            inver_R = 0
        else:
        # Calculate the inverse radius
            inver_R = yaw_demand / linear_x

        # Constrain the inverse radius
        if inver_R > 2:
            inver_R = 2
        elif inver_R < -2:
            inver_R = -2

        # Calculate feedforward steering ratio
        if inver_R < 0:
            inver_R = -inver_R
        str_ff = 0.08 * inver_R**3 + 0.08 * inver_R**2 + 0.12 * inver_R + 0.05
    
        # Constrain the steering ratio
        if str_ff > 1:
            str_ff = 1
        elif str_ff < -1:
            str_ff = -1

        return str_ff



