#!/usr/bin/python3

class PI_controller:

    # This is an incremental PID control

    def __init__(self,kp,ki) -> None:
        
        self.kp = kp
        self.ki = ki
        self.control_output = 0
        self.err_record     = 0

    def record_current_error(self,err) -> None:
        self.err_record = err

    def pi_control(self,ref,sig) -> float:

        err = ref - sig
        delta_err = err - self.err_record
        
        self.record_current_error(err)

        delta_output = self.kp * delta_err + self.ki * err

        self.control_output += delta_output

        return self.control_output
    
    def update_controller_param(self,kp,ki) -> None:

        self.kp = kp
        self.ki = ki
        
    def reset_controller(self):
        
        self.control_output = 0
        self.err_record     = 0       