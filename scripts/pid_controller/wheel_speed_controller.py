import numpy as np

class WheelSpeedController:
    def __init__(self, Ts=0.05, Kp= np.array([0.3, 0.3]), Ki = np.array([1, 1])):
        self.Ts = Ts
        self.Kp = Kp
        self.Ki = Ki
        self.integral = np.zeros(2)
    
    def compute(self,w_meas,w_ref):
        """
        Compute the control signal for the wheel speed controller.
        
        :param w_meas: Measured wheel speeds (numpy array of shape (2,))
        :param w_ref: Reference wheel speeds (numpy array of shape (2,))
        :return: Control signal (numpy array of shape (2,))
        """
        
        w_meas = np.array(w_meas)
        w_ref = np.array(w_ref)
        err = w_ref - w_meas

        if np.all(w_ref == 0):
            self.integral = np.zeros(2)
            u_raw = np.zeros(2)
        else:
            u_raw = self.Kp * err + self.Ki * self.integral
        
        u = np.clip(u_raw, 0, 1)

        for i in range(2):
            if (u_raw[i] > 1 and err[i] > 0) or (u_raw[i] < 0 and err[i] < 0):
                continue
            self.integ_err[i] += self.Ts * err[i]

        return u
    
    def update_gains(self, kp_left, ki_left, kp_right, ki_right):
        self.Kp = np.array([kp_left, kp_right])
        self.Ki = np.array([ki_left, ki_right])