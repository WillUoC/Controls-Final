import numpy as np
from .. import DroneParam as P


class FeedbackLoop:
    def __init__(self, K, kr, K2i=None, ki=None, lower_limit = None, upper_limit = None, sample_rate=None):
        self.K = K2i # state feedback gain
        self.kr = kr # Input gain

        self.K2 = K

        self.ulimit = upper_limit # Maximum force
        self.llimit = lower_limit

        self.Ts = sample_rate
        self.error_d1 = 0.0
        self.integrator = 0.0

        self.ki = ki

    def update(self, x_r, x):
        y_unsat = -self.K2 @ x + self.kr @ x_r


        if self.llimit is not None and self.ulimit is not None:
            return(self.saturate(y_unsat.item(0), self.ulimit, self.llimit))
        else:
            return(y_unsat.item(0))
    
    def update_int(self, x_r, x):
        error = x_r.item(0) - x.item(0)
        self.integrate_error(error)
        y_unsat = -self.K @ x - self.ki * self.integrator

        if self.llimit is not None and self.ulimit is not None:
            y_sat = self.saturate(y_unsat.item(0), self.ulimit, self.llimit)
            self.integrator = self.integrator - 1.0/self.ki * (y_sat - y_unsat)
            return(y_sat)
        else:
            return(y_unsat.item(0))

    def integrate_error(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error
    
    def reset_integrator(self):
        self.integrator = 0.0
        self.error_d1 = 0.0
    
    def saturate(self, u, ulimit, dlimit):
        if u > ulimit:
            u = ulimit
        elif u < dlimit:
            u = dlimit
        return u