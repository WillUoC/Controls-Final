import numpy as np
from .. import DroneParam as P


class FeedbackLoop:
    def __init__(self, K, kr, lower_limit = None, upper_limit = None, ki=None, sample_rate=None):
        self.K = K # state feedback gain
        self.kr = kr # Input gain

        self.ulimit = upper_limit # Maximum force
        self.llimit = lower_limit

        self.Ts = sample_rate
        self.error_d1 = 0.0
        self.integrator = 0.0

        self.ki = ki

    def update(self, x_r, x):
        print(self.K, x, self.kr, x_r)
        y_unsat = -self.K @ x + self.kr @ x_r


        if self.llimit is not None and self.ulimit is not None:
            return(self.saturate(y_unsat.item(0), self.ulimit, self.llimit))
        else:
            return(y_unsat.item(0))
    
    def update_int(self, x_r, x):
        error = x_r.item(0) - x.item(0)
        self.integrate_error(error)
        y_unsat = -self.K @ x - self.ki * self.integrator

        if self.llimit is not None and self.ulimit is not None:
            return(self.saturate(y_unsat.item(0), self.ulimit, self.llimit))
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