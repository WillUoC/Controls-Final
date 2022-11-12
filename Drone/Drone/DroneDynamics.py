import numpy as np 
import Drone.DroneParam as P

class DroneDynamics:
    
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.x0],
            [P.y0],
            [P.z0],
            [P.theta0],  # z initial position
            [P.alpha0],
            [P.psi0],
            [P.xdot0],
            [P.ydot0],
            [P.zdot0],
            [P.thetadot0],  # Theta initial orientation
            [P.alphadot0],
            [P.psidot0]  # zdot initial velocity
        ])

        # simulation time step
        self.Ts = P.Ts
        self.mc = P.mc
        self.jc = P.jc

        self.mu_lat = P.mu_lat
        self.mu_r = P.mu_r

        self.m_thrust = P.m_thrust
        self.b_thrust = P.b_thrust

        self.m_rot = P.m_rot
        self.b_rot = P.b_rot

        self.d = P.d

        # gravity constant is well known, don't change.
        self.g = P.g


    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input force
        
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output

        return y

    def f(self, state, u):
        # Return xdot = f(x,u)
        x = state.item(0)
        y = state.item(1)
        z = state.item(2)
        theta = state.item(3)
        alpha = state.item(4)
        psi = state.item(5)
        xdot = state.item(6)
        ydot = state.item(7)
        zdot = state.item(8)
        thetadot = state.item(9)
        alphadot = state.item(10)
        psidot = state.item(11)
        
        m_conv = np.array([
            [self.m_thrust, self.m_thrust, self.m_thrust, self.m_thrust],
            [self.d*self.m_thrust*np.sqrt(2)/2, -self.d*self.m_thrust*np.sqrt(2)/2, self.d*self.m_thrust*np.sqrt(2)/2, -self.d*self.m_thrust*np.sqrt(2)/2],
            [-self.d*self.m_thrust*np.sqrt(2)/2, -self.d*self.m_thrust*np.sqrt(2)/2, self.d*self.m_thrust*np.sqrt(2)/2, self.d*self.m_thrust*np.sqrt(2)/2],
            [-self.mu_r*self.m_rot, self.mu_r*self.m_rot, self.mu_r*self.m_rot, -self.mu_r*self.m_rot]
        ])

        b_conv = np.array([
            [4*self.b_thrust],
            [0.0],
            [0.0],
            [0.0]
        ])

        Ft, taux, tauy, tauz = m_conv@u + b_conv

        Ft = Ft[0]
        taux = taux[0]
        tauy = tauy[0]
        tauz = tauz[0]

        # The equations of motion.        
        M = np.array([
            [self.mc, 0, 0, 0, 0, 0],
            [0, self.mc, 0, 0, 0, 0],
            [0, 0, self.mc, 0, 0, 0],
            [0, 0, 0, self.jc, 0, 0],
            [0, 0, 0, 0, self.jc, 0],
            [0, 0, 0, 0, 0, self.jc]])

        C = np.array([
            [Ft*(np.sin(alpha)*np.cos(psi)*np.cos(theta) + np.sin(psi)*np.sin(theta)) - self.mu_lat*xdot],
            [Ft*(np.sin(alpha)*np.sin(psi)*np.cos(theta) - np.cos(psi)*np.sin(theta)) - self.mu_lat*xdot],
            [Ft*np.cos(alpha)*np.cos(theta)-self.g*self.mc],
            [taux*np.cos(alpha)*np.cos(psi) + tauy*(np.sin(alpha)*np.sin(theta)*np.cos(psi) - np.sin(psi)*np.cos(theta)) + tauz*(np.sin(alpha)*np.sin(psi)*np.cos(theta) - np.sin(theta)*np.cos(psi))],
            [taux*np.sin(psi)*np.cos(alpha) + tauy*(np.sin(alpha)*np.sin(psi)*np.sin(theta) + np.cos(psi)*np.cos(theta)) + tauz*(np.sin(alpha)*np.sin(psi)*np.cos(theta) - np.sin(theta)*np.cos(psi))],
            [-taux*np.sin(alpha) + tauy*np.sin(theta)*np.cos(alpha) + tauz*np.cos(alpha)*np.cos(theta)]
        ])

        tmp = np.linalg.inv(M) @ C

        xddot = tmp.item(0)
        yddot = tmp.item(1)
        zddot = tmp.item(2)
        thetaddot = tmp.item(3)
        alphaddot = tmp.item(4)
        psiddot = tmp.item(5)

        # build xdot and return
        xdot = np.array([[xdot], [ydot], [zdot], [thetadot], [alphadot], [psidot], [xddot], [yddot], [zddot], [thetaddot], [alphaddot], [psiddot]])

        return xdot

    def h(self):
        # return y = h(x)
        theta = self.state.item(0)
        psi = self.state.item(1)
        y = np.array([[theta], [psi]])

        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state = self.state + self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):

        for item, ind in enumerate(u):
            if abs(item) > limit:
                u[ind] = limit*np.sign(item)

        return u
