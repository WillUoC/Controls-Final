import matplotlib.pyplot as plt
import numpy as np
import Drone.DroneParam as P
from util.signalGenerator import signalGenerator
from Drone.DroneAnimation import DroneAnimation
from util.dataPlotter import dataPlotter
from Drone.DroneDynamics import DroneDynamics
from Drone.DroneCommander import DroneCommander
import logging

# Initialize Logging
logging.basicConfig(level=logging.INFO)


# instantiate reference input classes
reference = signalGenerator(amplitude=1, frequency=1/15)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = DroneAnimation()
drone = DroneDynamics()
commander = DroneCommander()


def saturate(u, llimit, ulimit):

    if u < llimit:
        u = llimit
    elif u > ulimit:
        u = ulimit
    return u

u = np.array([
    [0.64],
    [0.64],
    [0.64],
    [0.64]
])

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # Propagate dynamics at rate Ts
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        drone.update(u)

        F, taux, tauy, tauz = commander.update(drone.state)
        tlimit = 100.0

        u = np.array([
            [(F-4*P.b_thrust)/(4*P.m_thrust) - tauz/(4*P.m_rot*P.mu_r) + (np.sqrt(2)*taux)/(4*P.d*P.m_thrust) - (np.sqrt(2)*tauy)/(4*P.d*P.m_thrust)],
            [(F-4*P.b_thrust)/(4*P.m_thrust) + tauz/(4*P.m_rot*P.mu_r) - (np.sqrt(2)*taux)/(4*P.d*P.m_thrust) - (np.sqrt(2)*tauy)/(4*P.d*P.m_thrust)],
            [(F-4*P.b_thrust)/(4*P.m_thrust) + tauz/(4*P.m_rot*P.mu_r) + (np.sqrt(2)*taux)/(4*P.d*P.m_thrust) + (np.sqrt(2)*tauy)/(4*P.d*P.m_thrust)],
            [(F-4*P.b_thrust)/(4*P.m_thrust) - tauz/(4*P.m_rot*P.mu_r) - (np.sqrt(2)*taux)/(4*P.d*P.m_thrust) + (np.sqrt(2)*tauy)/(4*P.d*P.m_thrust)]
        ])

        for ind, i in enumerate(u):
            u[ind] = saturate(i, -tlimit, tlimit)
        t = t + P.Ts

    animation.update(drone.state)
    dataPlot.update(t, drone.state, u, tlimit, 0.0)
    plt.pause(P.Ts)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()