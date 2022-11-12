from Drone.Aero2Animation import Aero2Animation
import numpy as np
import matplotlib.pyplot as plt

animation = Aero2Animation()

x = np.array([
    [0],
    [0],
    [0]
    ])

animation.update(x)
plt.show()