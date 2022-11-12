import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')
import Drone.DroneParam as P

from mpl_toolkits.mplot3d import Axes3D

class DroneAnimation:
    '''
        Create VTOL animation
    '''
    def __init__(self):
        def generate_rotor_points(CIRCLE_RESOLUTION, ROTOR_RADIUS):


            rotor1xy = np.array([[0, 0, 0]])
            rotor2xy = np.array([[0, 0, 0]])
            rotor3xy = np.array([[0, 0, 0]])
            rotor4xy = np.array([[0, 0, 0]])

            for i in range(CIRCLE_RESOLUTION):
                x = ROTOR_RADIUS * np.sin(((i+1)/CIRCLE_RESOLUTION)*2*np.pi)
                y = ROTOR_RADIUS * np.cos(((i+1)/CIRCLE_RESOLUTION)*2*np.pi)

                rotor1xy = np.append(rotor1xy, [[x+P.d*np.sqrt(2)/2, y+P.d*np.sqrt(2)/2, 0]], axis=0)
                rotor2xy = np.append(rotor2xy, [[x+P.d*np.sqrt(2)/2, y-P.d*np.sqrt(2)/2, 0]], axis=0)
                rotor3xy = np.append(rotor3xy, [[x-P.d*np.sqrt(2)/2, y+P.d*np.sqrt(2)/2, 0]], axis=0)
                rotor4xy = np.append(rotor4xy, [[x-P.d*np.sqrt(2)/2, y-P.d*np.sqrt(2)/2, 0]], axis=0)
            
            rotor1xy = np.delete(rotor1xy, 0, 0)
            rotor2xy = np.delete(rotor2xy, 0, 0)
            rotor3xy = np.delete(rotor3xy, 0, 0)
            rotor4xy = np.delete(rotor4xy, 0, 0)



            return(rotor1xy, rotor2xy, rotor3xy, rotor4xy)
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        CIRCLE_RESOLUTION = P.rotor_resolution
        ROTOR_RADIUS = P.rotor_radius

        self.rotor1xy, self.rotor2xy, self.rotor3xy, self.rotor4xy = generate_rotor_points(CIRCLE_RESOLUTION, ROTOR_RADIUS)

        self.linexy = np.array([
            [P.d*np.sqrt(2)/2, P.d*np.sqrt(2)/2, 0],
            [-P.d*np.sqrt(2)/2, -P.d*np.sqrt(2)/2, 0]
        ])

        self.line2xy = np.array([
            [P.d*np.sqrt(2)/2, -P.d*np.sqrt(2)/2, 0],
            [-P.d*np.sqrt(2)/2, P.d*np.sqrt(2)/2, 0]
        ])

    def update(self, x):
        # Process inputs to function
        x_pos = x.item(0)
        y_pos = x.item(1)
        z_pos = x.item(2)
        theta = x.item(3)  # Rotation around y axis (pitch)
        alpha = x.item(4)
        psi = x.item(5)  # Rotation around z axis (yaw)

        self.drawVehicle(x_pos, y_pos, z_pos, theta, alpha, psi)
        # After each function has been called, initialization is over
    


    def drawVehicle(self, x, y, z, theta, alpha, psi):
        self.ax.cla()
        cube_lim = 5

        self.ax.axes.set_xlim(-cube_lim, cube_lim)
        self.ax.axes.set_ylim(-cube_lim, cube_lim)
        self.ax.axes.set_zlim(-cube_lim, cube_lim)

        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(theta), np.sin(theta)],
            [0, -np.sin(theta), np.cos(theta)]
        ])

        R_pitch = np.array([
            [np.cos(alpha), 0, -np.sin(alpha)],
            [0, 1, 0],
            [np.sin(alpha), 0, np.cos(alpha)]
        ])

        R_yaw = np.array([
            [np.cos(psi), np.sin(psi), 0],
            [-np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        Rbw = R_roll@R_pitch@R_yaw
        
        offset = np.array([x, y, z])

        linexy = self.linexy @ Rbw + np.tile(offset, (np.shape(self.linexy)[0], 1))
        line2xy = self.line2xy @ Rbw + np.tile(offset, (np.shape(self.line2xy)[0], 1))

        rotor1xy = self.rotor1xy @ Rbw + np.tile(offset, (np.shape(self.rotor1xy)[0], 1))
        rotor2xy = self.rotor2xy @ Rbw + np.tile(offset, (np.shape(self.rotor2xy)[0], 1))
        rotor3xy = self.rotor3xy @ Rbw + np.tile(offset, (np.shape(self.rotor3xy)[0], 1))
        rotor4xy = self.rotor4xy @ Rbw + np.tile(offset, (np.shape(self.rotor4xy)[0], 1))


        # When the class is initialized, a polygon patch object will be
        # created and added to the axes. After initialization, the polygon
        # patch object will only be updated.

        self.ax.plot3D(line2xy[:, 0], line2xy[:, 1], line2xy[:, 2], "b-")
        self.ax.plot3D(linexy[:, 0], linexy[:, 1], linexy[:, 2], "b-")
        self.ax.plot3D(rotor1xy[:, 0], rotor1xy[:, 1], rotor1xy[:, 2], "r--")
        self.ax.plot3D(rotor2xy[:, 0], rotor2xy[:, 1], rotor2xy[:, 2], "r--")
        self.ax.plot3D(rotor3xy[:, 0], rotor3xy[:, 1], rotor3xy[:, 2], "r--")
        self.ax.plot3D(rotor4xy[:, 0], rotor4xy[:, 1], rotor4xy[:, 2], "r--")
        self.ax.plot3D([0, 0.25], [0, 0], [0, 0], 'g--')
        self.ax.plot3D([0, 0], [0, 0.25], [0, 0], 'g--')
        self.ax.plot3D([0, 0], [0, 0], [0, 0.25], 'g--')

