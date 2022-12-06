import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing

class dataPlotter:
    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 3    # Number of subplot rows
        self.num_cols = 3    # Number of subplot columns

        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.xpos_history = []
        self.ypos_history = []
        self.zpos_history = []
        self.theta_history = []  # altitude h
        self.alpha_history = []
        self.psi_history = []  # angle theta
        self.thrust1_history = []  # control force
        self.thrust2_history = []  # control force
        self.thrust3_history = []  # control force
        self.thrust4_history = []  # control force
        self.zref_history = []
        self.tlimit_history = []

        # create a handle for every subplot.
        self.handle = []
        self.handle.append(myPlot(self.ax[0,0], ylabel='Position (x)', title='Position Data'))
        self.handle.append(myPlot(self.ax[1,0], ylabel='Position (y)'))
        self.handle.append(myPlot(self.ax[2,0], ylabel='Position (z)'))
        self.handle.append(myPlot(self.ax[0,1], ylabel='theta', title='Angle Data'))
        self.handle.append(myPlot(self.ax[1,1], ylabel='alpha'))
        self.handle.append(myPlot(self.ax[2,1], ylabel='psi'))
        self.handle.append(myPlot(self.ax[1,2], ylabel='Thrusts'))
#        self.handle = []
#        self.handle.append(myPlot(self.ax[0], ylabel='Positions (m)', title='Drone Data'))
#       self.handle.append(myPlot(self.ax[1], ylabel='Angles (deg)'))
#        self.handle.append(myPlot(self.ax[2], ylabel='Thrusts (v)'))

    def update(self, t, states, u, tlimit, zref):
        '''
            Add to the time and data histories, and update the plots.
        '''
        # update the time history of all plot variables
        self.time_history.append(t)
        self.xpos_history.append(states.item(0))
        self.ypos_history.append(states.item(1))
        self.zpos_history.append(states.item(2))
        self.theta_history.append(states.item(3) * 180/np.pi)
        self.alpha_history.append(states.item(4) * 180/np.pi)
        self.psi_history.append(states.item(5) * 180/np.pi)
        self.thrust1_history.append(u.item(0))
        self.thrust2_history.append(u.item(1))
        self.thrust3_history.append(u.item(2))
        self.thrust4_history.append(u.item(3))

        self.zref_history.append(zref)
        self.tlimit_history.append(tlimit)

        # update the plots with associated histories
        self.handle[0].update(self.time_history, [self.xpos_history]) #hi will
        self.handle[1].update(self.time_history, [self.ypos_history]) #add the commanded positions if you can!
        self.handle[2].update(self.time_history, [self.zpos_history])
        self.handle[3].update(self.time_history, [self.alpha_history]) #and angles? is this commanded?
        self.handle[4].update(self.time_history, [self.theta_history])
        self.handle[5].update(self.time_history, [self.psi_history])
        self.handle[6].update(self.time_history, [self.thrust1_history, self.thrust2_history, self.thrust3_history, self.thrust4_history])
   #     self.handle[0].update(self.time_history, [self.xpos_history, self.ypos_history, self.zpos_history, self.zref_history])
   #     self.handle[1].update(self.time_history, [self.theta_history, self.alpha_history, self.psi_history])
   #     self.handle[2].update(self.time_history, [self.thrust1_history, self.thrust2_history, self.thrust3_history, self.thrust4_history])


class myPlot:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'y', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '-', '-', '--', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data)):
                # Instantiate line object and add it to the axes
                self.line.append(Line2D(time,
                                        data[i],
                                        color=self.colors[np.mod(i, len(self.colors) - 1)],
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=self.legend if self.legend != None else None))
                self.ax.add_line(self.line[i])
            self.init = False
            # add legend if one is specified
            if self.legend != None:
                plt.legend(handles=self.line)
        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line)):
                self.line[i].set_xdata(time)
                self.line[i].set_ydata(data[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()
