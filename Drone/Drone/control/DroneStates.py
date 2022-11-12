import logging
from .FullStateFeedback import FeedbackLoop
from .. import DroneParam as P
from abc import ABC, abstractmethod
import numpy as np

class State(ABC):

    @abstractmethod
    def __init__(self, controllers, targets):
        pass

    @abstractmethod
    def update(self):
        pass

class TakeoffState(State):
    def __init__(self, h_dot_controller: FeedbackLoop, hdot_ref, climb_height):
        self.hdot_ref = hdot_ref
        self.h_dot_controller = h_dot_controller
        self.climb_height = climb_height

    def update(self, states):
        h = states.item(2)
        hdot = states.item(8)
        F = self.h_dot_controller.update(self.hdot_ref, hdot) + P.Fe
        taux, tauy, tauz = (0.0, 0.0, 0.0)

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if h >= self.climb_height:
            return('CLIMB', Forces)
        else:
            return('TAKEOFF', Forces)

class ClimbState(State):
    def __init__(self, h_controller: FeedbackLoop, h_target: float, TOLERANCE: float=1e-2):
        self.h_ref = h_target
        self.h_controller = h_controller
        self.TOLERANCE = TOLERANCE
    
    def update(self, states):
        x = np.array([
            [states.item(2)],
            [states.item(8)]
        ])

        x_r = np.array([[self.h_ref]])

        F = self.h_controller.update_int(x_r, x)
        taux, tauy, tauz = (0.0, 0.0, 0.0)

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if x.item(0) > self.h_ref - self.TOLERANCE and x.item(0) < self.h_ref + self.TOLERANCE:
            return('CRUISE', Forces)  # TODO: Change to cruise state when implemented
        else:
            return('CLIMB', Forces)


class CruiseState(State):
    def __init__(self, h_controller: FeedbackLoop, h_target: float, TOLERANCE: float=1e-2):
        self.h_ref = h_target
        self.h_controller = h_controller
        self.TOLERANCE = TOLERANCE
    
    def update(self, states):
        x = np.array([
            [states.item(2)],
            [states.item(8)]
        ])

        x_r = np.array([[self.h_ref]])

        F = self.h_controller.update_int(x_r, x)
        taux, tauy, tauz = (0.0, 0.0, 0.0)

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if x.item(0) > self.h_ref - self.TOLERANCE and x.item(0) < self.h_ref + self.TOLERANCE:
            return('CRUISE', Forces)  # TODO: Change to cruise state when implemented
        else:
            return('CRUISE', Forces)