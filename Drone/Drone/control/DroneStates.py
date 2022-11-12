import logging
from .FullStateFeedback import FeedbackLoop
from .. import DroneParam as P
from abc import ABC, abstractmethod
import numpy as np

class State(ABC):

    @abstractmethod
    def __init__(self, controllers: list[FeedbackLoop]):
        self.h_controller = controllers[0]
        self.theta_controller = controllers[1]
        self.alpha_controller = controllers[2]
        self.psi_controller = controllers[3]

    @abstractmethod
    def update(self, states) -> tuple[str, np.ndarray[float]]:
        pass

class TakeoffState(State):
    def __init__(self, controllers: list[FeedbackLoop], FORCE_SCALE, climb_height):
        self.FORCE_SCALE = FORCE_SCALE
        self.climb_height = climb_height

        self.alpha_ref = 0.0
        self.theta_ref = 0.0
        self.psi_ref = 0.0

        super().__init__(controllers)

    def update(self, states: np.ndarray[float]):
        h = states.item(2)
        hdot = states.item(8)

        xh = np.array([
            [h],
            [hdot]
        ])
        xh_r = np.array([
            [self.climb_height]
        ])

        self.h_controller.update_int(xh_r, xh)

        F = P.Fe * self.FORCE_SCALE
        tauz = self.psi_controller.update(np.array([[self.psi_ref]]), np.array([[states.item(5)],[states.item(11)]]))
        tauy = self.alpha_controller.update(np.array([[self.alpha_ref]]), np.array([[states.item(4)],[states.item(10)]]))
        taux = self.theta_controller.update(np.array([[self.theta_ref]]), np.array([[states.item(3)],[states.item(9)]]))

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if h >= self.climb_height:
            logging.info('Switching to climb...')
            return('CLIMB', Forces)
        else:
            return('TAKEOFF', Forces)

class ClimbState(State):
    def __init__(self, h_controller: FeedbackLoop, h_target: float, TOLERANCE: float=1e-2):
        self.h_ref = h_target
        self.h_controller = h_controller
        self.TOLERANCE = TOLERANCE
    
    def update(self, states: np.ndarray[float]):
        x = np.array([
            [states.item(2)],
            [states.item(8)]
        ])

        x_r = np.array([[self.h_ref]])

        F = self.h_controller.update_int(x_r, x) + P.Fe
        taux, tauy, tauz = (0.0, 0.0, 0.0)

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if x.item(0) > self.h_ref - self.TOLERANCE and x.item(0) < self.h_ref + self.TOLERANCE:
            logging.info('Switching to cruise')
            return('CRUISE', Forces)  # TODO: Change to cruise state when implemented
        else:
            return('CLIMB', Forces)


class CruiseState(State):
    def __init__(self, controllers: list[FeedbackLoop], h_target: float, TOLERANCE: float=1e-2):
        self.h_ref = h_target
        self.psi_ref = 0.0
        self.alpha_ref = 0.0
        self.theta_ref = 0.0

        super().__init__(controllers)
        self.TOLERANCE = TOLERANCE
    
    def set_theta(self, theta: float):
        self.theta_ref = theta

    def set_alpha(self, alpha: float):
        self.alpha_ref = alpha

    def set_psi(self, psi: float):
        self.psi_ref = psi

    def update(self, states: np.ndarray[float]):

        F = self.h_controller.update_int(np.array([[self.h_ref]]), np.array([[states.item(2)],[states.item(8)]]))
        tauz = self.psi_controller.update(np.array([[self.psi_ref]]), np.array([[states.item(5)],[states.item(11)]]))
        tauy = self.alpha_controller.update(np.array([[self.alpha_ref]]), np.array([[states.item(4)],[states.item(10)]]))
        taux = self.theta_controller.update(np.array([[self.theta_ref]]), np.array([[states.item(3)],[states.item(9)]]))

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if states.item(2) > self.h_ref + 10 - self.TOLERANCE and states.item(2) < self.h_ref + 10 + self.TOLERANCE:
            logging.info("Switching to descent")
            return('DESCENT', Forces)  # TODO: Change to cruise state when implemented
        else:
            return('CRUISE', Forces)

class DescentState(State):
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

        F = self.h_controller.update_int(x_r, x) + P.Fe
        taux, tauy, tauz = (0.0, 0.0, 0.0)

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if x.item(0) > self.h_ref - self.TOLERANCE and x.item(0) < self.h_ref + self.TOLERANCE:
            logging.info('Switching to landing')
            return('LANDING', Forces)  # TODO: Change to cruise state when implemented
        else:
            return('DESCENT', Forces)

class LandingState(State):
    def __init__(self, FORCE_SCALE):
        self.FORCE_SCALE = FORCE_SCALE
        self.climb_height = 0.0

    def update(self, states):
        h = states.item(2)
        hdot = states.item(8)
        F = P.Fe * self.FORCE_SCALE
        taux, tauy, tauz = (0.0, 0.0, 0.0)

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if h >= self.climb_height:
            logging.info('Landed')
            return('LANDING', Forces)
        else:
            return('LANDING', Forces)