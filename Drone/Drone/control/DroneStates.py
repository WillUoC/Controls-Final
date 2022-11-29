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
        self.x_controller = controllers[4]
        self.y_controller = controllers[5]

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
    def __init__(self, controllers: list[FeedbackLoop], h_target: float, NEXT_STATE: str='CRUISE', TOLERANCE: float=1e-2):
        super().__init__(controllers)
        self.h_ref = h_target
        self.TOLERANCE = TOLERANCE
        self.NEXT_STATE = NEXT_STATE
    
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

        if np.abs(x.item(0) - self.h_ref) < self.TOLERANCE:
            logging.info('Switching to cruise')
            return(self.NEXT_STATE, Forces)  # TODO: Change to cruise state when implemented
        else:
            return('CLIMB', Forces)

class CruiseState(State):
    def __init__(self, controllers: list[FeedbackLoop], h_target: float, STATE_NAME: str='CRUISE', NEXT_STATE: str='DESCENT', TOLERANCE: float=1e-2):
        self.h_ref = h_target
        self.psi_ref = 0.0
        self.alpha_ref = 0.0
        self.theta_ref = 0.0

        self.x_ref = 0.0
        self.y_ref = 0.0

        self.flag = True

        super().__init__(controllers)
        self.TOLERANCE = TOLERANCE
        self.NEXT_STATE = NEXT_STATE
        self.STATE_NAME = STATE_NAME

    def set_x(self, x):
        self.x_ref = x

    def set_y(self, y):
        self.y_ref = y

    def set_z(self, z):
        self.h_ref = z

    def update(self, states: np.ndarray[float]):
        if self.flag:
            self.psi_ref = np.arctan2((self.y_ref - states.item(1)), (self.x_ref - states.item(0)))
            self.flag = False
            logging.info(f'Commanded Psi: {self.psi_ref*180/np.pi}')
            logging.info(f'Drone Integrator: {self.x_controller.integrator}, {self.y_controller.integrator}')

        psi_rot = np.array([
            [ np.cos(states.item(5)), np.sin(states.item(5)), 0.0],
            [-np.sin(states.item(5)), np.cos(states.item(5)), 0.0],
            [0.0, 0.0, 1.0]
        ])

        x_ref = psi_rot @ np.array([
            [self.x_ref],
            [self.y_ref],
            [0.0]
        ])


        pos = psi_rot @ np.array([
            [states.item(0)],
            [states.item(1)],
            [0.0]
        ])


        posdot = psi_rot @ np.array([
            [states.item(6)],
            [states.item(7)],
            [0.0]
        ])

        angles = np.array([
            [states.item(3)],
            [states.item(4)],
            [states.item(5)]
        ])

        angledots = np.array([
            [states.item(9)],
            [states.item(10)],
            [states.item(11)]
        ])

        F = self.h_controller.update_int(np.array([[self.h_ref]]), np.array([[states.item(2)],[states.item(8)]]))

        if np.abs(states.item(5) - self.psi_ref) < np.pi/4:
            self.alpha_ref = self.x_controller.update_int(np.array([[x_ref.item(0)]]), np.array([[pos.item(0)], [posdot.item(0)]]))
            self.theta_ref = self.y_controller.update_int(np.array([[x_ref.item(1)]]), np.array([[pos.item(1)], [posdot.item(1)]]))

        tauz = self.psi_controller.update_int(np.array([[self.psi_ref]]), np.array([[angles.item(2)],[angledots.item(2)]]))
        tauy = self.alpha_controller.update(np.array([[self.alpha_ref]]), np.array([[angles.item(1)],[angledots.item(1)]]))
        taux = self.theta_controller.update(np.array([[self.theta_ref]]), np.array([[angles.item(0)],[angledots.item(0)]]))

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if np.abs(states.item(2) - self.h_ref) < self.TOLERANCE and np.abs(states.item(0) - self.x_ref) < self.TOLERANCE and np.abs(states.item(1) - self.y_ref) < self.TOLERANCE:
            logging.info(f"Switching to {self.NEXT_STATE}")
            # self.x_controller.reset_integrator()
            # self.y_controller.reset_integrator()
            # self.psi_controller.reset_integrator()
            return(self.NEXT_STATE, Forces)
        else:
            return(self.STATE_NAME, Forces)

class DescentState(State):
    def __init__(self, controllers: list[FeedbackLoop], h_target: float=0.0, TOLERANCE: float=1e-3):
        super().__init__(controllers)
        
        self.h_ref = h_target
        self.x_ref = 0.0
        self.y_ref = 0.0
        
        self.psi_ref = 0.0

        self.TOLERANCE = TOLERANCE

    def set_x(self, x):
        self.x_ref = x

    def set_y(self, y):
        self.y_ref = y

    def set_z(self, z):
        self.h_ref = z
    
    def update(self, states: np.ndarray[float]):
        x = np.array([
            [states.item(2)],
            [states.item(8)]
        ])

        x_r = np.array([[self.h_ref]])

        F = self.h_controller.update_int(x_r, x) + P.Fe

        self.alpha_ref = self.x_controller.update_int(np.array([[self.x_ref]]), np.array([[states.item(0)], [states.item(6)]]))
        self.theta_ref = self.y_controller.update_int(np.array([[self.y_ref]]), np.array([[states.item(1)], [states.item(7)]]))

        tauz = self.psi_controller.update(np.array([[self.psi_ref]]), np.array([[states.item(5)],[states.item(11)]]))
        tauy = self.alpha_controller.update(np.array([[self.alpha_ref]]), np.array([[states.item(4)],[states.item(10)]]))
        taux = self.theta_controller.update(np.array([[self.theta_ref]]), np.array([[states.item(3)],[states.item(9)]]))

        Forces = np.array([
            [F],
            [taux],
            [tauy],
            [tauz]
        ])

        if np.abs(states.item(2) - self.h_ref) < self.TOLERANCE and np.abs(states.item(0) - self.x_ref) < self.TOLERANCE and np.abs(states.item(1) - self.y_ref) < self.TOLERANCE:            
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
            return('LANDED', Forces)
        else:
            return('LANDING', Forces)

class LandedState(State):
    def __init__(self):
        return None

    def update(self, states):
        forces = np.array([
            [0],
            [0],
            [0],
            [0]
        ])

        return('LANDED', forces)
