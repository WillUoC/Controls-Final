from . import DroneParam as P
from .control.FullStateFeedback import FeedbackLoop
from .control.util.exceptions import InitializationError
from .control.DroneStates import State, TakeoffState, ClimbState, CruiseState, DescentState, LandingState
import numpy as np
import logging
import typing


class DroneCommander:
    def __init__(self):
        # Initialize Controllers
        hcontroller = FeedbackLoop(P.Kh, 0, ki=P.kih, sample_rate=P.Ts)
        psicontroller = FeedbackLoop(P.Kpsi, 0, ki=P.kipsi, sample_rate=P.Ts)
        alphacontroller = FeedbackLoop(P.Kal, 0, ki=P.kial, sample_rate=P.Ts)
        thetacontroller = FeedbackLoop(P.Kth, 0, ki=P.kith, sample_rate=P.Ts)

        controllers = [hcontroller, thetacontroller, alphacontroller, psicontroller]

        # Initialize States
        takeoff_state = TakeoffState(hcontroller, 1.1, 2)
        climb_state = ClimbState(hcontroller, 3)
        cruise_state = CruiseState(controllers, 10)
        descent_state = DescentState(hcontroller, 0.5)
        landing_state = LandingState(0.99)

        cruise_state.set_psi(np.pi)

        self.state_machine = StateMachine()
        self.state_machine.add_state('TAKEOFF', takeoff_state)
        self.state_machine.add_state('CLIMB', climb_state)
        self.state_machine.add_state('CRUISE', cruise_state)
        self.state_machine.add_state('DESCENT', descent_state)
        self.state_machine.add_state('LANDING', landing_state)

        self.state_machine.set_start('TAKEOFF')


    def update(self, states):
        forces, end_state = self.state_machine.update(states)
        forces = forces.reshape((4, 1))

        return(forces)

class StateMachine:
    def __init__(self):
        self.handlers: typing.Dict[str, State] = {}
        self.START_STATE = None
        self.end_states = []
        self.current_state = None

    def add_state(self, name: str, handler: State, end_state: bool=False) -> None:
        name = name.upper()
        self.handlers[name] = handler

        if end_state:
            self.end_states.append(name)
    
    def set_start(self, name: str) -> None:
        self.current_state = name.upper()
    
    def update(self, states):
        if self.current_state is None:
            raise InitializationError("NO INITIAL STATE SET")

        handler = self.handlers[self.current_state]
        new_state, forces = handler.update(states)
        self.current_state = new_state
        
        return(forces, new_state in self.end_states)
