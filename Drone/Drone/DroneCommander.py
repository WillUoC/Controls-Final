from . import DroneParam as P
from .control.FullStateFeedback import FeedbackLoop
from .control.util.exceptions import InitializationError
from .control.DroneStates import State, TakeoffState, ClimbState, CruiseState, DescentState, LandingState, LandedState
import numpy as np
import logging
import typing


class DroneCommander:
    def __init__(self):
        # Initialize Controllers
        hcontroller = FeedbackLoop(P.Kh, P.krh, P.Kh, ki=P.kih, sample_rate=P.Ts)
        psicontroller = FeedbackLoop(P.Kpsi, P.krpsi, P.Kpsi, ki=P.kipsi, sample_rate=P.Ts)
        alphacontroller = FeedbackLoop(P.Kal, P.kral, P.Kal, ki=P.kial, sample_rate=P.Ts)
        thetacontroller = FeedbackLoop(P.Kth, P.krth, P.Kth, ki=P.kith, sample_rate=P.Ts)
        xcontroller = FeedbackLoop(P.Kx, P.krx, P.Kx, ki=P.kix, lower_limit=-P.max_angle, upper_limit=P.max_angle, sample_rate=P.Ts)
        ycontroller = FeedbackLoop(P.Ky, P.kry, P.Ky, ki=P.kiy, lower_limit=-P.max_angle, upper_limit=P.max_angle, sample_rate=P.Ts)

        controllers = [hcontroller, thetacontroller, alphacontroller, psicontroller, xcontroller, ycontroller]

        # Initialize States
        takeoff_state = TakeoffState(controllers, 1.1, 2)
        climb_state = ClimbState(hcontroller, 8)
        cruise_state_1 = CruiseState(controllers, 8, NEXT_STATE='CRUISE2')
        cruise_state_2 = CruiseState(controllers, 8, STATE_NAME='CRUISE2', NEXT_STATE='CRUISE3')
        cruise_state_3 = CruiseState(controllers, 8, STATE_NAME='CRUISE3')
        descent_state = DescentState(controllers, 0.1)
        landing_state = LandingState(0.99)
        landed_state = LandedState()

        cruise_state_1.set_x(-4)
        cruise_state_1.set_y(-4)

        cruise_state_2.set_x(4)
        cruise_state_2.set_y(4)

        cruise_state_3.set_x(0)
        cruise_state_3.set_y(0)

        self.state_machine = StateMachine()
        self.state_machine.add_state('TAKEOFF', takeoff_state)
        self.state_machine.add_state('CLIMB', climb_state)
        self.state_machine.add_state('CRUISE', cruise_state_1)
        self.state_machine.add_state('CRUISE2', cruise_state_2)
        self.state_machine.add_state('CRUISE3', cruise_state_3)
        self.state_machine.add_state('DESCENT', descent_state)
        self.state_machine.add_state('LANDING', landing_state)
        self.state_machine.add_state('LANDED', landed_state, True)
        self.state_machine.set_start('TAKEOFF')


    def update(self, states):
        forces, end_state = self.state_machine.update(states)
        forces = forces.reshape((4,))

        return(forces, end_state)

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
