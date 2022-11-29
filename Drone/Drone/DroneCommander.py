from . import DroneParam as P
from .control.FullStateFeedback import FeedbackLoop
from .control.util.exceptions import InitializationError
from .control.DroneStates import State, TakeoffState, ClimbState, CruiseState, DescentState, LandingState, LandedState
import numpy as np
import logging
import typing


class DroneCommander:
    def __init__(self, flight_plan_points):
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
        climb_state = ClimbState(controllers, 8, 'AUTOCRUISE 0')

        cruise_states, cruise_names = self.__generate_flight_plan(flight_plan_points, controllers)

        descent_state = DescentState(controllers, 0.1)
        landing_state = LandingState(0.99)
        landed_state = LandedState()

        self.state_machine = StateMachine()
        self.state_machine.add_state('TAKEOFF', takeoff_state)
        self.state_machine.add_state('CLIMB', climb_state)

        for ind, i in enumerate(cruise_states):
            self.state_machine.add_state(cruise_names[ind], i)

        self.state_machine.add_state('DESCENT', descent_state)
        self.state_machine.add_state('LANDING', landing_state)
        self.state_machine.add_state('LANDED', landed_state, True)

        self.state_machine.set_start('TAKEOFF')

    def __generate_flight_plan(self, points: np.ndarray[float], controllers: list[FeedbackLoop], state_after: str='DESCENT'):
        states = []
        state_names = []
        for ind, point in enumerate(points):
            state_name = f'AUTOCRUISE {ind}'
            if ind == len(points) - 1:
                next_state = state_after
            else:
                next_state = f'AUTOCRUISE {ind + 1}'

            new_state = CruiseState(controllers, 0, state_name, next_state)

            new_state.set_x(point[0])
            new_state.set_y(point[1])
            new_state.set_z(point[2])

            states.append(new_state)
            state_names.append(state_name)
        return(states, state_names)


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
