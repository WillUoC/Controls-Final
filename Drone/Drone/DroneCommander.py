from . import DroneParam as P
from .control.FullStateFeedback import FeedbackLoop
from .control.util.exceptions import StateError
from .control.DroneStates import State, TakeoffState, ClimbState, CruiseState, DescentState, LandingState, LandedState, FlightState
import numpy as np
import logging
import typing


class DroneCommander:
    def __init__(self, flight_plan: list[tuple[np.ndarray[float], str]], trajectory_tolerance: float=0.1):
        possible_cruise_modes = [
            'WAYPOINT',
            'TRAJECTORY'
        ]

        # Check if all cruise type requests are valid before compiling flight plan.
        for i in flight_plan:
            if not(i[1] in possible_cruise_modes): raise(StateError)

        # Initialize Controllers
        hcontroller = FeedbackLoop(P.Kh, P.krh, P.Kh, ki=P.kih, sample_rate=P.Ts)
        psicontroller = FeedbackLoop(P.Kpsi, P.krpsi, P.Kpsi, ki=P.kipsi, sample_rate=P.Ts)
        alphacontroller = FeedbackLoop(P.Kal, P.kral, P.Kal, ki=P.kial, sample_rate=P.Ts)
        thetacontroller = FeedbackLoop(P.Kth, P.krth, P.Kth, ki=P.kith, sample_rate=P.Ts)
        xcontroller = FeedbackLoop(P.Kx, P.krx, P.Kx, ki=P.kix, lower_limit=-P.max_angle, upper_limit=P.max_angle, sample_rate=P.Ts)
        ycontroller = FeedbackLoop(P.Ky, P.kry, P.Ky, ki=P.kiy, lower_limit=-P.max_angle, upper_limit=P.max_angle, sample_rate=P.Ts)

        self.controllers = [hcontroller, thetacontroller, alphacontroller, psicontroller, xcontroller, ycontroller]

        self.TAKEOFF_HEIGHT = 2
        self.LANDING_HEIGHT = 0.1

        self.TAKEOFF_FORCE_SCALE = 1.1
        self.LANDING_FORCE_SCALE = 0.99

        self.trajectory_tolerance = trajectory_tolerance

        state_queue = self.__generate_flight_plan(flight_plan)

        # Initialize States

        self.state_machine = StateMachine(state_queue)

    def __generate_flight_plan(self, flight_plan: list[tuple[np.ndarray[float], str]]):

        initial_cruise_height: float = flight_plan[0][0][0, 2]

        state_queue = [
            TakeoffState(self.controllers, self.TAKEOFF_FORCE_SCALE, self.TAKEOFF_HEIGHT),
            ClimbState(self.controllers, initial_cruise_height),
            DescentState(self.controllers, self.LANDING_HEIGHT),
            LandingState(self.LANDING_FORCE_SCALE),
            LandedState()
            ]


        '''
        State Queue:
        0: Takeoff (to constant height)
        1: Climb (to first cruise height)
        ... 
            add cruise states here
        ...
        -3: Descent (to constant landing height)
        -2: Landing (to 0)
        -1: Landed (terminate execution)
        '''

        for cruise_plan in flight_plan:
            cruise_points = cruise_plan[0]
            cruise_type = cruise_plan[1]

            if cruise_type == "WAYPOINT":
                for coordinates in cruise_points[:]:
                    state_queue.insert(-3, CruiseState(self.controllers, tuple(coordinates.squeeze())))
            elif cruise_type == "TRAJECTORY":
                state_queue.insert(-3, FlightState(self.controllers, cruise_points, self.trajectory_tolerance))


        return state_queue

    def update(self, states):
        forces, end_state = self.state_machine.update(states)
        forces = forces.reshape((4,))

        return(forces, end_state)

class StateMachine:
    def __init__(self, state_queue: list[State] = []):
        self.state_queue = state_queue
        self.current_state_index = 0

    def insert_state(self, state: State, index: int=-1) -> None:
        self.state_queue.insert(index, state)

    def update(self, states):
        handler = self.state_queue[self.current_state_index]
        do_switch_state, forces = handler.update(states)
        self.current_state_index += int(do_switch_state)
        
        return(forces, self.current_state_index == len(self.state_queue) - 1)
