# Copyright (c) 2024, Leap AI.
# All rights reserved.

import typing
import abc

from simulation.states.state import State
from .action import Action


class PeriodicAction(Action):
    """
    Base class for periodic action.

    """

    def __init__(self, interval: float):
        """
        Initializes the periodic action.

        """
        self._interval = interval
        self._time_last_executed = 0.0

    def execute(
        self,
        object: typing.Any,
        time: float,
        states: dict[str, State] | None = None,
        info: dict[str, typing.Any] | None = None,
    ) -> None:
        if time - self._time_last_executed >= self._interval:
            self._time_last_executed = time
            self._execute(object, time, states, info)

    @abc.abstractmethod
    def _execute(
        self,
        object: typing.Any,
        time: float,
        states: dict[str, State] | None,
        info: dict[str, typing.Any] | None,
    ) -> None:
        """
        Execute the periodic action.

        """
        raise NotImplementedError()
