# Copyright (c) 2024, Leap AI.
# All rights reserved.

import abc
import typing

from simulation.states.state import State


class Action(abc.ABC):
    """
    Abstract class for action.

    """

    @abc.abstractmethod
    def execute(
        self,
        object: typing.Any,
        time: float,
        states: dict[str, State] | None,
        info: dict[str, typing.Any] | None,
    ) -> None:
        """
        Execute the action.

        Args:
            object (typing.Any): The object.
            time (float): The current simulation time in seconds.
            states (dict[str, State] | None): The states needed for the action.
            info (dict[str, typing.Any] | None): Extra information for the action.

        """
        raise NotImplementedError()
