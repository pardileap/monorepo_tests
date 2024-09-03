# Copyright (c) 2024, Leap AI.
# All rights reserved.

import typing

from .sim_object import SimObject
from simulation.states.state import State
from simulation.actions.action import Action


class SimObjectBase(SimObject):
    """
    Base class for simulation object.
    It implements some of the abstract methods in the SimObject class.

    """

    def __init__(
        self,
        name: str,
        object: typing.Any,
        states: dict[str, State] | None = None,
        actions: list[Action] | None = None,
    ):
        """
        Initializes the simulation object base.

        Args:
            name (str): The name of the object.
            object (typing.Any): The object.
            states (dict[str, State] | None): The states of the object.
            actions (list[Action] | None): The actions of the object.

        """
        self._name = name
        """str: The name of the object."""
        self._object = object
        """typing.Any: The object."""
        self._states = states
        """dict[str, State] | None: The states of the object."""
        self._actions = actions
        """list[Action] | None: The actions of the object."""

    @property
    def name(self) -> str:
        return self._name

    @property
    def object(self) -> typing.Any:
        return self._object

    def execute_action(
        self,
        time: float,
        action: Action,
        info: dict[str, typing.Any] | None = None,
    ) -> None:
        action.execute(self._object, time, self._states, info)

    def execute_registered_actions(
        self,
        time: float,
        info: dict[str, typing.Any] | None = None,
    ) -> None:
        if self._actions is not None:
            for action in self._actions:
                action.execute(self._object, time, self._states, info)

    def get_state(self, name: str) -> typing.Any:
        if self._states is None or self._states.get(name) is None:
            return None
        return self._states[name].get(self._object)
