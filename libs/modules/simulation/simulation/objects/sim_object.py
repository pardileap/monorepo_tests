# Copyright (c) 2024, Leap AI.
# All rights reserved.

import abc
import typing

from simulation.actions.action import Action


class SimObject(abc.ABC):
    """
    Abstract class for simulation object.

    """

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """
        Get the name of the simulation object.

        Returns:
            str: The name of the simulation object.

        """
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def object(self) -> typing.Any:
        """
        Get the object.

        Returns:
            typing.Any: The object.

        """
        raise NotImplementedError

    @abc.abstractmethod
    def execute_action(
        self,
        time: float,
        action: Action,
        info: dict[str, typing.Any] | None,
    ) -> None:
        """
        Execute the given actions.

        Args:
            time (float): The current simulation time in seconds.
            action (Action): The action to execute.
            info (dict[str, typing.Any] or None): Extra information for the actions.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def execute_registered_actions(
        self,
        time: float,
        info: dict[str, typing.Any] | None,
    ) -> None:
        """
        Execute the pre-registered actions.

        Args:
            time (float): The current simulation time.
            info (dict[str, typing.Any] or None): Extra information for the action execution.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def post_reset(self, info: dict[str, typing.Any] | None) -> None:
        """
        Reset the simulation object after the simulation is reset.

        Args:
            info (dict[str, typing.Any] or None): Extra information for the reset.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def get_state(self, name: str) -> typing.Any:
        """
        Get the state of the simulation object.

        Args:
            name (str): The name of the state.

        Returns:
            Any: The state of the simulation object, or None if the state does not exist.

        """
        raise NotImplementedError()
