# Copyright (c) 2024, Leap AI.
# All rights reserved.

import abc
import typing


class State(abc.ABC):
    """
    Abstract class for State.

    """

    @abc.abstractmethod
    def get(self, object: typing.Any) -> typing.Any:
        """
        Get the state of the object.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def update(self, object: typing.Any, data: typing.Any = None) -> None:
        """
        Update the state of the object.

        """
        raise NotImplementedError()