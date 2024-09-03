# Copyright (c) 2024, Leap AI.
# All rights reserved.

import abc
import enum
import typing


class CommunicationManager(abc.ABC):
    """
    Abstract class for communication manager.

    """

    @abc.abstractmethod
    def open(self, info: typing.Any) -> bool:
        """
        Opens the communication channel.

        Returns:
            bool: True if the communication channel is opened successfully, False otherwise.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def close(self) -> bool:
        """
        Closes the communication channel.

        Returns:
            bool: True if the communication channel is closed successfully, False otherwise.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def send(self, msg: typing.Any, msg_name: str, msg_type: enum.Enum) -> bool:
        """
        Sends a message over the communication channel.

        Args:
            msg (Any): The message to be sent.
            msg_name (str): The name of the message.
            msg_type (Enum): The type of the message.

        Returns:
            bool: True if the message is sent successfully, False otherwise.

        """
        raise NotImplementedError()

    @abc.abstractmethod
    def receive(self, msg_name: str, msg_type: enum.Enum) -> typing.Any:
        """
        Receives a message from the communication channel.

        Args:
            msg_name (str): The name of the message.
            msg_type (Enum): The type of the message.

        Returns:
            Any: The received message. If no message is received, returns None.

        """
        raise NotImplementedError()
