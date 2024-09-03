# Copyright (c) 2024, Leap AI.
# All rights reserved.

import enum
import threading
import typing
import queue

import rclpy
import rclpy.node
import rclpy.executors
import rclpy.qos
import rclpy.callback_groups
import rosidl_runtime_py.utilities as rosidl_utils

from .communication_manager import CommunicationManager


class Ros2MsgType(enum.Enum):
    """
    Enum class for ROS2 message types.

    """

    TOPIC = 1
    SERVICE = 2
    ACTION = 3


class Ros2Manager(CommunicationManager):
    """
    The implementation of a communication manager that handles sending and receiving messages through ROS2 communication channel.

    Note:
        Callback groups potentially need to be saved as class attributes to prevent garbage collection.
        If communication fails, this will be a first place to look.
    """

    def __init__(self, node: rclpy.node.Node = None):
        """
        Initializes the ROS2 communication manager.

        """
        # Setup ROS2 node and executor
        if not rclpy.ok():
            rclpy.init()
        if node is None:
            self._node = rclpy.node.Node("isaac_sim_bridge")
        else:
            self._node = node
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)

        # Create placeholders
        self._publishers = {}
        """dict[str, rclpy.publisher.Publisher]: The publishers for each topic."""
        self._subscribers = {}
        """dict[str, rclpy.subscription.Subscription]: The subscribers for each topic."""
        self._subscribers_data_queue = {}
        """dict[str, list[queue.Queue, threading.Lock]]: The data queue and lock for each subscriber."""
        self._service_clients = {}
        """dict[str, rclpy.client.Client]: The service clients for each service."""
        self._service_data_queue = {}
        """dict[str, list[queue.Queue, threading.Lock]]: The data queue and lock for each service."""
        self._action_clients = {}
        """dict[str, rclpy.action.Client]: The action clients for each action."""
        self._action_data_queue = {}
        """dict[str, list[queue.Queue, threading.Lock]]: The data queue and lock for each action."""

        self._spining_thread = threading.Thread(target=self._spin_node)

    def open(self, info: typing.Any = None) -> bool:
        # Start spining thread
        if not self._spining_thread.is_alive():
            self._spining_thread.start()
        return True

    def close(self) -> bool:
        if self._spining_thread.is_alive():
            self._executor.shutdown()
            self._spining_thread.join()
        return True

    def send(self, msg: typing.Any, msg_name: str, msg_type: enum.Enum) -> bool:
        if not self._spining_thread.is_alive():
            return False
        match msg_type:
            case Ros2MsgType.TOPIC:
                return self._publish_topic(msg, msg_name)
            case Ros2MsgType.SERVICE:
                return self._call_service(msg, msg_name)
            case Ros2MsgType.ACTION:
                return self._call_action(msg, msg_name)
            case _:
                raise ValueError("Invalid message type.")

    def receive(self, msg_name: str, msg_type: enum.Enum) -> typing.Any:
        if not self._spining_thread.is_alive():
            return False
        match msg_type:
            case Ros2MsgType.TOPIC:
                return self._receive_topic(msg_name)
            case Ros2MsgType.SERVICE:
                return self._receive_service(msg_name)
            case Ros2MsgType.ACTION:
                return self._receive_action(msg_name)
            case _:
                raise ValueError("Invalid message type.")

    def _spin_node(self) -> None:
        """
        Spins the ROS2 node.

        """
        self._executor.spin()

    def _publish_topic(self, msg: typing.Any, topic_name: str) -> bool:
        """
        Sends a message over a ROS2 topic.

        Args:
            msg (Any): The message to be sent.
            msg_name (str): The name of the topic.

        Returns:
            bool: Always returns True.

        """
        if self._publishers.get(topic_name) is None:
            cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            self._publishers[topic_name] = self._node.create_publisher(
                msg_type=msg.__class__,
                topic=topic_name,
                qos_profile=rclpy.qos.qos_profile_default,
                callback_group=cb_group,
            )

        self._publishers[topic_name].publish(msg)
        return True

    def _call_service(self, request: typing.Any, service_name: str) -> bool:
        """
        Calls a ROS2 service and save the response in the data queue. The size of the data queue is 1
        which means only the latest response will be stored and the previous response will be overwritten.

        Args:
            request (Any): The service message to be sent.
            service_name (str): The name of the service.

        Returns:
            bool: True if the service is called successfully, False otherwise.

        """
        if self._service_clients.get(service_name) is None:
            cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            srv_type_name = request.__class__.__module__.split(".")[:-1]
            srv_type_name.append(request.__class__.__name__.split("_")[0])
            self._service_clients[service_name] = self._node.create_client(
                srv_type=rosidl_utils.get_service("/".join(srv_type_name)),
                srv_name=service_name,
                callback_group=cb_group,
            )

        if not self._service_clients[service_name].service_is_ready():
            return False

        # TODO: Should we call asynchronously?
        response = self._service_clients[service_name].call(request)

        if self._service_data_queue.get(service_name) is None:
            self._service_data_queue[service_name] = [
                queue.Queue(maxsize=1),
                threading.Lock(),
            ]
        self._add_data_to_queue(self._service_data_queue[service_name], response)
        return True

    def _call_action(self, action: typing.Any, action_name: str) -> bool:
        raise NotImplementedError()

    def _receive_topic(self, topic_name: str) -> typing.Any:
        """
        Receives a ROS2 topic message from the data queue. The size of the data queue is 1
        which means only the latest message will be stored and the previous message will be overwritten.

        Note:
            This method will create a subscriber if it does not exist.
            So even if the message is published before the initial receive call, it will not be received.

        Args:
            topic_name (str): The name of the topic.

        Returns:
            Any: The received message.

        """
        if self._subscribers.get(topic_name) is None:
            # Find the type of the topic
            topic_list = self._node.get_topic_names_and_types()
            topic_types = [
                types[0]
                for name, types in topic_list
                if name == topic_name or name == "/" + topic_name
            ]
            if len(topic_types) == 0:
                return None

            # Create callback and subscriber
            self._subscribers_data_queue[topic_name] = [
                queue.Queue(maxsize=1),
                threading.Lock(),
            ]
            callback = lambda msg: self._add_data_to_queue(
                self._subscribers_data_queue[topic_name], msg
            )
            cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            self._subscribers[topic_name] = self._node.create_subscription(
                msg_type=rosidl_utils.get_message(
                    topic_types[0]  # assume only one type for the topic
                ),
                topic=topic_name,
                callback=callback,
                qos_profile=rclpy.qos.qos_profile_default,
                callback_group=cb_group,
            )

        return self._get_data_from_queue(self._subscribers_data_queue[topic_name])

    def _receive_service(self, service_name: str) -> typing.Any:
        """
        Receives a ROS2 service reponse from the data queue.

        Args:
            service_name (str): The name of the service.

        Returns:
            Any: The received message.

        """
        if self._service_data_queue.get(service_name) is None:
            self._service_data_queue[service_name] = [
                queue.Queue(maxsize=1),
                threading.Lock(),
            ]

        return self._get_data_from_queue(self._service_data_queue[service_name])

    def _receive_action(self, action_name: str) -> typing.Any:
        raise NotImplementedError()

    def _add_data_to_queue(
        self, data_queue: list[queue.Queue, threading.Lock], data: typing.Any
    ) -> None:
        """
        Adds data to a queue.

        Args:
            queue (Queue): The queue where data should be added.
            data (Any): The data to be added.

        """
        queue, lock = data_queue
        with lock:
            if queue.full():
                queue.get()
            queue.put(data)

    def _get_data_from_queue(
        self, data_queue: list[queue.Queue, threading.Lock]
    ) -> typing.Any:
        """
        Gets data from a queue.

        Args:
            queue (Queue): The queue where data should be retrieved.

        Returns:
            Any: The data retrieved from the queue.

        """
        queue, lock = data_queue
        with lock:
            if queue.empty():
                return None
            return queue.get()
