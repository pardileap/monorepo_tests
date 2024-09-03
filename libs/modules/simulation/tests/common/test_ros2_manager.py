# Copyright (c) 2024, Leap AI.
# All rights reserved.

import threading
import time
import enum
import pytest

import rclpy
import rclpy.node
import rclpy.executors
import std_msgs.msg
import std_srvs.srv

from common.communication_managers.ros2_manager import Ros2Manager, Ros2MsgType

"""
TODO: Mock the ROS2 node to test the ROS2 manager more effectively.

"""


@pytest.fixture
def ros2_manager():
    """
    Fixture for creating a ROS2 manager object and opening it.

    """
    manager = Ros2Manager()
    manager.open()
    yield manager
    manager.close()


@pytest.fixture
def ros2_node():
    """
    Fixture for creating a ROS2 node and spinning it in a separate thread.

    """
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.node.Node("test_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    yield node
    executor.shutdown()
    thread.join()


@pytest.fixture
def ros2_node_with_publisher_and_msg(ros2_node):
    """
    Fixture for creating a ROS2 node with a publisher and a message.

    """
    pub = ros2_node.create_publisher(std_msgs.msg.Int8, "test_topic", 10)
    msg = std_msgs.msg.Int8()
    msg.data = 42
    return ros2_node, pub, msg


@pytest.fixture
def ros2_node_with_subscriber_and_callback(ros2_node):
    """
    Fixture for creating a ROS2 node with a subscriber and a callback.

    """

    def callback(msg):
        callback.counter += 1

    callback.counter = 0
    sub = ros2_node.create_subscription(std_msgs.msg.Int8, "test_topic", callback, 10)
    return ros2_node, sub, callback


@pytest.fixture
def ros2_node_with_service_and_callback(ros2_node):
    """
    Fixture for creating a ROS2 node with a service and a callback.

    """

    def callback(request, response):
        callback.counter += 1
        response.message = "success"
        return response

    callback.counter = 0
    service = ros2_node.create_service(std_srvs.srv.Trigger, "test_service", callback)
    return ros2_node, service, callback


def test_open():
    """
    Test the open method of the ROS2 manager multiple times.

    """
    ros2_manager = Ros2Manager()
    assert ros2_manager.open()
    assert ros2_manager.open()
    ros2_manager.close()


def test_close():
    """
    Test the close method of the ROS2 manager multiple times.

    """
    ros2_manager = Ros2Manager()
    assert ros2_manager.close()
    assert ros2_manager.close()


def test_send_or_receive_without_open(ros2_manager):
    """
    Test the send and receive methods of the ROS2 manager without opening it.

    """
    ros2_manager = Ros2Manager()
    assert not ros2_manager.send(std_msgs.msg.Int8(), "test_topic", Ros2MsgType.TOPIC)
    assert not ros2_manager.receive("test_topic", Ros2MsgType.TOPIC)


def test_invalid_type(ros2_manager):
    """
    Test the send and receive methods of the ROS2 manager with an invalid message type.

    """

    class InvalidType(enum.Enum):
        INVALID = 1

    with pytest.raises(ValueError):
        ros2_manager.send(std_msgs.msg.Int8(), "test_topic", InvalidType.INVALID)
    with pytest.raises(ValueError):
        ros2_manager.receive("test_topic", InvalidType.INVALID)


def test_send_topic(ros2_manager, ros2_node_with_subscriber_and_callback):
    """
    Test the send method of the ROS2 manager for topics.

    """
    _, _, callback = ros2_node_with_subscriber_and_callback

    for _ in range(3):
        ros2_manager.send(std_msgs.msg.Int8(), "test_topic", Ros2MsgType.TOPIC)
    time.sleep(1)  # Wait for the message to be received

    assert callback.counter == 3


def test_recieve_topic(ros2_manager, ros2_node_with_publisher_and_msg):
    """
    Test the receive method of the ROS2 manager for topics.

    """
    _, pub, msg = ros2_node_with_publisher_and_msg
    ros2_manager.receive("test_topic", Ros2MsgType.TOPIC)
    pub.publish(msg)
    time.sleep(1)  # Wait for the message to be received
    recieved_msg = ros2_manager.receive("test_topic", Ros2MsgType.TOPIC)
    recieved_msg_empty = ros2_manager.receive("test_topic", Ros2MsgType.TOPIC)

    assert recieved_msg.data == 42
    assert recieved_msg_empty == None


def test_send_service(ros2_manager, ros2_node_with_service_and_callback):
    """
    Test the send method of the ROS2 manager for services.

    """
    _, _, callback = ros2_node_with_service_and_callback
    ros2_manager.send(
        std_srvs.srv.Trigger.Request(), "test_service", Ros2MsgType.SERVICE
    )

    assert callback.counter == 1


def test_receive_service_fail(ros2_manager, ros2_node_with_service_and_callback):
    """
    Test the receive method of the ROS2 manager for services when receive call is made before the service is called.

    """
    _, _, callback = ros2_node_with_service_and_callback
    response = ros2_manager.receive("test_service", Ros2MsgType.SERVICE)

    assert callback.counter == 0
    assert response == None


def test_receive_service_success(ros2_manager, ros2_node_with_service_and_callback):
    """
    Test the receive method of the ROS2 manager for services.

    """
    _, _, callback = ros2_node_with_service_and_callback
    ros2_manager.send(
        std_srvs.srv.Trigger.Request(), "test_service", Ros2MsgType.SERVICE
    )
    response = ros2_manager.receive("test_service", Ros2MsgType.SERVICE)
    response_empty = ros2_manager.receive("test_service", Ros2MsgType.SERVICE)

    assert callback.counter == 1
    assert response.message == "success"
    assert response_empty == None


def test_send_action(ros2_manager):
    """
    Test the send method of the ROS2 manager for actions.

    """
    with pytest.raises(NotImplementedError):
        ros2_manager.send(None, "test_action", Ros2MsgType.ACTION)


def test_receive_action(ros2_manager):
    """
    Test the receive method of the ROS2 manager for actions.

    """
    with pytest.raises(NotImplementedError):
        ros2_manager.receive("test_action", Ros2MsgType.ACTION)
