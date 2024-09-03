# Copyright (c) 2024, Leap AI.
# All rights reserved.

import pytest

from simulation.actions.action import Action
from simulation.states.state import State
from simulation.objects.sim_object_base import SimObjectBase


class SimObjectTest(SimObjectBase):
    def post_reset(self, info=None) -> None:
        pass


class ActionTest(Action):
    def __init__(self):
        self.count = 0

    def execute(self, object, time, states, info):
        self.count += 1


class StateTest(State):
    def __init__(self):
        self.data = 42

    def get(self, object):
        return self.data

    def update(self, object, data):
        pass


@pytest.fixture
def sim_object():
    actions = [ActionTest()]
    states = {"test": StateTest()}
    return SimObjectTest("test_object", None, states, actions)


def test_action_execute(sim_object):
    given_action = ActionTest()
    sim_object.execute_action(0, given_action)
    sim_object.execute_registered_actions(0)

    assert sim_object._actions[0].count == 1
    assert given_action.count == 1


def test_state_get(sim_object):
    assert sim_object.get_state("test") == 42
    assert sim_object.get_state("nonexistent") is None
