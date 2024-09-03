# Copyright (c) 2024, Leap AI.
# All rights reserved.

import typing

from .sim_object_base import SimObjectBase


class SimStaticObject(SimObjectBase):

    def post_reset(self, info: dict[str, typing.Any] | None = None) -> None:
        pass
