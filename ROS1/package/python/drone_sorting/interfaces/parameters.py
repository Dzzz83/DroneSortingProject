"""Validate private ROS parameters before passing them to typed application code."""

import math
from typing import List, Optional, Sequence

import rospy


def read_float(name: str, default: float, *, positive: bool = False) -> float:
    value = rospy.get_param('~' + name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('Parameter ~' + name + ' must be a number')
    number = float(value)
    if not math.isfinite(number) or (positive and number <= 0):
        raise ValueError('Parameter ~' + name + ' must be finite' + (' and positive' if positive else ''))
    return number


def read_int(name: str, default: int) -> int:
    value = rospy.get_param('~' + name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError('Parameter ~' + name + ' must be an integer')
    return value


def read_bool(name: str, default: bool) -> bool:
    value = rospy.get_param('~' + name, default)
    if not isinstance(value, bool):
        raise ValueError('Parameter ~' + name + ' must be true or false')
    return value


def read_float_list(name: str, default: Optional[Sequence[float]]) -> Optional[List[float]]:
    value = rospy.get_param('~' + name, default)
    if value is None and default is None:
        return None
    if not isinstance(value, (list, tuple)):
        raise ValueError('Parameter ~' + name + ' must be a list of numbers')
    numbers = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError('Parameter ~' + name + ' must contain finite numbers')
        numbers.append(float(item))
    return numbers
