#!/usr/bin/env python3
from enum import Enum


class RoadWarning(Enum):
    STRAIGHT_ROAD_AHEAD = 2
    INTERSECTION_AHEAD = 0
    TURN_AHEAD = 1
    SAME_AHEAD = 3
