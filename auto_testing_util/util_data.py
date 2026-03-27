from dataclasses import dataclass
from enum import Enum, auto

class PublishingMode(Enum):
    NONE = auto()
    ROBOT = auto()
    SIM = auto()

class AllianceStationID(Enum):
    Unknown = auto()
    Red1 = auto()
    Red2 = auto()
    Red3 = auto()
    Blue1 = auto()
    Blue2 = auto()
    Blue3 = auto()

class Pose:
    def __init__(self, x: float, y: float, rotation: float):
        self.x = x
        self.y = y
        self.rotation = rotation

    def copy(self):
        return Pose(self.x, self.y, self.rotation)

    def withX(self, x: float):
        return Pose(x, self.y, self.rotation)

    def withY(self, y: float):
        return Pose(self.x, y, self.rotation)

    def withRotation(self, rotation: float):
        return Pose(self.x, self.y, rotation)

    def __eq__(self, value):
        if not isinstance(value, Pose):
            return False
        return self.x == value.x and self.y == value.y and self.rotation == value.rotation

    def add(self, other: 'Pose'):
        return Pose(self.x + other.x, self.y + other.y, self.rotation + other.rotation)

@dataclass
class Auto:
    name: str
    can_be_mirrored: bool
    should_be_flipped: bool

class AutoStatus(Enum):
    Idle = auto()
    Running = auto()
