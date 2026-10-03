from enum import Enum, auto


class MissionState(Enum):
    IDLE = auto()
    GO_TO_PICKUP = auto()
    WAIT_FOR_PICKUP = auto()
    GO_TO_DROPOFF = auto()
    WAIT_FOR_DROP = auto()
    COMPLETE = auto()
