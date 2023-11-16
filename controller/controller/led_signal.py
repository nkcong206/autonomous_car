from enum import Enum
class led_signal(Enum):
    REACH_DESTINATION = 0
    PLACES_EMPTY = 1
    ERROR_GPS = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4
    ALL_BLOCK = 5
