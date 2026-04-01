# Adapted from: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/px4_custom_mode.h
from enum import Enum


class FloatEnum(Enum):
    def __new__(cls, value, *args):
        obj = object.__new__(cls)
        obj._value_ = float(value)
        return obj


class PX4CustomMainMode(FloatEnum):
    MANUAL = 1.0
    ALTCTL = 2.0
    POSCTL = 3.0
    AUTO = 4.0
    ACRO = 5.0
    OFFBOARD = 6.0
    STABILIZED = 7.0
    RATTITUDE_LEGACY = 8.0
    SIMPLE = 9.0  # Unused/reserved for future use
    TERMINATION = 10.0


class PX4CustomSubModeAuto(FloatEnum):
    READY = 1.0
    TAKEOFF = 2.0
    LOITER = 3.0
    MISSION = 4.0
    RTL = 5.0
    LAND = 6.0
    RESERVED_DO_NOT_USE = 7.0  # was RTGS, deleted 2020-03-05
    FOLLOW_TARGET = 8.0
    PRECLAND = 9.0
    VTOL_TAKEOFF = 10.0
    EXTERNAL1 = 11.0
    EXTERNAL2 = 12.0
    EXTERNAL3 = 13.0
    EXTERNAL4 = 14.0
    EXTERNAL5 = 15.0
    EXTERNAL6 = 16.0
    EXTERNAL7 = 17.0
    EXTERNAL8 = 18.0


class PX4CustomSubModePosctl(FloatEnum):
    POSCTL = 0.0
    ORBIT = 1.0
    SLOW = 2.0


class PX4CustomMode:
    def __init__(self, main_mode=0, sub_mode=0, reserved=0):
        self.reserved = reserved
        self.main_mode = main_mode
        self.sub_mode = sub_mode

    def __repr__(self):
        return f"<PX4CustomMode main_mode={self.main_mode} sub_mode={self.sub_mode}>"


NAVIGATION_STATE_MANUAL = 0.0
NAVIGATION_STATE_ALTCTL = 1.0
NAVIGATION_STATE_POSCTL = 2.0
NAVIGATION_STATE_POSITION_SLOW = 3.0
NAVIGATION_STATE_AUTO_MISSION = 4.0
NAVIGATION_STATE_AUTO_LOITER = 5.0
NAVIGATION_STATE_AUTO_RTL = 6.0
NAVIGATION_STATE_ACRO = 7.0
NAVIGATION_STATE_DESCEND = 8.0
NAVIGATION_STATE_TERMINATION = 9.0
NAVIGATION_STATE_OFFBOARD = 10.0
NAVIGATION_STATE_STAB = 11.0
NAVIGATION_STATE_AUTO_TAKEOFF = 12.0
NAVIGATION_STATE_AUTO_LAND = 13.0
NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 14.0
NAVIGATION_STATE_AUTO_PRECLAND = 15.0
NAVIGATION_STATE_ORBIT = 16.0
NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 17.0
NAVIGATION_STATE_EXTERNAL1 = 18.0
NAVIGATION_STATE_EXTERNAL2 = 19.0
NAVIGATION_STATE_EXTERNAL3 = 20.0
NAVIGATION_STATE_EXTERNAL4 = 21.0
NAVIGATION_STATE_EXTERNAL5 = 22.0
NAVIGATION_STATE_EXTERNAL6 = 23.0
NAVIGATION_STATE_EXTERNAL7 = 24.0
NAVIGATION_STATE_EXTERNAL8 = 25.0


def get_px4_custom_mode(nav_state):
    """Translate a navigation state into a PX4 custom mode."""
    custom_mode = PX4CustomMode()

    if nav_state == NAVIGATION_STATE_MANUAL:
        custom_mode.main_mode = PX4CustomMainMode.MANUAL
    elif nav_state == NAVIGATION_STATE_ALTCTL:
        custom_mode.main_mode = PX4CustomMainMode.ALTCTL
    elif nav_state == NAVIGATION_STATE_POSCTL:
        custom_mode.main_mode = PX4CustomMainMode.POSCTL
    elif nav_state == NAVIGATION_STATE_POSITION_SLOW:
        custom_mode.main_mode = PX4CustomMainMode.POSCTL
        custom_mode.sub_mode = PX4CustomSubModePosctl.SLOW
    elif nav_state == NAVIGATION_STATE_AUTO_MISSION:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.MISSION
    elif nav_state == NAVIGATION_STATE_AUTO_LOITER:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.LOITER
    elif nav_state == NAVIGATION_STATE_AUTO_RTL:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.RTL
    elif nav_state == NAVIGATION_STATE_ACRO:
        custom_mode.main_mode = PX4CustomMainMode.ACRO
    elif nav_state == NAVIGATION_STATE_DESCEND:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.LAND
    elif nav_state == NAVIGATION_STATE_TERMINATION:
        custom_mode.main_mode = PX4CustomMainMode.TERMINATION
    elif nav_state == NAVIGATION_STATE_OFFBOARD:
        custom_mode.main_mode = PX4CustomMainMode.OFFBOARD
    elif nav_state == NAVIGATION_STATE_STAB:
        custom_mode.main_mode = PX4CustomMainMode.STABILIZED
    elif nav_state == NAVIGATION_STATE_AUTO_TAKEOFF:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.TAKEOFF
    elif nav_state == NAVIGATION_STATE_AUTO_LAND:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.LAND
    elif nav_state == NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.FOLLOW_TARGET
    elif nav_state == NAVIGATION_STATE_AUTO_PRECLAND:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.PRECLAND
    elif nav_state == NAVIGATION_STATE_ORBIT:
        custom_mode.main_mode = PX4CustomMainMode.POSCTL
        custom_mode.sub_mode = PX4CustomSubModePosctl.ORBIT
    elif nav_state == NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.VTOL_TAKEOFF
    elif nav_state == NAVIGATION_STATE_EXTERNAL1:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL1
    elif nav_state == NAVIGATION_STATE_EXTERNAL2:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL2
    elif nav_state == NAVIGATION_STATE_EXTERNAL3:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL3
    elif nav_state == NAVIGATION_STATE_EXTERNAL4:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL4
    elif nav_state == NAVIGATION_STATE_EXTERNAL5:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL5
    elif nav_state == NAVIGATION_STATE_EXTERNAL6:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL6
    elif nav_state == NAVIGATION_STATE_EXTERNAL7:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL7
    elif nav_state == NAVIGATION_STATE_EXTERNAL8:
        custom_mode.main_mode = PX4CustomMainMode.AUTO
        custom_mode.sub_mode = PX4CustomSubModeAuto.EXTERNAL8

    return custom_mode


# Example usage:
if __name__ == "__main__":
    nav_state = NAVIGATION_STATE_AUTO_MISSION
    mode = get_px4_custom_mode(nav_state)
    print(mode)
