"""Halt and keyboard behaviors."""

from .interface import Behavior, SensorState, TELEOP_TIMEOUT, VelocityCommand
from .behaviors import _front_cone_minimums

HALT_DURATION = 1


class HaltBehavior(Behavior):
    """Stop the robot when a bumper first reports a collision."""

    name = 'halt'

    def __init__(self):
        self._bump_start = 0.0

    def compute(self, state: SensorState):
        """Return a stop command during the halt window, else pass."""
        if (state.halted == True):
            if (state.now - self._bump_start >= HALT_DURATION):
                state.halted = False
                state.earliest_free_time_since_last_halt = state.now
                left_min, right_min = _front_cone_minimums(state)
                if (left_min == None) and (right_min == None):
                    state.force_escape = True
                return None
        if not state.bumped:
            return None
        state.halted = True
        self._bump_start = state.now
        if state.now - self._bump_start < HALT_DURATION:
            return VelocityCommand(linear=0.0, angular=0.0)
        return None


class KeyboardBehavior(Behavior):
    """Pass through human keyboard commands while the user is actively driving."""

    name = 'keyboard'

    def compute(self, state: SensorState):
        """Return the latest keyboard command if it is recent, else pass."""
        if state.teleop_cmd is None:
            return None
        if state.now - state.teleop_stamp > TELEOP_TIMEOUT:
            return None
        print("go")
        return state.teleop_cmd
