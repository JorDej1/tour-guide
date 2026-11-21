"""Tests that the arbiter works as expected, enforcing priority correctly."""

from p1_reactive.behaviors import DriveForward, RandomTurn
from p1_reactive.halt_keyboard import HaltBehavior, KeyboardBehavior
from p1_reactive.interface import (
    Arbiter, Behavior, FORWARD_SPEED, SensorState, VelocityCommand,
)

KEY = VelocityCommand(linear=0.1, angular=0.5)
STOP = VelocityCommand(linear=0.0, angular=0.0)


def make_arbiter():
    # Escape and avoid go between random turn once merged
    return Arbiter([HaltBehavior(), KeyboardBehavior(), RandomTurn(), DriveForward()])


class CountingBehavior(Behavior):
    """Test helper that tracks how many times it was asked, always passes."""

    name = 'counter'

    def __init__(self):
        self.calls = 0

    def compute(self, state):
        self.calls += 1
        return None


def test_forward_wins_when_nothing_else_active():
    name, cmd = make_arbiter().select(SensorState())
    assert name == 'drive_forward'
    assert cmd == VelocityCommand(linear=FORWARD_SPEED, angular=0.0)


def test_keyboard_beats_autonomy():
    state = SensorState(teleop_cmd=KEY, teleop_stamp=5.0, now=5.1)
    assert make_arbiter().select(state) == ('keyboard', KEY)


def test_halt_beats_keyboard():
    state = SensorState(bumped=True, teleop_cmd=KEY, teleop_stamp=5.0, now=5.1)
    assert make_arbiter().select(state) == ('halt', STOP)


def test_outranked_behaviors_still_run():
    counter = CountingBehavior()
    Arbiter([HaltBehavior(), counter]).select(SensorState(bumped=True))
    assert counter.calls == 1


def test_everyone_passes_means_stop():
    assert Arbiter([CountingBehavior()]).select(SensorState()) == ('none', STOP)
