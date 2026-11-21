"""Test for the halt and keyboard behaviors."""

from p1_reactive.halt_keyboard import HaltBehavior, KeyboardBehavior
from p1_reactive.interface import Arbiter, SensorState, VelocityCommand


def test_halt_passes_when_not_bumped():
    assert HaltBehavior().compute(SensorState(bumped=False)) is None


def test_halt_stops_when_bumped():
    cmd = HaltBehavior().compute(SensorState(bumped=True))
    assert cmd == VelocityCommand(linear=0.0, angular=0.0)

def test_keyboard_passes_with_no_input():
    assert KeyboardBehavior().compute(SensorState()) is None


def test_keyboard_returns_fresh_command():
    cmd = VelocityCommand(linear=0.1, angular=0.5)
    state = SensorState(teleop_cmd=cmd, teleop_stamp=10.0, now=10.2)
    assert KeyboardBehavior().compute(state) == cmd


def test_keyboard_passes_when_stale():
    cmd = VelocityCommand(linear=0.1, angular=0.5)
    state = SensorState(teleop_cmd=cmd, teleop_stamp=10.0, now=11.0)
    assert KeyboardBehavior().compute(state) is None


def test_keyboard_stop_key_still_claims_control():
    stop = VelocityCommand(linear=0.0, angular=0.0)
    state = SensorState(teleop_cmd=stop, teleop_stamp=10.0, now=10.1)
    assert KeyboardBehavior().compute(state) == stop

def test_halt_releases_after_duration():
    halt = HaltBehavior()
    halt.compute(SensorState(bumped=True, now=10.0))
    assert halt.compute(SensorState(bumped=True, now=11.5)) is None


def test_halt_rearms_after_contact_ends():
    halt = HaltBehavior()
    halt.compute(SensorState(bumped=True, now=10.0))
    halt.compute(SensorState(bumped=False, now=11.0))
    assert halt.compute(SensorState(bumped=True, now=12.0)) == VelocityCommand(0.0, 0.0)

def test_halt_releases_during_continuous_contact():
    halt = HaltBehavior()
    state = SensorState(bumped=True)
    cmd = None
    for i in range(20):  # 2 s of unbroken contact at 10 Hz
        state.now = 10.0 + 0.1 * i
        cmd = halt.compute(state)
    assert cmd is None  # halt must have released after 1 s
