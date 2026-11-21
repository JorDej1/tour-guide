import math

from p1_reactive.behaviors import AvoidBehavior, EscapeBehavior
from p1_reactive.halt_keyboard import KeyboardBehavior
from p1_reactive.interface import Arbiter, SensorState, VelocityCommand


def make_scan(front_left=None, front_right=None, num_points=360):
    ranges = [float('inf')] * num_points
    angle_min = -math.pi
    angle_increment = 2 * math.pi / num_points

    def index_for_angle(angle):
        return int(round((angle - angle_min) / angle_increment)) % num_points

    if front_left is not None:
        ranges[index_for_angle(math.radians(15))] = front_left
    if front_right is not None:
        ranges[index_for_angle(math.radians(-15))] = front_right

    return SensorState(
        ranges=ranges, angle_min=angle_min,
        angle_increment=angle_increment, yaw=0.0,
    )


def test_escape_triggers_on_symmetric_obstacle():
    escape = EscapeBehavior()
    state = make_scan(front_left=0.2, front_right=0.2)  # ~equal, both close
    cmd = escape.compute(state)
    assert cmd is not None
    assert cmd.angular != 0.0
    assert cmd.linear == 0.0


def test_escape_keeps_turning_after_obstacle_clears():
    # Fixed action pattern: escape should NOT stop just because the
    # obstacle that triggered it is gone on the next tick.

    escape = EscapeBehavior()
    triggering_state = make_scan(front_left=0.2, front_right=0.2)
    escape.compute(triggering_state)  # trigger the escape

    clear_state = make_scan()  # obstacle now gone, robot mid-turn
    clear_state.yaw = 0.01  # barely turned yet
    cmd = escape.compute(clear_state)
    assert cmd is not None  # still committed to the turn
    assert cmd.angular != 0.0


def test_escape_releases_once_target_heading_reached():
    escape = EscapeBehavior()
    triggering_state = make_scan(front_left=0.2, front_right=0.2)
    escape.compute(triggering_state)
    target = escape._target_yaw

    finished_state = make_scan()
    finished_state.yaw = target  # pretend we haveve turned all the way
    cmd = escape.compute(finished_state)
    assert cmd is None  # turn complete, control released

def test_avoid_turns_away_from_closer_side():
    avoid = AvoidBehavior()
    # Closer obstacle on the left -> should turn right (negative angular).
    state = make_scan(front_left=0.15, front_right=0.28)
    cmd = avoid.compute(state)
    assert cmd is not None
    assert cmd.angular < 0.0

def test_avoid_ignores_symmetric_obstacles():
    avoid = AvoidBehavior()
    state = make_scan(front_left=0.2, front_right=0.2)
    assert avoid.compute(state) is None  # that's escape's job


def test_avoid_ignores_far_obstacles():
    avoid = AvoidBehavior()
    state = make_scan(front_left=2.0, front_right=2.0)  # beyond OBSTACLE_RANGE
    assert avoid.compute(state) is None


def test_avoid_releases_when_obstacle_clears():
    avoid = AvoidBehavior()
    assert avoid.compute(make_scan()) is None
