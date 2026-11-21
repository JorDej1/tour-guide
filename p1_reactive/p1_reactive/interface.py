"""This is our shared interface."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple


# Relevant constants, subject to change

FOOT = 0.3048  # meters per foot

OBSTACLE_RANGE = 1.0 * FOOT             # Obstacle detection range, currently obselete because this was for the cone check, not the ellipse addon
FRONT_HALF_ANGLE = math.radians(90.0)   # 'front' angle used for detection
FRONT_ANGLE = 0.0              # lidar angle straight ahead

UNDOCK_DISTANCE = 1.5 * FOOT
HALT_BLOCK_DURATION = 3.5 # Seconds drive forward is blocked after being freed from halt

MIN_OBSTACLE_RANGE = 1.0 * FOOT        # Detection range on the sides, gradually increases to MAX_OBSTACLE_RANGE when approaching center
MAX_OBSTACLE_RANGE = 1.0 * FOOT         # Detection range in the center

RANDOM_TURN_DISTANCE = 1.0 * FOOT
RANDOM_TURN_MAX = math.radians(15.0)    # randomly turns up to 15 degrees in either direction

ESCAPE_TURN = math.radians(180.0)       # turns away from obstacle
ESCAPE_TOLERANCE = math.radians(30.0)   # slight tolerance for escape turn

FORWARD_SPEED = 0.13                     # Bot speed in m/s
TURN_SPEED = 1.0                        # Turn speed in rad/s
CONTROL_PERIOD = 0.1                    # Arbiter loop time in s
TELEOP_TIMEOUT = 0.5                    # Keyboard gives up control after this many seconds


# Relevant ROS2 topic list constants

SCAN_TOPIC = '/scan'                    # lidar readings for certain behvaviors
ODOM_TOPIC = '/odom'                    # odometry readings for certain behaviors
HAZARD_TOPIC = '/hazard_detection'      # bumper contacts arrive here
CMD_VEL_TOPIC = '/cmd_vel'              # arbiter only
TELEOP_TOPIC = '/teleop_twist_keyboard'        # teleop_twist_keyboard remapped here


@dataclass
class VelocityCommand:
    """What a behavior asks the robot to do. Arbiter will select one of these to send to the robot."""

    linear: float = 0.0   # forward speed, m/s
    angular: float = 0.0  # turn speed, rad/s


@dataclass
class SensorState:
    """Latest sensor readings, the robots view of the world."""

    # Initial phase
    has_undocked = False
    
    # Lidar
    ranges: List[float] = field(default_factory=list)
    angle_min: float = 0.0
    angle_increment: float = 0.0

    # Odometry
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    # Bumper
    bumped: bool = False

    # Keyboard
    teleop_cmd: Optional[VelocityCommand] = None
    teleop_stamp: float = 0.0

    # Halting 
    earliest_free_time_since_last_halt: float = 0.0
    halted: bool = False
    force_escape = False # to get around bumping into the dock

    # Clock
    now: float = 0.0
    start_time: float | None = None


class Behavior(ABC):
    """Behavior adherence template."""

    name: str = 'behavior'

    @abstractmethod
    def compute(self, state: SensorState) -> Optional[VelocityCommand]:
        """Return a command to request control, or None to pass."""

    def on_preempted(self, state: SensorState) -> None:
        """Discard any in-progress behavior state after a higher priority wins."""


class Arbiter:
    """Enforces a priority ordering of behaviors."""

    def __init__(self, behaviors: List[Behavior]):
        self.behaviors = behaviors  # ordered in terms of project specs given priority

    # Returns the first non-None command in priority order.
    def select(self, state: SensorState) -> Tuple[str, VelocityCommand]:
        for index, behavior in enumerate(self.behaviors):
            cmd = behavior.compute(state)
            if cmd is not None:
                for preempted in self.behaviors[index + 1:]:
                    preempted.on_preempted(state)
                return behavior.name, cmd
        return 'none', VelocityCommand()
