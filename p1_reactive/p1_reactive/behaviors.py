import math
import random
from .interface import FORWARD_SPEED
from .interface import Behavior
from .interface import VelocityCommand
from .interface import RANDOM_TURN_DISTANCE
from .interface import RANDOM_TURN_MAX

from .interface import ESCAPE_TOLERANCE
from .interface import ESCAPE_TURN
from .interface import FRONT_ANGLE
from .interface import FRONT_HALF_ANGLE
from .interface import OBSTACLE_RANGE
from .interface import MIN_OBSTACLE_RANGE
from .interface import MAX_OBSTACLE_RANGE
from .interface import TURN_SPEED

from .interface import FOOT
from .interface import UNDOCK_DISTANCE
from .interface import HALT_BLOCK_DURATION

# How close two sides of the front cone have to be (in meters) to count as
# a "roughly symmetric" obstacle (escape's job) rather than an asymmetric
# one (avoid's job).
SYMMETRY_TOLERANCE = 0.15 * 0.3048 

# How close (in radians) the robot has to get to its target heading before
# an escape turn counts as "done."
YAW_TOLERANCE = math.radians(5.0)

MOVE_TOLERANCE = 0.3 * FOOT

class DriveForward(Behavior):
    name = "drive_forward"

    def compute(self, state):
        if (state.now - state.earliest_free_time_since_last_halt < HALT_BLOCK_DURATION) or (state.halted == True):
            return None # Give robot time to scan for turns
        
        return VelocityCommand(
            linear=FORWARD_SPEED,
            angular=0.0
        )

class RandomTurn(Behavior):
    name = "random_turn"

    def __init__(self):
        self.start_x = None
        self.start_y = None
        self.turning = False
        self.target_yaw = 0.0

    def on_preempted(self, state):
        "Discard pending turn progress while a higher priority behavior runs."
        self.start_x = state.x
        self.start_y = state.y
        self.turning = False

    def compute(self, state):

        # Continue turning until target angle is reached
        if self.turning:

            remaining = _angle_diff(self.target_yaw, state.yaw)

            if abs(remaining) < YAW_TOLERANCE:
                self.turning = False
                print("Random turn complete")
                return None

            turn_speed = TURN_SPEED if remaining > 0 else -TURN_SPEED

            return VelocityCommand(
                linear=0.0,
                angular=turn_speed
            )

        # First time running, store position
        if self.start_x is None:
            self.start_x = state.x
            self.start_y = state.y
            return None

        # Calculate distance traveled
        distance = math.sqrt(
            (state.x - self.start_x) ** 2 +
            (state.y - self.start_y) ** 2
        )

        # After moving 1 foot, start random turn
        if distance >= RANDOM_TURN_DISTANCE:

            self.start_x = state.x
            self.start_y = state.y

            # Pick random turn angle
            angle = random.uniform(
                -RANDOM_TURN_MAX,
                RANDOM_TURN_MAX
            )

            self.target_yaw = state.yaw + angle
            self.turning = True

            print(
                f"Starting random turn: {math.degrees(angle):.2f} degrees"
            )

            return VelocityCommand(
                linear=0.0,
                angular=TURN_SPEED if angle > 0 else -TURN_SPEED
            )

        return None

    
def _angle_diff(a, b):
    # Return the smallest signed difference a - b, wrapped to (-pi, pi].
    
    
    
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _angle_range(angle):
   absolute_angle = abs(angle)

   range = (MAX_OBSTACLE_RANGE * MIN_OBSTACLE_RANGE) / math.sqrt(
       (MIN_OBSTACLE_RANGE**2) * math.cos(absolute_angle)**2 +
       (MAX_OBSTACLE_RANGE**2) * math.sin(absolute_angle)**2
   )

   return max(0, range)

def _front_cone_minimums(state):
   # Find the closest obstacle on each half of the front cone.
   ranges = state.ranges
   if not ranges or state.angle_increment == 0.0:
       return None, None

   left_min = None
   right_min = None

   for i, r in enumerate(ranges):
       if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
           continue
       angle = state.angle_min + i * state.angle_increment
       offset = _angle_diff(angle, FRONT_ANGLE)
       absolute_offset = abs(offset)

       if r > _angle_range(angle):
           continue
       if ((absolute_offset > FRONT_HALF_ANGLE) or ((absolute_offset > math.radians(55.0)) and (absolute_offset < math.radians(70.0)))):
           continue  # outside the front cone entirely or part of corners blocking lidar and should not be counted

       if offset >= 0.0:
           if left_min is None or r < left_min:
               left_min = r
       else:
           if right_min is None or r < right_min:
               right_min = r 
    
   return left_min, right_min

class EscapeBehavior(Behavior):
    # Priority-3 fixed action pattern: escape (roughly) symmetric obstacles.

    name = "escape"

    def __init__(self):
        self._escaping = False
        self._target_yaw = 0.0

    def on_preempted(self, state):
        "Cancel a stale escape turn if a higher-priority behavior takes over."
        self._escaping = False

    def compute(self, state):
        if self._escaping:
            return self._continue_escape(state)

        if (state.force_escape == True):
            print("forcing escape to get around the dock")
            state.force_escape = False
            self._start_escape(state)
            return self._continue_escape(state)

        left_min, right_min = _front_cone_minimums(state)
        if left_min is None or right_min is None:
            return None  # obstacle isn't on both sides -> not symmetric

        if abs(left_min - right_min) > SYMMETRY_TOLERANCE:
            return None  # asymmetric -> that's avoid()'s job

        self._start_escape(state)
        return self._continue_escape(state)

    def _start_escape(self, state):
        # Random direction, random magnitude within the tolerance band.
        direction = random.choice((-1.0, 1.0))
        magnitude = ESCAPE_TURN + random.uniform(-ESCAPE_TOLERANCE, ESCAPE_TOLERANCE)
        self._target_yaw = state.yaw + direction * magnitude
        self._escaping = True

    def _continue_escape(self, state):
        remaining = _angle_diff(self._target_yaw, state.yaw)
        if abs(remaining) < YAW_TOLERANCE:
            self._escaping = False
            return None  # turn complete, release control
        turn = TURN_SPEED if remaining > 0 else -TURN_SPEED
        return VelocityCommand(linear=0.0, angular=turn)


class AvoidBehavior(Behavior):
    # Priority-4 reflex: avoid asymmetric obstacles.

    name = "avoid"

    def compute(self, state):
        
        left_min, right_min = _front_cone_minimums(state)

        if left_min is None and right_min is None:
            return None  # nothing close in front

        if left_min is not None and right_min is not None:
            if abs(left_min - right_min) <= SYMMETRY_TOLERANCE:
                return None  # symmetric -> escape's job, not avoid's
            turn = -TURN_SPEED if left_min < right_min else TURN_SPEED
        elif left_min is not None:
            turn = -TURN_SPEED  # obstacle only on the left -> turn right
        else:
            turn = TURN_SPEED  # obstacle only on the right -> turn left

        return VelocityCommand(linear=0.0, angular=turn)

class Undock(Behavior):
    name = "undock"
    
    def __init__(self):
        self.start_x = None
        self.start_y = None
        self.current_yaw = None
        self.target_yaw = None
        self.finished_backing_out = False
        self.finished_turning_around = False

    def compute(self, state):
        if state.has_undocked == True:
            return None
        
        if ((self.finished_backing_out == False) and (self.finished_turning_around == True)):
            if (self.start_x) is None:
                print("first undock state")
                self.start_x = state.x
                self.start_y = state.y
            
            distance = math.sqrt(
                (state.x - self.start_x) ** 2 +
                (state.y - self.start_y) ** 2
            )
            
            
            if ((distance > UNDOCK_DISTANCE - MOVE_TOLERANCE) and (distance < UNDOCK_DISTANCE + MOVE_TOLERANCE)):
                self.finished_backing_out = True
                state.has_undocked = True
                print("Finished undock")
                return None
            
            print(UNDOCK_DISTANCE - distance)
            return VelocityCommand(
                linear=FORWARD_SPEED if (UNDOCK_DISTANCE - distance > 0) else -FORWARD_SPEED,
                angular=0.0
            )
        
        if (self.finished_turning_around == False):
            if (self.target_yaw is None):
                print(state.yaw)
                self.target_yaw = ((state.yaw + math.radians(180.0)) % (2 * math.pi))
                if self.target_yaw > math.pi:
                    self.target_yaw -= 2 * math.pi
                    
                print(self.target_yaw)
            
            remaining_yaw = _angle_diff(self.target_yaw, state.yaw)
            print(remaining_yaw)
            
            if abs(remaining_yaw) < YAW_TOLERANCE:
                self.finished_turning_around = True
                print("Random turn complete")
                return None
            
            return VelocityCommand(
                linear=0.0,
                angular=TURN_SPEED if remaining_yaw > 0 else -TURN_SPEED)
