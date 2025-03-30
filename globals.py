import time
from bezier import Vector2, Vector3, get_point_on_bezier_curve, map_float, constrain
from enum import Enum


class State(Enum):
    INITIALIZE = 0
    STAND = 1
    CAR = 2
    CRAB = 3
    CALIBRATE = 4
    SLAM_ATTACK = 5
    SLEEP = 6
    ATTACH = 7


class LegState(Enum):
    PROPELLING = 0
    LIFTING = 1
    STANDING = 2
    RESET = 3


class Gait(Enum):
    TRI = 0
    RIPPLE = 1
    WAVE = 2
    QUAD = 3
    BI = 4
    HOP = 5


class PackageType(Enum):
    RC_CONTROL_DATA = 1
    RC_SETTINGS_DATA = 2
    HEXAPOD_SETTINGS_DATA = 3
    HEXAPOD_SENSOR_DATA = 4


# Button states
UNPRESSED = 0x1
PRESSED = 0x0


class GlobalState:
    def __init__(self):
        self.current_type = PackageType.RC_CONTROL_DATA

        # Global variables
        self.connected = False
        self.dynamic_stride_length = False

        self.gaits = [Gait.TRI, Gait.RIPPLE, Gait.WAVE, Gait.QUAD, Gait.BI, Gait.HOP]

        self.raw_offsets = [0] * 18
        self.base_offset = Vector3(90, 50, -10)
        self.offsets = [Vector3() for _ in range(6)]

        self.current_state = State.INITIALIZE
        self.current_gait = Gait.TRI
        self.previous_gait = Gait.TRI

        self.standing_distance_adjustment = 0
        self.distance_from_ground_base = -60
        self.target_distance_from_ground = 0

        self.joy1_target_vector = Vector2(0, 0)
        self.joy1_target_magnitude = 0
        self.joy1_current_vector = Vector2(0, 0)
        self.joy1_current_magnitude = 0

        self.joy2_target_vector = Vector2(0, 0)
        self.joy2_target_magnitude = 0
        self.joy2_current_vector = Vector2(0, 0)
        self.joy2_current_magnitude = 0

        self.time_of_last_input = time.time()
        self.attack_cooldown = 0
        self.elapsed_time = 0
        self.loop_start_time = 0

        # Car state variables
        self.forward_amount = 0
        self.turn_amount = 0
        self.t_array = [0] * 6
        self.control_points_amount = 0
        self.rotate_control_points_amount = 0
        self.push_fraction = 3.0 / 6.0
        self.speed_multiplier = 0.5
        self.stride_length_multiplier = 1.5
        self.lift_height_multiplier = 1.0
        self.max_stride_length = 200
        self.max_speed = 100

        self.left_slider = 50
        self.global_speed_multiplier = 0.55
        self.global_rotation_multiplier = 0.55

        # These would be initialized elsewhere in the full implementation
        self.control_points = [Vector3() for _ in range(10)]  # Assuming max 10 control points
        self.rotate_control_points = [Vector3() for _ in range(10)]
        self.cycle_start_points = [Vector3() for _ in range(6)]
        self.current_points = [Vector3() for _ in range(6)]
        self.cycle_progress = [0] * 6
        self.leg_states = [LegState.RESET] * 6

        self.leg_placement_angle = 45
        self.stride_multiplier = [1, 1, 1, -1, -1, -1]
        self.rotation_multiplier = [-1, 0, 1, -1, 0, 1]

        self.distance_from_ground = -60
        self.distance_from_center = 170 # Used in kinematics for the distance of trajectory from center
        self.lift_height = 130
        self.land_height = 70
        self.stride_overshoot = 10
        self.points = 1000

        self.current_legs = [-1, -1, -1]
        self.stand_loops = 0
        self.stand_progress = 0

        # Sleep state variables
        self.target_sleep_position = Vector3(130, 0, -46)
        self.sleep_state_state = 1

        # Attack state variables
        self.slam_started = False

        self.servo_attached = False


# Create a singleton instance
g = GlobalState()


def milliseconds():
    return int(time.time() * 1000)
