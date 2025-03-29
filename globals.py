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


current_type = PackageType.RC_CONTROL_DATA

# Button states
UNPRESSED = 0x1
PRESSED = 0x0

# Global variables
connected = False
dynamic_stride_length = True

total_gaits = 6
gaits = [Gait.TRI, Gait.RIPPLE, Gait.WAVE, Gait.QUAD, Gait.BI, Gait.HOP]

raw_offsets = [0] * 18
base_offset = Vector3(90, 50, -10)
offsets = [Vector3() for _ in range(6)]

current_state = State.INITIALIZE
current_gait = Gait.TRI
previous_gait = Gait.TRI
current_gait_id = 0

standing_distance_adjustment = 0
distance_from_ground_base = -60
target_distance_from_ground = 0

joy1_target_vector = Vector2(0, 0)
joy1_target_magnitude = 0
joy1_current_vector = Vector2(0, 0)
joy1_current_magnitude = 0

joy2_target_vector = Vector2(0, 0)
joy2_target_magnitude = 0
joy2_current_vector = Vector2(0, 0)
joy2_current_magnitude = 0

time_since_last_input = 0
attack_cooldown = 0
elapsed_time = 0
loop_start_time = 0

# Car state variables
forward_amount = 0
turn_amount = 0
t_array = [0] * 6
control_points_amount = 0
rotate_control_points_amount = 0
push_fraction = 3.0 / 6.0
speed_multiplier = 0.5
stride_length_multiplier = 1.5
lift_height_multiplier = 1.0
max_stride_length = 200
max_speed = 100
leg_placement_angle = 56

left_slider = 50
global_speed_multiplier = 0.55
global_rotation_multiplier = 0.55

# These would be initialized elsewhere in the full implementation
control_points = [Vector3() for _ in range(10)]  # Assuming max 10 control points
rotate_control_points = [Vector3() for _ in range(10)]
cycle_start_points = [Vector3() for _ in range(6)]
current_points = [Vector3() for _ in range(6)]
cycle_progress = [0] * 6
leg_states = [LegState.RESET] * 6
stride_multiplier = [1.0] * 6
rotation_multiplier = [1.0] * 6
distance_from_ground = -60
distance_from_center = 173
lift_height = 130
land_height = 70
stride_overshoot = 10
points = 1000
