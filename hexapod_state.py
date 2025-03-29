from bezier import *
from enum import Enum


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


# RC control data class (simplified)
class RC_Control_Data:
    def __init__(self):
        self.slider1 = 50
        # Add other fields as needed


rc_control_data = RC_Control_Data()


def car_state():
    global left_slider, global_speed_multiplier, global_rotation_multiplier
    global current_state, current_gait, previous_gait, push_fraction
    global speed_multiplier, stride_length_multiplier, lift_height_multiplier
    global max_stride_length, max_speed, forward_amount, turn_amount

    if current_state != State.CAR:
        print('Car State.')

    left_slider = int(rc_control_data.slider1)
    global_speed_multiplier = (left_slider + 10.0) * 0.01
    global_rotation_multiplier = map_float(rc_control_data.slider1, 0, 100, 40, 130) * 0.01

    if current_state != State.CAR or previous_gait != current_gait:
        current_state = State.CAR

        # Initialize Leg States
        for i in range(6):
            leg_states[i] = LegState.RESET

        if current_gait == Gait.TRI:
            cycle_progress[0] = 0
            cycle_progress[1] = points / 2
            cycle_progress[2] = 0
            cycle_progress[3] = points / 2
            cycle_progress[4] = 0
            cycle_progress[5] = points / 2

            push_fraction = 3.1 / 6.0
            speed_multiplier = 1
            stride_length_multiplier = 1.2
            lift_height_multiplier = 1.1
            max_stride_length = 240
            max_speed = 200

        elif current_gait == Gait.WAVE:
            # Offsets
            cycle_progress[0] = 0
            cycle_progress[1] = points / 6
            cycle_progress[2] = (points / 6) * 2
            cycle_progress[3] = (points / 6) * 5
            cycle_progress[4] = (points / 6) * 4
            cycle_progress[5] = (points / 6) * 3

            # Percentage Time On Ground
            push_fraction = 4.9 / 6.0

            speed_multiplier = 0.40
            stride_length_multiplier = 2
            lift_height_multiplier = 1.2
            max_stride_length = 150
            max_speed = 160

        # Add other gait cases here...

    for i in range(6):
        t_array[i] = float(cycle_progress[i]) / points

    forward_amount = joy1_current_magnitude
    turn_amount = joy2_current_vector.x

    for i in range(6):
        move_to_pos(i, get_gait_point(i, push_fraction))

    progress_change_amount = (
        max(abs(forward_amount), abs(turn_amount)) * speed_multiplier
    ) * global_speed_multiplier
    progress_change_amount = constrain(
        progress_change_amount, 0, max_speed * global_speed_multiplier
    )

    for i in range(6):
        cycle_progress[i] += progress_change_amount
        if cycle_progress[i] >= points:
            cycle_progress[i] = cycle_progress[i] - points


def get_gait_point(leg, push_fraction):
    global control_points_amount, rotate_control_points_amount

    rotate_stride_length = joy2_current_vector.x * global_rotation_multiplier
    v = Vector2(joy1_current_vector.x, joy1_current_vector.y)

    if not dynamic_stride_length:
        v.normalize()
        v = v * 70

    v = v * Vector2(1, stride_length_multiplier)
    v.y = constrain(v.y, -max_stride_length / 2, max_stride_length / 2)
    v = v * global_speed_multiplier

    if not dynamic_stride_length:
        if rotate_stride_length < 0:
            rotate_stride_length = -70
        else:
            rotate_stride_length = 70

    weight_sum = abs(forward_amount) + abs(turn_amount)
    t = t_array[leg]

    # Propelling
    if t < push_fraction:
        if leg_states[leg] != LegState.PROPELLING:
            set_cycle_start_points(leg)
        leg_states[leg] = LegState.PROPELLING

        control_points[0] = cycle_start_points[leg]
        control_points[1] = Vector3(
            v.x * stride_multiplier[leg] + distance_from_center,
            -v.y * stride_multiplier[leg],
            distance_from_ground,
        ).rotate(leg_placement_angle * rotation_multiplier[leg], Vector2(distance_from_center, 0))
        control_points_amount = 2

        straight_point = get_point_on_bezier_curve(
            control_points[:control_points_amount],
            control_points_amount,
            map_float(t, 0, push_fraction, 0, 1),
        )

        rotate_control_points[0] = cycle_start_points[leg]
        rotate_control_points[1] = Vector3(distance_from_center + 40, 0, distance_from_ground)
        rotate_control_points[2] = Vector3(
            distance_from_center, rotate_stride_length, distance_from_ground
        )
        rotate_control_points_amount = 3

        rotate_point = get_point_on_bezier_curve(
            rotate_control_points[:rotate_control_points_amount],
            rotate_control_points_amount,
            map_float(t, 0, push_fraction, 0, 1),
        )

        return (straight_point * abs(forward_amount) + rotate_point * abs(turn_amount)) / weight_sum

    # Lifting
    else:
        if leg_states[leg] != LegState.LIFTING:
            set_cycle_start_points(leg)
        leg_states[leg] = LegState.LIFTING

        # Set control points for lifting phase
        # ... (similar to the Arduino code)

        # Return weighted average of straight and rotate points
        # ... (similar to the Arduino code)


def set_cycle_start_points(leg):
    cycle_start_points[leg] = current_points[leg]


def move_to_pos(leg, position):
    # This would be implemented to control the actual leg movement
    current_points[leg] = position
    pass
