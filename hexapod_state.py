import time
from bezier import constrain, get_point_on_bezier_curve, map_float, Vector2, Vector3
from globals import (
    g,
    Gait,
    LegState,
    State,
)
from hexapod_control import move_to_pos, set_cycle_start_points
from nrf import (
    rc_settings_data,
    hex_settings_data,
    rc_control_data,
)
from hexapod_initializations import a1, a2, detach_servos

# Standing Control Points Array
SCPA = [[Vector3(0, 0, 0) for _ in range(10)] for _ in range(6)]

# Standing state variables
standing_start_points = [
    Vector3(0, 0, 0) for _ in range(6)
]  # points legs are at when entering standing state
standing_in_between_points = [Vector3(0, 0, 0) for _ in range(6)]  # middle points of bezier curves
g.standing_end_point = Vector3(0, 0, 0)


def car_state():
    if g.current_state != State.CAR:
        print('Car State.')

    g.left_slider = int(rc_control_data.slider1)
    g.global_speed_multiplier = (g.left_slider + 10.0) * 0.01
    g.global_rotation_multiplier = map_float(rc_control_data.slider1, 0, 100, 40, 130) * 0.01

    if g.current_state != State.CAR or g.previous_gait != g.current_gait:
        g.current_state = State.CAR

        # Initialize Leg States
        for i in range(6):
            g.leg_states[i] = LegState.RESET

    if g.current_gait == Gait.TRI:
        g.cycle_progress[0] = 0
        g.cycle_progress[1] = g.points / 2
        g.cycle_progress[2] = 0
        g.cycle_progress[3] = g.points / 2
        g.cycle_progress[4] = 0
        g.cycle_progress[5] = g.points / 2

        g.push_fraction = 3.1 / 6.0
        g.speed_multiplier = 1
        g.stride_length_multiplier = 1.2
        g.lift_height_multiplier = 1.1
        g.max_stride_length = 240
        g.max_speed = 200

    elif g.current_gait == Gait.WAVE:
        # Offsets
        g.cycle_progress[0] = 0
        g.cycle_progress[1] = g.points / 6
        g.cycle_progress[2] = (g.points / 6) * 2
        g.cycle_progress[3] = (g.points / 6) * 5
        g.cycle_progress[4] = (g.points / 6) * 4
        g.cycle_progress[5] = (g.points / 6) * 3

        # Percentage Time On Ground
        g.push_fraction = 4.9 / 6.0

        g.speed_multiplier = 0.40
        g.stride_length_multiplier = 2
        g.lift_height_multiplier = 1.2
        g.max_stride_length = 150
        g.max_speed = 160

    elif g.current_gait == Gait.RIPPLE:
        # Offsets
        g.cycle_progress[0] = 0
        g.cycle_progress[1] = (g.points / 6) * 4
        g.cycle_progress[2] = (g.points / 6) * 2
        g.cycle_progress[3] = (g.points / 6) * 5
        g.cycle_progress[4] = g.points / 6
        g.cycle_progress[5] = (g.points / 6) * 3

        # Percentage Time On Ground
        g.push_fraction = 3.2 / 6.0

        g.speed_multiplier = 1
        g.stride_length_multiplier = 1.3
        g.lift_height_multiplier = 1.1
        g.max_stride_length = 220
        g.max_speed = 200

    elif g.current_gait == Gait.BI:
        # Offsets
        g.cycle_progress[0] = 0
        g.cycle_progress[1] = g.points / 3
        g.cycle_progress[2] = (g.points / 3) * 2
        g.cycle_progress[3] = 0
        g.cycle_progress[4] = g.points / 3
        g.cycle_progress[5] = (g.points / 3) * 2

        # Percentage Time On Ground
        g.push_fraction = 2.1 / 6.0

        g.speed_multiplier = 4
        g.stride_length_multiplier = 1
        g.lift_height_multiplier = 1.8
        g.max_stride_length = 230
        g.max_speed = 130

    elif g.current_gait == Gait.QUAD:
        # Offsets
        g.cycle_progress[0] = 0
        g.cycle_progress[1] = g.points / 3
        g.cycle_progress[2] = (g.points / 3) * 2
        g.cycle_progress[3] = 0
        g.cycle_progress[4] = g.points / 3
        g.cycle_progress[5] = (g.points / 3) * 2

        # Percentage Time On Ground
        g.push_fraction = 4.1 / 6.0

        g.speed_multiplier = 1
        g.stride_length_multiplier = 1.2
        g.lift_height_multiplier = 1.1
        g.max_stride_length = 220
        g.max_speed = 200

    elif g.current_gait == Gait.HOP:
        # Offsets
        g.cycle_progress[0] = 0
        g.cycle_progress[1] = 0
        g.cycle_progress[2] = 0
        g.cycle_progress[3] = 0
        g.cycle_progress[4] = 0
        g.cycle_progress[5] = 0

        # Percentage Time On Ground
        g.push_fraction = 3 / 6.0

        g.speed_multiplier = 1
        g.stride_length_multiplier = 1.6
        g.lift_height_multiplier = 2.5
        g.max_stride_length = 240
        g.max_speed = 200

    for i in range(6):
        g.t_array[i] = float(g.cycle_progress[i]) / g.points

    g.forward_amount = g.joy1_current_magnitude
    g.turn_amount = g.joy2_current_vector.x

    for i in range(6):
        move_to_pos(i, get_gait_point(i, g.push_fraction))

    progress_change_amount = (
        max(abs(g.forward_amount), abs(g.turn_amount)) * g.speed_multiplier
    ) * g.global_speed_multiplier
    progress_change_amount = constrain(
        progress_change_amount, 0, g.max_speed * g.global_speed_multiplier
    )

    for i in range(6):
        g.cycle_progress[i] += progress_change_amount
        if g.cycle_progress[i] >= g.points:
            g.cycle_progress[i] = g.cycle_progress[i] - g.points


def get_gait_point(leg, push_fraction):
    rotate_stride_length = g.joy2_current_vector.x * g.global_rotation_multiplier
    v = g.joy1_current_vector.copy()

    if not g.dynamic_stride_length:
        v.normalize()
        v = v * 70

    v = v * Vector2(1, g.stride_length_multiplier)
    v.y = constrain(v.y, -g.max_stride_length / 2, g.max_stride_length / 2)
    v = v * g.global_speed_multiplier

    if not g.dynamic_stride_length:
        if rotate_stride_length < 0:
            rotate_stride_length = -70
        else:
            rotate_stride_length = 70

    weight_sum = abs(g.forward_amount) + abs(g.turn_amount)

    t = g.t_array[leg]

    # Propelling
    if t < push_fraction:
        if g.leg_states[leg] != LegState.PROPELLING:
            set_cycle_start_points(leg)
        g.leg_states[leg] = LegState.PROPELLING

        g.control_points[0] = g.cycle_start_points[leg]
        g.control_points[1] = Vector3(
            v.x * g.stride_multiplier[leg] + g.distance_from_center,
            -v.y * g.stride_multiplier[leg],
            g.distance_from_ground,
        ).rotate(
            g.leg_placement_angle * g.rotation_multiplier[leg], Vector2(g.distance_from_center, 0)
        )
        g.control_points_amount = 2
        straight_point = get_point_on_bezier_curve(
            g.control_points[: g.control_points_amount],
            g.control_points_amount,
            map_float(t, 0, push_fraction, 0, 1),
        )

        g.rotate_control_points[0] = g.cycle_start_points[leg]
        g.rotate_control_points[1] = Vector3(g.distance_from_center + 40, 0, g.distance_from_ground)
        g.rotate_control_points[2] = Vector3(
            g.distance_from_center, rotate_stride_length, g.distance_from_ground
        )
        g.rotate_control_points_amount = 3
        rotate_point = get_point_on_bezier_curve(
            g.rotate_control_points[: g.rotate_control_points_amount],
            g.rotate_control_points_amount,
            map_float(t, 0, push_fraction, 0, 1),
        )

        return (
            straight_point * abs(g.forward_amount) + rotate_point * abs(g.turn_amount)
        ) / weight_sum

    # Lifting
    else:
        if g.leg_states[leg] != LegState.LIFTING:
            set_cycle_start_points(leg)
        g.leg_states[leg] = LegState.LIFTING

        g.control_points[0] = g.cycle_start_points[leg]
        g.control_points[1] = g.cycle_start_points[leg] + Vector3(
            0, 0, g.lift_height * g.lift_height_multiplier
        )
        g.control_points[2] = Vector3(
            -v.x * g.stride_multiplier[leg] + g.distance_from_center,
            (v.y + g.stride_overshoot) * g.stride_multiplier[leg],
            g.distance_from_ground + g.land_height,
        ).rotate(
            g.leg_placement_angle * g.rotation_multiplier[leg], Vector2(g.distance_from_center, 0)
        )
        g.control_points[3] = Vector3(
            -v.x * g.stride_multiplier[leg] + g.distance_from_center,
            v.y * g.stride_multiplier[leg],
            g.distance_from_ground,
        ).rotate(
            g.leg_placement_angle * g.rotation_multiplier[leg], Vector2(g.distance_from_center, 0)
        )
        g.control_points_amount = 4
        straight_point = get_point_on_bezier_curve(
            g.control_points[: g.control_points_amount],
            g.control_points_amount,
            map_float(t, push_fraction, 1, 0, 1),
        )

        g.rotate_control_points[0] = g.cycle_start_points[leg]
        g.rotate_control_points[1] = g.cycle_start_points[leg] + Vector3(
            0, 0, g.lift_height * g.lift_height_multiplier
        )
        g.rotate_control_points[2] = Vector3(
            g.distance_from_center + 40,
            0,
            g.distance_from_ground + g.lift_height * g.lift_height_multiplier,
        )
        g.rotate_control_points[3] = Vector3(
            g.distance_from_center,
            -(rotate_stride_length + g.stride_overshoot),
            g.distance_from_ground + g.land_height,
        )
        g.rotate_control_points[4] = Vector3(
            g.distance_from_center, -rotate_stride_length, g.distance_from_ground
        )
        g.rotate_control_points_amount = 5
        rotate_point = get_point_on_bezier_curve(
            g.rotate_control_points[: g.rotate_control_points_amount],
            g.rotate_control_points_amount,
            map_float(t, push_fraction, 1, 0, 1),
        )

        return (
            straight_point * abs(g.forward_amount) + rotate_point * abs(g.turn_amount)
        ) / weight_sum


# AM - checked
def lerp(
    a: float | Vector2 | Vector3, b: float | Vector2 | Vector3, t: float
) -> float | Vector2 | Vector3:
    """
    Linear interpolation between a and b by t amount.

    Args:
        a: Starting value
        b: Ending value
        t: Interpolation factor (0.0 to 1.0)

    Returns:
        Interpolated value

    """
    if isinstance(a, Vector3) and isinstance(b, Vector3):
        return __lerp_vector3(a, b, t)
    elif isinstance(a, Vector2) and isinstance(b, Vector2):
        return __lerp_vector2(a, b, t)
    else:
        return __lerp_float(a, b, t)


def __lerp_float(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# AM - checked
def __lerp_vector2(a: Vector2, b: Vector2, f: float) -> Vector2:
    return Vector2(__lerp_float(a.x, b.x, f), __lerp_float(a.y, b.y, f))


# AM - checked
def __lerp_vector3(a: Vector3, b: Vector3, f: float) -> Vector3:
    return Vector3(__lerp_float(a.x, b.x, f), __lerp_float(a.y, b.y, f), __lerp_float(a.z, b.z, f))


# AM - checked Hexapod_Code/Initialization_State.ino
def state_initialize():
    move_to_pos(0, Vector3(160, 0, 0))
    move_to_pos(1, Vector3(160, 0, 0))
    move_to_pos(2, Vector3(160, 0, 0))
    move_to_pos(3, Vector3(160, 0, 0))
    move_to_pos(4, Vector3(160, 0, 0))
    move_to_pos(5, Vector3(160, 0, 0))

    # time.sleep(25 / 1000)

    move_to_pos(0, Vector3(225, 0, 115))
    move_to_pos(1, Vector3(225, 0, 115))
    move_to_pos(2, Vector3(225, 0, 115))
    move_to_pos(3, Vector3(225, 0, 115))
    move_to_pos(4, Vector3(225, 0, 115))
    move_to_pos(5, Vector3(225, 0, 115))

    # time.sleep(50 / 1000)


# AM - checked, stub
def get_send_nrf_data():
    """
    Exchange data with the remote control.

    Returns:
        bool: True if connection is active, False otherwise

    """
    # In a real implementation, this would communicate with the remote control
    # For example, using an RF24 radio module:
    # if radio.available():
    #     buffer = bytearray(radio.getDynamicPayloadSize())
    #     radio.read(buffer, len(buffer))
    #     # Parse buffer into rc_control_data
    #     return True

    # For simulation/testing:
    # Simulate receiving data
    # rc_control_data.joy1_X = 127  # Center position
    # rc_control_data.joy1_Y = 127  # Center position

    # For now, always return True to simulate an active connection
    return True


# AM - checked
def standing_state():
    """Put the hexapod in a standing position."""
    if g.current_state != State.STAND:
        print('Standing State')

    move_all_at_once = False
    high_lift = False
    set_cycle_start_points()
    # Use the standing_distance_adjustment variable here
    g.standing_end_point = Vector3(
        g.distance_from_center, 0, g.distance_from_ground + g.standing_distance_adjustment
    )
    g.stand_loops = 2

    # Set flags based on current state
    if (
        g.current_state == State.CALIBRATE
        or g.current_state == State.INITIALIZE
        or g.current_state == State.SLAM_ATTACK
        or g.current_state == State.SLEEP
        or g.current_state == State.ATTACH
    ):
        move_all_at_once = True

    if g.current_state == State.SLAM_ATTACK or g.current_state == State.SLEEP:
        high_lift = True

    if g.current_state != State.STAND:
        set_3_highest_leg()
        g.stand_loops = 0
        g.stand_progress = 0

        # Copy current points to standing start points
        for i in range(6):
            standing_start_points[i] = g.current_points[i]

        g.current_state = State.STAND

        # Calculate the in-between and ending points
        for i in range(6):
            in_between_point = Vector3(
                standing_start_points[i].x, standing_start_points[i].y, standing_start_points[i].z
            )
            in_between_point.x = (in_between_point.x + g.standing_end_point.x) / 1.5
            in_between_point.y = (in_between_point.y + g.standing_end_point.y) / 1.5

            in_between_point.z = (in_between_point.z + g.standing_end_point.z) / 2
            if abs(in_between_point.z - g.standing_end_point.z) < 50:
                in_between_point.z += 70
            if high_lift:
                in_between_point.z += 80

            standing_in_between_points[i] = in_between_point

            SCPA[i][0] = standing_start_points[i]
            SCPA[i][1] = standing_in_between_points[i]
            SCPA[i][2] = g.standing_end_point

        for i in range(6):
            g.leg_states[i] = LegState.STANDING

    # Update distance from ground constantly
    for i in range(6):
        SCPA[i][2] = g.standing_end_point

    # Readjusting - takes about a second
    while g.stand_loops < 2:
        g.stand_progress += 20

        t = float(g.stand_progress) / g.points
        if t > 1:
            t = 1

        if move_all_at_once:
            for i in range(6):
                move_to_pos(i, get_point_on_bezier_curve(SCPA[i][:3], 3, t))

            if g.stand_progress > g.points:
                g.stand_progress = 0
                g.stand_loops = 2
        else:
            for i in range(3):
                if g.current_legs[i] != -1:
                    move_to_pos(
                        g.current_legs[i],
                        get_point_on_bezier_curve(SCPA[g.current_legs[i]][:3], 3, t),
                    )

            if g.stand_progress > g.points:
                g.stand_progress = 0
                g.stand_loops += 1
                set_3_highest_leg()

    # Constantly move to the standing end position
    for i in range(6):
        move_to_pos(i, get_point_on_bezier_curve(SCPA[i][:3], 3, 1))


def set_3_highest_leg():
    """Select the three legs with highest z-position to move first."""
    g.current_legs[0] = -1
    g.current_legs[1] = -1
    g.current_legs[2] = -1

    for j in range(3):
        for i in range(6):  # Go through the legs
            # If leg already on the list of current legs, skip it
            if i in g.current_legs:
                continue

            # If leg already in position, don't add it
            if g.current_points[i] == g.standing_end_point:
                continue

            # If leg's z is greater than the leg already there, add it
            if (
                g.current_legs[j] == -1
                or g.current_points[i].z > g.current_points[g.current_legs[j]].z
            ):
                g.current_legs[j] = i


# AM - checked
def sleep_state():
    """Put the hexapod in a sleep/powered down position."""
    if g.current_state != State.SLEEP:
        print('Sleep State')
        g.sleep_state_state = 1

    g.current_state = State.SLEEP

    # Skip if servos are not attached - using the correct variable
    if not g.servos_attached:
        return

    # State 1: Move to sleep position
    if g.sleep_state_state == 1:
        target_reached = True

        for i in range(6):
            next_pos = lerp(g.current_points[i], g.target_sleep_position, 0.03)

            # Snap to target when very close
            if abs(g.current_points[i].x - g.target_sleep_position.x) < 1:
                next_pos.x = g.target_sleep_position.x
            if abs(g.current_points[i].y - g.target_sleep_position.y) < 1:
                next_pos.y = g.target_sleep_position.y
            if abs(g.current_points[i].z - g.target_sleep_position.z) < 1:
                next_pos.z = g.target_sleep_position.z

            move_to_pos(i, next_pos)

            # Check if all legs have reached target
            if g.current_points[i] != g.target_sleep_position:
                target_reached = False

        if target_reached:
            sleep_state_state = 2

    # State 2: Detach servos to save power
    elif sleep_state_state == 2:
        detach_servos()
        sleep_state_state = 3


def calibration_state():
    """Put the hexapod in calibration mode."""
    if g.current_state != State.CALIBRATE:
        print('Calibration State')

    g.current_state = State.CALIBRATE

    # Target position for calibration
    target_calibration = Vector3(a1 + 43, 0, a2 + 185)
    in_between_z = -20

    legs_up = True

    # Lifting legs up so the hex is sitting on the ground
    for i in range(6):
        if g.current_points[i].z < in_between_z:
            legs_up = False
            next_z = lerp(g.current_points[i].z, in_between_z + 2, 0.03)
            move_to_pos(i, Vector3(g.current_points[i].x, g.current_points[i].y, next_z))

    if legs_up:
        # If connected to the controller, use its offsets
        set_offsets_from_controller_data()

        # Move legs to calibration position
        for i in range(6):
            next_x = min(g.current_points[i].x + 5, target_calibration.x)
            next_y = min(g.current_points[i].y + 5, target_calibration.y)
            next_z = min(g.current_points[i].z + 5, target_calibration.z)
            move_to_pos(i, Vector3(next_x, next_y, next_z))


# AM - checked
# Save offsets to the servo_offsets.txt used by load_raw_offsets_from_eeprom
def save_offsets():
    print('Saving rawOffsets to EEPROM (servo_offsets.txt). ', end='')
    with open('servo_offsets.txt', 'w') as f:
        for i in range(18):
            f.write(f'{g.raw_offsets[i]}\n')
    print('Done')


# AM - checked
def update_offset_variables():
    # updating Vector3 offsets[]
    for i in range(6):
        g.offsets[i] = Vector3(
            g.raw_offsets[i * 3] + g.base_offset.x,
            g.raw_offsets[i * 3 + 1] + g.base_offset.y,
            g.raw_offsets[i * 3 + 2] + g.base_offset.z,
        )

    # updating hex_data.offsets[18]
    for i in range(18):
        hex_settings_data.offsets[i] = g.raw_offsets[i]


# AM - checked
def set_offsets_from_controller_data():
    # don't set offsets data if the controller isn't connected
    if rc_settings_data.offsets[0] == -128 or not g.connected:
        return

    print_raw_offsets()
    for i in range(18):
        g.raw_offsets[i] = rc_settings_data.offsets[i]

    update_offset_variables()


# AM - checked
def print_raw_offsets():
    return  # This function is disabled in the original code
