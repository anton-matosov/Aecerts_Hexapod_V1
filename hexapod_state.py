import time
from bezier import constrain, get_point_on_bezier_curve, map_float, Vector2, Vector3
from globals import (
    control_points,
    current_gait,
    current_points,
    current_state,
    cycle_progress,
    cycle_start_points,
    distance_from_center,
    distance_from_ground,
    distance_from_ground_base,
    dynamic_stride_length,
    forward_amount,
    Gait,
    global_rotation_multiplier,
    global_speed_multiplier,
    joy1_current_magnitude,
    joy1_current_vector,
    joy2_current_vector,
    leg_placement_angle,
    leg_states,
    LegState,
    max_stride_length,
    offsets,
    points,
    previous_gait,
    rotate_control_points,
    rotation_multiplier,
    State,
    stride_length_multiplier,
    stride_multiplier,
    t_array,
    turn_amount,
    base_offset,
    raw_offsets,
    connected,
)
from hexapod_control import move_to_pos, set_cycle_start_points
from nrf import (
    rc_settings_data,
    hex_settings_data,
    rc_control_data,
)
from hexapod_initializations import a1, a2, detach_servos, servos_attached

# Standing Control Points Array
SCPA = [[Vector3(0, 0, 0) for _ in range(10)] for _ in range(6)]

# Standing state variables
standing_start_points = [
    Vector3(0, 0, 0) for _ in range(6)
]  # points legs are at when entering standing state
standing_in_between_points = [Vector3(0, 0, 0) for _ in range(6)]  # middle points of bezier curves
standing_end_point = Vector3(0, 0, 0)
current_legs = [-1, -1, -1]
stand_loops = 0
stand_progress = 0
standing_distance_adjustment = 0  # Adjustment for standing height

# Sleep state variables
target_sleep_position = Vector3(130, 0, -46)
sleep_state_state = 1


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

    elif current_gait == Gait.RIPPLE:
        # Offsets
        cycle_progress[0] = 0
        cycle_progress[1] = (points / 6) * 4
        cycle_progress[2] = (points / 6) * 2
        cycle_progress[3] = (points / 6) * 5
        cycle_progress[4] = points / 6
        cycle_progress[5] = (points / 6) * 3

        # Percentage Time On Ground
        push_fraction = 3.2 / 6.0

        speed_multiplier = 1
        stride_length_multiplier = 1.3
        lift_height_multiplier = 1.1
        max_stride_length = 220
        max_speed = 200

    elif current_gait == Gait.BI:
        # Offsets
        cycle_progress[0] = 0
        cycle_progress[1] = points / 3
        cycle_progress[2] = (points / 3) * 2
        cycle_progress[3] = 0
        cycle_progress[4] = points / 3
        cycle_progress[5] = (points / 3) * 2

        # Percentage Time On Ground
        push_fraction = 2.1 / 6.0

        speed_multiplier = 4
        stride_length_multiplier = 1
        lift_height_multiplier = 1.8
        max_stride_length = 230
        max_speed = 130

    elif current_gait == Gait.QUAD:
        # Offsets
        cycle_progress[0] = 0
        cycle_progress[1] = points / 3
        cycle_progress[2] = (points / 3) * 2
        cycle_progress[3] = 0
        cycle_progress[4] = points / 3
        cycle_progress[5] = (points / 3) * 2

        # Percentage Time On Ground
        push_fraction = 4.1 / 6.0

        speed_multiplier = 1
        stride_length_multiplier = 1.2
        lift_height_multiplier = 1.1
        max_stride_length = 220
        max_speed = 200

    elif current_gait == Gait.HOP:
        # Offsets
        cycle_progress[0] = 0
        cycle_progress[1] = 0
        cycle_progress[2] = 0
        cycle_progress[3] = 0
        cycle_progress[4] = 0
        cycle_progress[5] = 0

        # Percentage Time On Ground
        push_fraction = 3 / 6.0

        speed_multiplier = 1
        stride_length_multiplier = 1.6
        lift_height_multiplier = 2.5
        max_stride_length = 240
        max_speed = 200

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


# AM - checked
def lerp(a, b, f):
    """
    Linear interpolation between a and b values.

    Args:
        a: Starting value
        b: Ending value
        f: Interpolation factor (0.0 to 1.0)

    Returns:
        Interpolated value

    """
    return a * (1.0 - f) + (b * f)


# AM - checked
def lerp_vector2(a, b, f):
    """
    Linear interpolation between two Vector2 objects.

    Args:
        a: Starting Vector2
        b: Ending Vector2
        f: Interpolation factor (0.0 to 1.0)

    Returns:
        Interpolated Vector2

    """
    return Vector2(lerp(a.x, b.x, f), lerp(a.y, b.y, f))


# AM - checked
def lerp_vector3(a, b, f):
    """
    Linear interpolation between two Vector3 objects.

    Args:
        a: Starting Vector3
        b: Ending Vector3
        f: Interpolation factor (0.0 to 1.0)

    Returns:
        Interpolated Vector3

    """
    return Vector3(lerp(a.x, b.x, f), lerp(a.y, b.y, f), lerp(a.z, b.z, f))


# AM - checked Hexapod_Code/Initialization_State.ino
def state_initialize():
    move_to_pos(0, Vector3(160, 0, 0))
    move_to_pos(1, Vector3(160, 0, 0))
    move_to_pos(2, Vector3(160, 0, 0))
    move_to_pos(3, Vector3(160, 0, 0))
    move_to_pos(4, Vector3(160, 0, 0))
    move_to_pos(5, Vector3(160, 0, 0))

    time.sleep(25 / 1000)

    move_to_pos(0, Vector3(225, 0, 115))
    move_to_pos(1, Vector3(225, 0, 115))
    move_to_pos(2, Vector3(225, 0, 115))
    move_to_pos(3, Vector3(225, 0, 115))
    move_to_pos(4, Vector3(225, 0, 115))
    move_to_pos(5, Vector3(225, 0, 115))

    time.sleep(50 / 1000)


# AM - checked, stub
def get_send_nrf_data():
    """
    Exchange data with the remote control.

    Returns:
        bool: True if connection is active, False otherwise

    """
    global rc_control_data

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
    global current_state, stand_loops, stand_progress, standing_end_point, current_legs

    if current_state != State.STAND:
        print('Standing State')

    move_all_at_once = False
    high_lift = False
    set_cycle_start_points()
    # Use the standing_distance_adjustment variable here
    standing_end_point = Vector3(
        distance_from_center, 0, distance_from_ground + standing_distance_adjustment
    )
    stand_loops = 2

    # Set flags based on current state
    if (
        current_state == State.CALIBRATE
        or current_state == State.INITIALIZE
        or current_state == State.SLAM_ATTACK
        or current_state == State.SLEEP
        or current_state == State.ATTACH
    ):
        move_all_at_once = True

    if current_state == State.SLAM_ATTACK or current_state == State.SLEEP:
        high_lift = True

    if current_state != State.STAND:
        set_3_highest_leg()
        stand_loops = 0
        stand_progress = 0

        # Copy current points to standing start points
        for i in range(6):
            standing_start_points[i] = current_points[i]

        current_state = State.STAND

        # Calculate the in-between and ending points
        for i in range(6):
            in_between_point = Vector3(
                standing_start_points[i].x, standing_start_points[i].y, standing_start_points[i].z
            )
            in_between_point.x = (in_between_point.x + standing_end_point.x) / 1.5
            in_between_point.y = (in_between_point.y + standing_end_point.y) / 1.5

            in_between_point.z = (in_between_point.z + standing_end_point.z) / 2
            if abs(in_between_point.z - standing_end_point.z) < 50:
                in_between_point.z += 70
            if high_lift:
                in_between_point.z += 80

            standing_in_between_points[i] = in_between_point

            SCPA[i][0] = standing_start_points[i]
            SCPA[i][1] = standing_in_between_points[i]
            SCPA[i][2] = standing_end_point

        for i in range(6):
            leg_states[i] = LegState.STANDING

    # Update distance from ground constantly
    for i in range(6):
        SCPA[i][2] = standing_end_point

    # Readjusting - takes about a second
    while stand_loops < 2:
        stand_progress += 20

        t = float(stand_progress) / points
        if t > 1:
            t = 1

        if move_all_at_once:
            for i in range(6):
                move_to_pos(i, get_point_on_bezier_curve(SCPA[i][:3], 3, t))

            if stand_progress > points:
                stand_progress = 0
                stand_loops = 2
        else:
            for i in range(3):
                if current_legs[i] != -1:
                    move_to_pos(
                        current_legs[i], get_point_on_bezier_curve(SCPA[current_legs[i]][:3], 3, t)
                    )

            if stand_progress > points:
                stand_progress = 0
                stand_loops += 1
                set_3_highest_leg()

    # Constantly move to the standing end position
    for i in range(6):
        move_to_pos(i, get_point_on_bezier_curve(SCPA[i][:3], 3, 1))


def set_3_highest_leg():
    """Select the three legs with highest z-position to move first."""
    global current_legs

    current_legs[0] = -1
    current_legs[1] = -1
    current_legs[2] = -1

    for j in range(3):
        for i in range(6):  # Go through the legs
            # If leg already on the list of current legs, skip it
            if i in current_legs:
                continue

            # If leg already in position, don't add it
            if current_points[i] == standing_end_point:
                continue

            # If leg's z is greater than the leg already there, add it
            if current_legs[j] == -1 or current_points[i].z > current_points[current_legs[j]].z:
                current_legs[j] = i


# AM - checked
def sleep_state():
    """Put the hexapod in a sleep/powered down position."""
    global current_state, sleep_state_state

    if current_state != State.SLEEP:
        print('Sleep State')
        sleep_state_state = 1

    current_state = State.SLEEP

    # Skip if servos are not attached - using the correct variable
    if not servos_attached:
        return

    # State 1: Move legs to target sleep position
    if sleep_state_state == 1:
        target_reached = True
        for i in range(6):
            next_pos = lerp(current_points[i], target_sleep_position, 0.03)

            # Snap to target when very close
            if abs(current_points[i].x - target_sleep_position.x) < 1:
                next_pos.x = target_sleep_position.x
            if abs(current_points[i].y - target_sleep_position.y) < 1:
                next_pos.y = target_sleep_position.y
            if abs(current_points[i].z - target_sleep_position.z) < 1:
                next_pos.z = target_sleep_position.z

            move_to_pos(i, next_pos)

            # Check if all legs have reached target
            if current_points[i] != target_sleep_position:
                target_reached = False

        if target_reached:
            sleep_state_state = 2

    # State 2: Detach servos to save power
    elif sleep_state_state == 2:
        detach_servos()
        sleep_state_state = 3


def calibration_state():
    """Put the hexapod in calibration mode."""
    global current_state, current_points

    if current_state != State.CALIBRATE:
        print('Calibration State')

    current_state = State.CALIBRATE

    # Target position for calibration
    target_calibration = Vector3(a1 + 43, 0, a2 + 185)
    in_between_z = -20

    legs_up = True

    # Lifting legs up so the hex is sitting on the ground
    for i in range(6):
        if current_points[i].z < in_between_z:
            legs_up = False
            next_z = lerp(current_points[i].z, in_between_z + 2, 0.03)
            move_to_pos(i, Vector3(current_points[i].x, current_points[i].y, next_z))

    if legs_up:
        # If connected to the controller, use its offsets
        set_offsets_from_controller_data()

        # Move legs to calibration position
        for i in range(6):
            next_x = min(current_points[i].x + 5, target_calibration.x)
            next_y = min(current_points[i].y + 5, target_calibration.y)
            next_z = min(current_points[i].z + 5, target_calibration.z)
            move_to_pos(i, Vector3(next_x, next_y, next_z))


# AM - checked
# Save offsets to the servo_offsets.txt used by load_raw_offsets_from_eeprom
def save_offsets():
    print('Saving rawOffsets to EEPROM (servo_offsets.txt). ', end='')
    with open('servo_offsets.txt', 'w') as f:
        for i in range(18):
            f.write(f'{raw_offsets[i]}\n')
    print('Done')


# AM - checked
def update_offset_variables():
    global offsets, hex_settings_data

    # updating Vector3 offsets[]
    for i in range(6):
        offsets[i] = Vector3(
            raw_offsets[i * 3] + base_offset.x,
            raw_offsets[i * 3 + 1] + base_offset.y,
            raw_offsets[i * 3 + 2] + base_offset.z,
        )

    # updating hex_data.offsets[18]
    for i in range(18):
        hex_settings_data.offsets[i] = raw_offsets[i]


# AM - checked
def set_offsets_from_controller_data():
    global raw_offsets

    # don't set offsets data if the controller isn't connected
    if rc_settings_data.offsets[0] == -128 or not connected:
        return

    print_raw_offsets()
    for i in range(18):
        raw_offsets[i] = rc_settings_data.offsets[i]

    update_offset_variables()


# AM - checked
def print_raw_offsets():
    return  # This function is disabled in the original code
