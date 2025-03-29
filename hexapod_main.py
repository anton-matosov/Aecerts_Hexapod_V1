import math
import time

from bezier import constrain, map_float, print_value, Vector2, Vector3
from globals import (
    PRESSED,
    connected,
    current_gait,
    current_type,
    distance_from_ground,
    distance_from_ground_base,
    joy1_current_magnitude,
    joy1_current_vector,
    joy2_current_magnitude,
    joy2_current_vector,
    loop_start_time,
    offsets,
    PackageType,
    raw_offsets,
    State,
    current_state,
    attack_cooldown,
)

from hexapod_attacks import slam_attack
from nrf import (
    RC_Control_Data_Package,
    rc_control_data,
    rc_settings_data,
    hex_sensor_data,
)

from hexapod_initializations import (
    a1,
    a2,
    a3,
    coxa1,
    coxa2,
    coxa3,
    coxa4,
    coxa5,
    coxa6,
    current_points,
    cycle_start_points,
    femur1,
    femur2,
    femur3,
    femur4,
    femur5,
    femur6,
    leg_length,
    servos_attached,
    tibia1,
    tibia2,
    tibia3,
    tibia4,
    tibia5,
    tibia6,
)
from hexapod_state import (
    attach_servos,
    calibration_state,
    get_send_nrf_data,
    lerp,
    lerp_vector2,
    rc_setup,
    sleep_state,
    standing_state,
    state_initialize,
)


# AM - checked
def setup():
    print('Initializing...')
    attach_servos()
    rc_setup()
    load_raw_offsets_from_eeprom()
    state_initialize()


# AM - checked
def loop():
    global elapsed_time, loop_start_time, connected

    elapsed_time = time.time() * 1000 - loop_start_time
    loop_start_time = time.time() * 1000

    connected = get_send_nrf_data()

    if not connected:
        sleep_state()
        return

    # Process data based on type
    if current_type == PackageType.RC_CONTROL_DATA:
        process_control_data(rc_control_data)
    if current_type == PackageType.RC_SETTINGS_DATA:
        process_settings_data(rc_settings_data)


# AM - checked
def process_control_data(data: RC_Control_Data_Package):
    global dynamic_stride_length, joy1_target_vector, joy1_target_magnitude
    global joy2_target_vector, joy2_target_magnitude, target_distance_from_ground
    global distance_from_ground, distance_from_center, joy1_current_vector
    global joy1_current_magnitude, joy2_current_vector, joy2_current_magnitude
    global previous_gait, current_gait, time_since_last_input, attack_cooldown, loop_start_time

    dynamic_stride_length = data.dynamic_stride_length

    # Sleep mode
    if data.sleep == 1:
        sleep_state()
        return

    # Idle mode
    if data.idle == 1:
        standing_state()
        return

    # Process joystick inputs
    joy1x = map_float(data.joy1_X, 0, 254, -100, 100)
    joy1y = map_float(data.joy1_Y, 0, 254, -100, 100)
    joy2x = map_float(data.joy2_X, 0, 254, -100, 100)
    joy2y = map_float(data.joy2_Y, 0, 254, -100, 100)

    joy1_target_vector = Vector2(joy1x, joy1y)
    joy1_target_magnitude = constrain(math.hypot(joy1x, joy1y), 0, 100)

    joy2_target_vector = Vector2(joy2x, joy2y)
    joy2_target_magnitude = constrain(math.hypot(joy2x, joy2y), 0, 100)

    # Process height adjustment
    target_distance_from_ground = distance_from_ground_base + (data.slider2 * -1.7)
    distance_from_ground = lerp(distance_from_ground, target_distance_from_ground, 0.04)
    if distance_from_ground >= 0:
        distance_from_ground = target_distance_from_ground

    distance_from_center = 170

    # Smooth joystick movements
    joy1_current_vector = lerp_vector2(joy1_current_vector, joy1_target_vector, 0.08)

    joy2_current_vector = lerp_vector2(joy2_current_vector, joy2_target_vector, 0.12)
    joy2_current_magnitude = lerp(joy2_current_magnitude, joy2_target_magnitude, 0.12)

    previous_gait = current_gait
    current_gait = data.gait

    # Drive
    if abs(joy1_current_magnitude) >= 10 or abs(joy2_current_magnitude) >= 10:
        from hexapod_state import car_state

        car_state()
        time_since_last_input = time.time() * 1000
        return

    # Idle from hexapod
    if abs(time_since_last_input - time.time() * 1000) > 5:
        standing_state()
        return

    # Attack
    if data.joy1_Button == PRESSED and attack_cooldown == 0:
        print('slam attack')
        reset_movement_vectors()
        slam_attack()
        standing_state()
        attack_cooldown = 50
        loop_start_time = time.time() * 1000
        return
    else:
        attack_cooldown = max(attack_cooldown - elapsed_time, 0)


# AM - checked
def process_settings_data(data):
    global send_type, current_state
    send_type = (
        PackageType.HEXAPOD_SETTINGS_DATA
    )  # when settings data is being processed, always send settings data back

    if data.calibrating == 1:
        calibration_state()
        return

    # finished calibrating, save offsets
    if current_state == State.CALIBRATE:
        save_offsets()

    sleep_state()


# AM - checked
def reset_movement_vectors():
    global joy1_current_vector, joy1_current_magnitude, joy2_current_vector, joy2_current_magnitude

    joy1_current_vector = Vector2(0, 0)
    joy1_current_magnitude = 0

    joy2_current_vector = Vector2(0, 0)
    joy2_current_magnitude = 0


# AM - checked
# merge of 2 setters to make it pythonic
def set_cycle_start_points(leg=None):
    global cycle_start_points, current_points

    if leg is not None:
        cycle_start_points[leg] = current_points[leg]
    else:
        for i in range(6):
            cycle_start_points[i] = current_points[i]


# AM - checked
def angle_to_microseconds(angle):
    val = 500.0 + (((2500.0 - 500.0) / 180.0) * angle)
    return int(val)


# AM - checked
def rotate_to_angle(leg, target_rot):
    global servos_attached

    if not servos_attached:
        attach_servos()

    coxa_microseconds = angle_to_microseconds(target_rot.x)
    femur_microseconds = angle_to_microseconds(target_rot.y)
    tibia_microseconds = angle_to_microseconds(target_rot.z)

    if leg == 0:
        coxa1.write_microseconds(coxa_microseconds)
        femur1.write_microseconds(femur_microseconds)
        tibia1.write_microseconds(tibia_microseconds)
    elif leg == 1:
        coxa2.write_microseconds(coxa_microseconds)
        femur2.write_microseconds(femur_microseconds)
        tibia2.write_microseconds(tibia_microseconds)
    elif leg == 2:
        coxa3.write_microseconds(coxa_microseconds)
        femur3.write_microseconds(femur_microseconds)
        tibia3.write_microseconds(tibia_microseconds)
    elif leg == 3:
        coxa4.write_microseconds(coxa_microseconds)
        femur4.write_microseconds(femur_microseconds)
        tibia4.write_microseconds(tibia_microseconds)
    elif leg == 4:
        coxa5.write_microseconds(coxa_microseconds)
        femur5.write_microseconds(femur_microseconds)
        tibia5.write_microseconds(tibia_microseconds)
    elif leg == 5:
        coxa6.write_microseconds(coxa_microseconds)
        femur6.write_microseconds(femur_microseconds)
        tibia6.write_microseconds(tibia_microseconds)


# AM - checked
def move_to_pos(leg, pos):
    global hex_sensor_data, servos_attached, current_points, target_rot

    hex_sensor_data.foot_positions[leg].x = int(pos.x)
    hex_sensor_data.foot_positions[leg].y = int(pos.y)

    if not servos_attached:
        attach_servos()

    current_points[leg] = pos

    dis = Vector3(0, 0, 0).distance_to(pos)
    if dis > leg_length:
        print_value('Point impossible to reach', pos, False)
        print_value('Distance', dis, True)
        return

    x = pos.x
    y = pos.y
    z = pos.z

    o1 = offsets[leg].x
    o2 = offsets[leg].y
    o3 = offsets[leg].z

    theta1 = math.atan2(y, x) * (180 / math.pi) + o1  # base angle
    l = math.sqrt(x * x + y * y)  # x and y extension
    l1 = l - a1
    h = math.sqrt(l1 * l1 + z * z)

    phi1 = math.acos(constrain((h**2 + a2**2 - a3**2) / (2 * h * a2), -1, 1))
    phi2 = math.atan2(z, l1)
    theta2 = (phi1 + phi2) * 180 / math.pi + o2
    phi3 = math.acos(constrain((a2**2 + a3**2 - h**2) / (2 * a2 * a3), -1, 1))
    theta3 = 180 - (phi3 * 180 / math.pi) + o3

    target_rot = Vector3(theta1, theta2, theta3)

    # TODO(AM): use rotateToAngle
    coxa_microseconds = angle_to_microseconds(target_rot.x)
    femur_microseconds = angle_to_microseconds(target_rot.y)
    tibia_microseconds = angle_to_microseconds(target_rot.z)

    if leg == 0:
        coxa1.write_microseconds(coxa_microseconds)
        femur1.write_microseconds(femur_microseconds)
        tibia1.write_microseconds(tibia_microseconds)
    elif leg == 1:
        coxa2.write_microseconds(coxa_microseconds)
        femur2.write_microseconds(femur_microseconds)
        tibia2.write_microseconds(tibia_microseconds)
    elif leg == 2:
        coxa3.write_microseconds(coxa_microseconds)
        femur3.write_microseconds(femur_microseconds)
        tibia3.write_microseconds(tibia_microseconds)
    elif leg == 3:
        coxa4.write_microseconds(coxa_microseconds)
        femur4.write_microseconds(femur_microseconds)
        tibia4.write_microseconds(tibia_microseconds)
    elif leg == 4:
        coxa5.write_microseconds(coxa_microseconds)
        femur5.write_microseconds(femur_microseconds)
        tibia5.write_microseconds(tibia_microseconds)
    elif leg == 5:
        coxa6.write_microseconds(coxa_microseconds)
        femur6.write_microseconds(femur_microseconds)
        tibia6.write_microseconds(tibia_microseconds)


# AM - checked
# This is a better implementation made by AI
def load_raw_offsets_from_eeprom():
    """Load servo calibration offsets from persistent storage."""
    global raw_offsets

    print('Loading calibration data...')
    try:
        # In Python, you might use a file or database instead of EEPROM
        with open('servo_offsets.txt', 'r') as f:
            for i, line in enumerate(f):
                if i < 18:  # 18 servos
                    raw_offsets[i] = float(line.strip())
    except FileNotFoundError:
        print('No calibration data found, using defaults')
        raw_offsets = [0] * 18


# AM - checked
def print_connected_status():
    print(f'Connected: {"TRUE" if connected else "FALSE"}')
