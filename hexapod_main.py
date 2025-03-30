import math
import time

from bezier import constrain, map_float, Vector2
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
    PackageType,
    raw_offsets,
    State,
    current_state,
    attack_cooldown,
    time_since_last_input,
)

from hexapod_attacks import slam_attack
from nrf import (
    RC_Control_Data_Package,
    rc_control_data,
    rc_settings_data,
)

from hexapod_state import (
    calibration_state,
    get_send_nrf_data,
    lerp,
    lerp_vector2,
    sleep_state,
    standing_state,
    state_initialize,
)

from hexapod_initializations import (
    attach_servos,
)


# AM - checked
def setup():
    print('Initializing...')
    attach_servos()
    # rc_setup() # AM - removed
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
