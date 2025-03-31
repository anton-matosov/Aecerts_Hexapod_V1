import math
import time

from bezier import constrain, map_float, Vector2
from globals import (
    PRESSED,
    Gait,
    g,
    PackageType,
    State,
    milliseconds,
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
    save_offsets,
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
    g.elapsed_time = milliseconds() - g.loop_start_time
    g.loop_start_time = milliseconds()

    g.connected = get_send_nrf_data()

    if not g.connected:
        sleep_state()
        return

    # Process data based on type
    if g.current_type == PackageType.RC_CONTROL_DATA:
        process_control_data(rc_control_data)
    if g.current_type == PackageType.RC_SETTINGS_DATA:
        process_settings_data(rc_settings_data)


# AM - checked
def process_control_data(data: RC_Control_Data_Package):
    g.dynamic_stride_length = data.dynamic_stride_length

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

    g.joy1_target_vector = Vector2(joy1x, joy1y)
    g.joy1_target_magnitude = constrain(math.hypot(joy1x, joy1y), 0, 100)

    g.joy2_target_vector = Vector2(joy2x, joy2y)
    g.joy2_target_magnitude = constrain(math.hypot(joy2x, joy2y), 0, 100)

    # Process height adjustment
    g.target_distance_from_ground = g.distance_from_ground_base + (data.slider2 * -1.7)
    g.distance_from_ground = lerp(g.distance_from_ground, g.target_distance_from_ground, 0.04)
    if g.distance_from_ground >= 0:
        g.distance_from_ground = g.target_distance_from_ground

    # Smooth joystick movements
    g.joy1_current_vector = lerp(g.joy1_current_vector, g.joy1_target_vector, 0.08)
    g.joy1_current_magnitude = lerp(g.joy1_current_magnitude, g.joy1_target_magnitude, 0.08)

    g.joy2_current_vector = lerp(g.joy2_current_vector, g.joy2_target_vector, 0.12)
    g.joy2_current_magnitude = lerp(g.joy2_current_magnitude, g.joy2_target_magnitude, 0.12)

    g.previous_gait = g.current_gait
    g.current_gait = Gait(data.gait)

    # Drive
    if abs(g.joy1_current_magnitude) >= 10 or abs(g.joy2_current_magnitude) >= 10:
        from hexapod_state import car_state

        car_state()
        g.time_of_last_input = milliseconds()
        return

    # Idle from hexapod
    if abs(g.time_of_last_input - milliseconds()) > 500:
        g.time_of_last_input = milliseconds()  # AM - added
        standing_state()
        return

    # Attack
    if data.joy1_Button == PRESSED and g.attack_cooldown == 0:
        print('slam attack')
        reset_movement_vectors()
        slam_attack()
        standing_state()
        g.attack_cooldown = 50
        g.loop_start_time = milliseconds()
        return
    else:
        g.attack_cooldown = max(g.attack_cooldown - g.elapsed_time, 0)


# AM - checked
def process_settings_data(data):
    g.current_type = (
        PackageType.HEXAPOD_SETTINGS_DATA
    )  # when settings data is being processed, always send settings data back

    if data.calibrating == 1:
        calibration_state()
        return

    # finished calibrating, save offsets
    if g.current_state == State.CALIBRATE:
        save_offsets()

    sleep_state()


# AM - checked
def reset_movement_vectors():
    g.joy1_current_vector = Vector2(0, 0)
    g.joy1_current_magnitude = 0

    g.joy2_current_vector = Vector2(0, 0)
    g.joy2_current_magnitude = 0


# AM - checked
# This is a better implementation made by AI
def load_raw_offsets_from_eeprom():
    """Load servo calibration offsets from persistent storage."""
    print('Loading calibration data...')
    try:
        # In Python, you might use a file or database instead of EEPROM
        with open('servo_offsets.txt', 'r') as f:
            for i, line in enumerate(f):
                if i < 18:  # 18 servos
                    g.raw_offsets[i] = float(line.strip())
    except FileNotFoundError:
        print('No calibration data found, using defaults')
        g.raw_offsets = [0] * 18


# AM - checked
def print_connected_status():
    print(f'Connected: {"TRUE" if g.connected else "FALSE"}')
