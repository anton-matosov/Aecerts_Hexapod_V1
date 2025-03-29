import math
import time
from enum import Enum
from bezier import *
from hexapod_state import *

class State(Enum):
    INITIALIZE = 0
    STAND = 1
    CAR = 2
    CRAB = 3
    CALIBRATE = 4
    SLAM_ATTACK = 5
    SLEEP = 6
    ATTACH = 7

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

# RC data classes
class RC_Control_Data_Package:
    def __init__(self):
        self.joy1_X = 127
        self.joy1_Y = 127
        self.joy2_X = 127
        self.joy2_Y = 127
        self.joy1_Button = UNPRESSED
        self.slider1 = 50
        self.slider2 = 50
        self.sleep = 0
        self.idle = 0
        self.gait = 0
        self.dynamic_stride_length = True

class RC_Settings_Data_Package:
    def __init__(self):
        self.calibrating = 0

# Initialize global instances
rc_control_data = RC_Control_Data_Package()
rc_settings_data = RC_Settings_Data_Package()

def setup():
    print("Initializing...")
    attach_servos()
    rc_setup()
    load_raw_offsets_from_eeprom()
    state_initialize()

def loop():
    global elapsed_time, loop_start_time, connected
    
    elapsed_time = time.time() * 1000 - loop_start_time
    loop_start_time = time.time() * 1000
    
    connected = get_send_nrf_data()
    
    if not connected:
        sleep_state()
        return
    
    # In a real implementation, we'd have a way to determine which type of data was received
    # For this example, we'll just process control data
    process_control_data(rc_control_data)

def process_control_data(data):
    global dynamic_stride_length, joy1_target_vector, joy1_target_magnitude
    global joy2_target_vector, joy2_target_magnitude, target_distance_from_ground
    global distance_from_ground, distance_from_center, joy1_current_vector
    global joy1_current_magnitude, joy2_current_vector, joy2_current_magnitude
    global previous_gait, current_gait, time_since_last_input, attack_cooldown
    
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
    joy1_target_magnitude = constrain(math.sqrt(joy1x**2 + joy1y**2), 0, 100)
    
    joy2_target_vector = Vector2(joy2x, joy2y)
    joy2_target_magnitude = constrain(math.sqrt(joy2x**2 + joy2y**2), 0, 100)
    
    # Process height adjustment
    target_distance_from_ground = distance_from_ground_base + (data.slider2 * -1.7)
    distance_from_ground = lerp(distance_from_ground, target_distance_from_ground, 0.04)
    if distance_from_ground >= 0:
        distance_from_ground = target_distance_from_ground
    
    distance_from_center = 170
    
    # Smooth joystick movements
    joy1_current_vector = lerp_vector2(joy1_current_vector, joy1_target_vector, 0.08)
    joy1_current_magnitude = lerp(joy1_current_magnitude, joy1_target_magnitude, 0.08)
    
    joy