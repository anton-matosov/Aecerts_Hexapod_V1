# Python equivalent of NRF.h
from enum import Enum
from bezier import Vector2


# Enum for package types
class PackageType(Enum):
    RC_CONTROL_DATA = 0
    RC_SETTINGS_DATA = 1
    HEXAPOD_SETTINGS_DATA = 2
    HEXAPOD_SENSOR_DATA = 3


# Data package classes
class RC_Control_Data_Package:
    def __init__(self):
        self.type = PackageType.RC_CONTROL_DATA.value
        self.joy1_X = 127
        self.joy1_Y = 127
        self.joy2_X = 127
        self.joy2_Y = 127
        self.slider1 = 0
        self.slider2 = 0

        # Bit flags
        self.joy1_Button = 0
        self.joy2_Button = 0
        self.pushButton1 = 0
        self.pushButton2 = 0
        self.idle = 0
        self.sleep = 0
        self.dynamic_stride_length = 0

        self.gait = 0


class RC_Settings_Data_Package:
    def __init__(self):
        self.type = PackageType.RC_SETTINGS_DATA.value
        self.calibrating = 0
        self.offsets = [0] * 18


class Hexapod_Settings_Data_Package:
    def __init__(self):
        self.type = PackageType.HEXAPOD_SETTINGS_DATA.value
        self.offsets = [0] * 18


class Hexapod_Sensor_Data_Package:
    def __init__(self):
        self.type = PackageType.HEXAPOD_SENSOR_DATA.value
        self.current_sensor_value = 0.0
        self.foot_positions = [Vector2(0, 0) for _ in range(6)]


# Global instances
rc_control_data = RC_Control_Data_Package()
rc_settings_data = RC_Settings_Data_Package()
hex_settings_data = Hexapod_Settings_Data_Package()
hex_sensor_data = Hexapod_Sensor_Data_Package()
rc_send_interval = 50  # milliseconds


# NRF communication functions
def setup_nrf():
    """Initialize the NRF24L01 radio module"""
    print('Setting up NRF24L01 radio')
    # Implementation would depend on the Python library used for NRF24L01


def send_nrf_data(package_type):
    """Send data over NRF24L01 radio"""
    if package_type == PackageType.RC_CONTROL_DATA:
        # Pack and send rc_control_data
        pass
    elif package_type == PackageType.RC_SETTINGS_DATA:
        # Pack and send rc_settings_data
        pass
    elif package_type == PackageType.HEXAPOD_SETTINGS_DATA:
        # Pack and send hex_settings_data
        pass
    elif package_type == PackageType.HEXAPOD_SENSOR_DATA:
        # Pack and send hex_sensor_data
        pass
