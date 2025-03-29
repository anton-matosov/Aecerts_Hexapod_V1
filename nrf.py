# Python equivalent of NRF.h
from bezier import Vector2
from globals import PackageType, UNPRESSED, dynamic_stride_length


# AM - checked
# Data package classes
class RC_Control_Data_Package:
    def __init__(self):
        self.type = PackageType.RC_CONTROL_DATA
        self.joy1_X = 127
        self.joy1_Y = 127
        self.joy2_X = 127
        self.joy2_Y = 127
        self.slider1 = 50
        self.slider2 = 50

        # Bit flags
        self.joy1_Button = UNPRESSED
        self.joy2_Button = UNPRESSED
        self.pushButton1 = UNPRESSED
        self.pushButton2 = UNPRESSED
        self.idle = 0
        self.sleep = 0
        self.dynamic_stride_length = dynamic_stride_length

        self.gait = 0


# AM - checked
class RC_Settings_Data_Package:
    def __init__(self):
        self.type = PackageType.RC_SETTINGS_DATA
        self.calibrating = 0
        self.offsets = [0] * 18


class Hexapod_Settings_Data_Package:
    def __init__(self):
        self.type = PackageType.HEXAPOD_SETTINGS_DATA
        self.offsets = [0] * 18


class Hexapod_Sensor_Data_Package:
    def __init__(self):
        self.type = PackageType.HEXAPOD_SENSOR_DATA
        self.current_sensor_value = 0.0
        self.foot_positions = [
            # AM: Its Vector2int in C++, but we are not doing real transmission, so it doesn't matter
            Vector2(0, 0)
            for _ in range(6)
        ]


# AM - checked
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
    print(f'Sending NRF data of type {package_type}')
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
