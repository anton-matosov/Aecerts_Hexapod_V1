# https://pypi.org/project/pydualsense/ for additional libs install
#
# macOS
# brew install hidapi
#
# linux:
# # sudo apt install libhidapi-dev
from pydualsense import pydualsense, TriggerModes
from bezier import map_float
from globals import UNPRESSED, PRESSED

ds = pydualsense()  # open controller
ds.init()  # initialize controller

ds.light.setColorI(0, 0, 0)

def map_stick(stick_value: float):
    return map_float(stick_value, -128, 128, 0, 255)

def map_button(button_value: bool):
    return PRESSED if button_value else UNPRESSED

def update(rc_control_data):
    rc_control_data.joy1_X = map_stick(ds.state.LX)
    rc_control_data.joy1_Y = map_stick(ds.state.LY)
    rc_control_data.joy2_X = map_stick(ds.state.RX)
    # rc_control_data.joy2_Y = map_stick(ds.state.RY)# unused, causes crash if used without joy2_X

    # rc_control_data.slider1 = ds.state.L2
    # rc_control_data.slider2 = ds.state.R2

    rc_control_data.pushButton1 = map_button(ds.state.L1)
    rc_control_data.pushButton2 = map_button(ds.state.R1)

    rc_control_data.joy1_Button = map_button(ds.state.cross)
    rc_control_data.joy2_Button = map_button(ds.state.circle)

    # rc_control_data.gait = ds.state.dpad_down
    # rc_control_data.dynamic_stride_length = ds.state.triangle

def close():
    ds.close()  # closing the controller
