# https://pypi.org/project/pydualsense/ for additional libs install
#
# macOS
# brew install hidapi
#
# linux:
# # sudo apt install libhidapi-dev
from pydualsense import pydualsense, TriggerModes
from bezier import map_float

ds = pydualsense()  # open controller
ds.init()  # initialize controller

ds.light.setColorI(0, 0, 0)

def update(rc_control_data):
    rc_control_data.joy1_X = map_float(ds.state.LX, -128, 128, 0, 255)
    rc_control_data.joy1_Y = map_float(ds.state.LY, -128, 128, 0, 255)
    rc_control_data.joy2_X = map_float(ds.state.RX, -128, 128, 0, 255)
    # rc_control_data.joy2_Y = map_float(ds.state.RY, -128, 128, 0, 255)
    # rc_control_data.slider1 = ds.state.L2
    # rc_control_data.slider2 = ds.state.R2
    # rc_control_data.pushButton1 = ds.state.L1
    # rc_control_data.pushButton2 = ds.state.R1

    # rc_control_data.joy1_Button = ds.state.cross
    # rc_control_data.joy2_Button = ds.state.circle

    # rc_control_data.gait = ds.state.dpad_down
    # rc_control_data.dynamic_stride_length = ds.state.triangle

def close():
    ds.close()  # closing the controller

    #
    # rc_control_data.joy1_Button = PRESSED
    # rc_control_data.joy1_X = 127 + 30 # x>127 == right, x<127 == left
    # rc_control_data.joy1_Y = 127 - 30 # y>127 == backwards, y<127 == forwards
    # rc_control_data.joy2_X = 127 + 30

    # rc_control_data.joy2_Y = 127 + 30 # unused, causes crash if used without joy2_X
