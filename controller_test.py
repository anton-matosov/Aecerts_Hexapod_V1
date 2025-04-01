# # https://pypi.org/project/pydualsense/ for additional libs install
# #
# # macOS
# # brew install hidapi
# #
# # linux:
# # sudo apt install libhidapi-dev
# from pydualsense import pydualsense, TriggerModes

# def cross_pressed(state):
#     print(state)

# ds = pydualsense() # open controller
# ds.init() # initialize controller

# ds.cross_pressed += cross_pressed
# ds.light.setColorI(0,255,0) # set touchpad color to red
# # ds.triggerL.setMode(TriggerModes.Rigid)
# ds.triggerL.setMode(TriggerModes.Pulse)
# ds.triggerL.setForce(1, 255)
# ds.close() # closing the controller



# from pydualsense import *


# def cross_down(state):
#     print(f'cross {state}')


# def circle_down(state):
#     print(f'circle {state}')


# def dpad_down(state):
#     print(f'dpad {state}')


# def joystick(stateX, stateY):
#     print(f'lj {stateX} {stateY}')


# def gyro_changed(pitch, yaw, roll):
#     print(f'{pitch}, {yaw}, {roll}')


# # create dualsense
# dualsense = pydualsense()
# # find device and initialize
# dualsense.init()

# # add events handler functions
# dualsense.cross_pressed += cross_down
# dualsense.circle_pressed += circle_down
# dualsense.dpad_down += dpad_down
# dualsense.left_joystick_changed += joystick
# dualsense.gyro_changed += gyro_changed

# # read controller state until R1 is pressed
# while not dualsense.state.R1:
#     ...

# # close device
# dualsense.close()



from pydualsense import *

# get dualsense instance
dualsense = pydualsense()
# initialize controller and connect
dualsense.init()

print('Trigger Effect demo started')

# set left and right rumble motors
dualsense.setLeftMotor(255)
dualsense.setRightMotor(100)

# set left l2 trigger to Rigid and set index 1 to force 255
dualsense.triggerL.setMode(TriggerModes.Rigid)
dualsense.triggerL.setForce(1, 255)

# set left r2 trigger to Rigid
dualsense.triggerR.setMode(TriggerModes.Pulse_A)
dualsense.triggerR.setForce(0, 200)
dualsense.triggerR.setForce(1, 255)
dualsense.triggerR.setForce(2, 175)

# loop until r1 is pressed to feel effect
while not dualsense.state.R1:
    ...

# terminate the thread for message and close the device
dualsense.close()
