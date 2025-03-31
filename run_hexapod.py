#!./.venv/bin/python

from matplotlib import pyplot as plt
from globals import PRESSED, Gait, g, PackageType
from hexapod_initializations import setup_sim_legs, a1, a2, a3
from models import HexapodModel
from plotting import plot_hexapod, update_hexapod_plot

import hexapod_main
from nrf import rc_control_data, rc_settings_data

import ps5_controller

# drqp_front_offset = 0.116924  # x offset for the front and back legs
# drqp_side_offset = 0.063871  # y offset fo the front and back legs
# drqp_middle_offset = 0.103  # x offset for the middle legs

# drqp_coxa = 0.053
# drqp_femur = 0.066225
# drqp_tibia = 0.123

# hexapod = HexapodModel(
#     coxa_len=drqp_coxa * 1000,
#     femur_len=drqp_femur * 1000,
#     tibia_len=drqp_tibia * 1000,
#     front_offset=drqp_front_offset * 1000,
#     middle_offset=drqp_middle_offset * 1000,
#     side_offset=drqp_side_offset * 1000,
#     leg_rotation=[0, 0, g.leg_placement_angle],
# )

hexapod = HexapodModel(
    coxa_len=a1,
    femur_len=a2,
    tibia_len=a3,
    front_offset=85,
    middle_offset=100,
    side_offset=55,
    leg_rotation=[0, 0, g.leg_placement_angle],
)
setup_sim_legs(hexapod)
# hexapod.forward_kinematics(0.0, 81.19264931247422, 137.66638455325148)

rc_control_data.gait = Gait.TRI
hexapod_main.setup()

# g.current_type = PackageType.RC_SETTINGS_DATA
# rc_settings_data.calibrating = 1
# hexapod_main.loop()

g.current_type = PackageType.RC_CONTROL_DATA
hexapod_main.loop()

fig, ax, plot_data = plot_hexapod(hexapod, feet_trails_frames=60)

frame = 0
while plt.get_fignums(): # window(s) open
    g.current_type = PackageType.RC_CONTROL_DATA
    #
    # rc_control_data.joy1_Button = PRESSED
    # rc_control_data.joy1_X = 127 + 30 # x>127 == right, x<127 == left
    # rc_control_data.joy1_Y = 127 - 30 # y>127 == backwards, y<127 == forwards
    # rc_control_data.joy2_X = 127 + 30

    # rc_control_data.joy2_Y = 127 + 30 # unused, causes crash if used without joy2_X

    ps5_controller.update(rc_control_data)
    hexapod_main.loop()
    update_hexapod_plot(hexapod, plot_data)

    plt.show(block=False)
    plt.pause(0.001)
    frame += 1

ps5_controller.close()
