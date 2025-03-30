#!./.venv/bin/python

from matplotlib import pyplot as plt
from globals import PRESSED, Gait
from hexapod_initializations import setup_sim_legs, a1, a2, a3
from models import HexapodModel
from plotting import plot_hexapod, update_hexapod_plot

import hexapod_main
from nrf import rc_control_data


hexapod = HexapodModel(
    coxa_len=a1,
    femur_len=a2,
    tibia_len=a3,
    front_offset=85,
    middle_offset=100,
    side_offset=55,
)
setup_sim_legs(hexapod)
hexapod.forward_kinematics(0.0, 81.19264931247422, 137.66638455325148)
fig, ax, plot_data = plot_hexapod(hexapod)

rc_control_data.gait = Gait.TRI
hexapod_main.setup()

leg_tips = {}
collecting_frames_range = (50, 200)
plotted = False
collected = False

frame = 0
while plt.get_fignums(): # window(s) open
    # rc_control_data.joy1_Button = PRESSED
    # rc_control_data.joy1_X = 127 + 30 # forward
    rc_control_data.joy1_Y = 127 + 30 # right
    # rc_control_data.joy2_X = 127 + 30

    # rc_control_data.joy2_Y = 127 + 30 # unused, causes crash if used without joy2_X

    hexapod_main.loop()
    update_hexapod_plot(hexapod, plot_data)

    collected = frame > collecting_frames_range[1]
    if frame in range(*collecting_frames_range):
        for leg in hexapod.legs:
            if leg.label not in leg_tips:
                leg_tips[leg.label] = []
            leg_tips[leg.label].append(leg.tibia_end.numpy())
    elif collected and not plotted:
        plotted = True
        for leg in hexapod.legs:
            ax.plot(*zip(*leg_tips[leg.label]), label=leg.label)

    plt.show(block=False)
    plt.pause(0.001)
    frame += 1
