#!./.venv/bin/python

from matplotlib import pyplot as plt
from globals import PRESSED, Gait
from hexapod_initializations import setup_sim_legs, a1, a2, a3
from models import HexapodModel
from plotting import plot_hexapod, update_hexapod_plot
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection

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

frames_to_collect = 60
colors = np.linspace(1, 0, frames_to_collect)

leg_tips = {}
leg_tip_collections = {}

frame = 0
while plt.get_fignums(): # window(s) open
    # rc_control_data.joy1_Button = PRESSED
    # rc_control_data.joy1_X = 127 + 30 # forward
    rc_control_data.joy1_Y = 127 + 30 # right
    # rc_control_data.joy2_X = 127 + 30

    # rc_control_data.joy2_Y = 127 + 30 # unused, causes crash if used without joy2_X

    hexapod_main.loop()
    update_hexapod_plot(hexapod, plot_data)

    for leg in hexapod.legs:
        if leg.label not in leg_tips:
            leg_tips[leg.label] = []
        if len(leg_tips[leg.label]) > frames_to_collect:
            leg_tips[leg.label].pop(0)
        leg_tips[leg.label].append(leg.tibia_end.numpy())

        points = np.array(leg_tips[leg.label])

        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        segments = segments.reshape(-1, 2, 3)

        if leg.label not in leg_tip_collections:
            lc = Line3DCollection(segments, cmap='plasma')
            leg_tip_collections[leg.label] = lc
            lc.set_array(colors)
            lc.set_linewidth(2)
            line = ax.add_collection(lc)
        else:
            lc = leg_tip_collections[leg.label]
            lc.set_segments(segments)

    plt.show(block=False)
    plt.pause(0.001)
    frame += 1
