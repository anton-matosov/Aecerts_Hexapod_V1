#!./.venv/bin/python

from matplotlib import pyplot as plt
from hexapod_initializations import setup_sim_legs, a1, a2, a3
from models import HexapodModel
from plotting import plot_hexapod, update_hexapod_plot

import hexapod_main

hexapod = HexapodModel(coxa_len=a1, femur_len=a2, tibia_len=a3)
setup_sim_legs(hexapod)
hexapod.forward_kinematics(0.0, 81.19264931247422, 137.66638455325148)
fig, ax, plot_data = plot_hexapod(hexapod)

hexapod_main.setup()
while True:
    hexapod_main.loop()
    update_hexapod_plot(hexapod, plot_data)
    plt.show(block=False)
    plt.pause(0.01)
