#!./.venv/bin/python

from matplotlib import pyplot as plt
from models import HexapodModel
from plotting import plot_hexapod

import hexapod_main

hexapod = HexapodModel()
hexapod.forward_kinematics(0, 0, 90)
fig, ax, plot_data = plot_hexapod(hexapod)

hexapod_main.setup()
while True:
    hexapod_main.loop()
    plt.show(block=False)
    plt.pause(0.01)
