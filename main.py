import math
import random
import sys
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
from collections import deque
from scipy.linalg import qr, svd
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui


# Numbers that work well (in inches) with the graph and keep proper proportions
xtable = 108
ytable = 60

xnet = 66
ynet = 6  # Kinda like a z coord after 90 degree rotation


# Init robot arm in frame
def init_arm(w):

    # -- Create Arm Base --
    basePlate = 12
    armBasePlate = gl.GLGridItem()  # Create base
    armBasePlate.setColor((255, 0, 255, 255))
    armBasePlate.translate(-10.5 - (basePlate / 2), (ytable / 2), 0)  # Move to correct coord
    armBasePlate.setSize(basePlate, basePlate)  # Size table
    w.addItem(armBasePlate)

    return armBasePlate


# Main Testing Loop
def main():
    # ---- Set Up Window ----
    app = QtGui.QApplication(sys.argv)  # Truthfully, don't know why we need this
    w = gl.GLViewWidget()
    w.opts['distance'] = 150
    w.setWindowTitle('Ping-Pong Robot-Control')
    w.setGeometry(0, 110, 1280, 720)
    w.show()

    # ---- Plot Table and Net ----
    table = gl.GLGridItem()  # Create table
    table.translate(xtable / 2, ytable / 2, 0)  # Move to correct coord
    table.setSize(xtable, ytable)  # Size table
    table.setSpacing(6, 6)  # Size grid spaces
    w.addItem(table)  # Add table to view

    net = gl.GLGridItem()  # Create net
    net.rotate(90, 1, 0, 0)  # Rotate plain
    net.rotate(90, 0, 0, 1)  # Rotate plain
    net.translate(xtable / 2, ytable / 2, ynet / 2)  # Move to correct pos
    net.setSize(xnet, ynet)  # Size table
    w.addItem(net)  # Add table to view

    # ---- Create Objects to be Moved / Plotted ----

    # Create arm
    arm = init_arm(w)

    while True:

        # update arm


        # detect keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key != 255:
            print('KEY PRESS:', [chr(key)])


# Run!
if __name__ == '__main__':
    main()
