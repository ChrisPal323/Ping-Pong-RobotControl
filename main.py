import math
import random
import sys
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
from collections import deque
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
    basePlate = 12  # in inches
    armBasePlate = gl.GLGridItem()  # Create base
    armBasePlate.setColor((255, 0, 255, 255))
    armBasePlate.translate(-10.5 - (basePlate / 2), (ytable / 2), 0)  # Move to correct coord
    armBasePlate.setSize(basePlate, basePlate)  # Size table
    w.addItem(armBasePlate)

    # -- Create Base Stands --
    baseStandVerts = np.array([[1, 0, 0],  # 0
                                [0, 0, 0],  # 1
                                [0, 1, 0],  # 2
                                [0, 0, 1],  # 3
                                [1, 1, 0],  # 4
                                [1, 1, 1],  # 5
                                [0, 1, 1],  # 6
                                [1, 0, 1]])  # 7

    baseStandFaces = np.array([[1, 0, 7], [1, 3, 7],
                                [1, 2, 4], [1, 0, 4],
                                [1, 2, 6], [1, 3, 6],
                                [0, 4, 5], [0, 7, 5],
                                [2, 4, 5], [2, 6, 5],
                                [3, 6, 5], [3, 7, 5]])

    colors = np.array([[1, 0, 0, 1] for i in range(12)])

    # Create same stands
    baseStand1 = gl.GLMeshItem(vertexes=baseStandVerts, faces=baseStandFaces, faceColors=colors,
                               drawEdges=True, edgeColor=(0, 0, 0, 1))
    baseStand2 = gl.GLMeshItem(vertexes=baseStandVerts, faces=baseStandFaces, faceColors=colors,
                               drawEdges=True, edgeColor=(0, 0, 0, 1))

    # Translate stands

    # Add to frame
    w.addItem(baseStand1)
    w.addItem(baseStand2)

    return baseStand1


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
