import sys
import cv2
import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets
import robot_arm
import fabrik_solver

# Numbers that work well (in inches) with the graph and keep proper proportions
xtable = 108
ytable = 60

xnet = 66
ynet = 6  # Kinda like a z coord after 90 degree rotation


# Main Testing Loop
def main():
    # ---- Set Up Window ----
    app = QtWidgets.QApplication(sys.argv)  # Truthfully, don't know why we need this
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

    # --------- Create Arm ------------
    arm = robot_arm.RobotArm(w)

    # ------- Create IK Solver ------
    solver = fabrik_solver.FabrikSolver(arm)
    pos1 = 0
    pos2 = 0
    movePos1 = True
    movePos2 = True

    while True:

        #  ----- Just Some Testing -----
        solver.computeAndUpdate(10, 30 - pos1 * 2, 12)
        #solver.computeAndUpdate(10, 30, 12)
        solver.plotTarget(w)

        # Rotate Arm3
        arm.rotateArm3(90, pos1*30)

        # Increase Pos
        if movePos1:
            pos1 += 0.01
            if pos1 > 15:
                movePos1 = False
        else:
            pos1 -= 0.01
            if pos1 < -15:
                movePos1 = True

        # Increase Pos
        if movePos2:
            pos2 += 0.5
            if pos2 > 15:
                movePos2 = False
        else:
            pos2 -= 0.5
            if pos2 < -15:
                movePos2 = True

        #  ----- Just Some Testing -----

        # detect keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):

            break
        elif key != 255:
            print('KEY PRESS:', [chr(key)])


# Run!
if __name__ == '__main__':
    main()
