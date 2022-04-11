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
    solver = fabrik_solver.FabrikSolver(arm, w)
    pos = 0
    movePos = True

    while True:

        #  ----- Just Some Testing -----
        solver.computeAndUpdate(20, 15, 12)

        # Rebound Goal
        solver.updatePaddleRebound([95, 30, 0])
        print(pos)

        # Plot Target
        solver.plotTarget()
        solver.plotReboundGoal()

        # Increase Pos
        if movePos:
            pos += 1
            if pos == 180:
                movePos = False
        else:
            pos -= 1
            if pos == 0:
                movePos = True

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
