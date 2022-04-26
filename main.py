import cv2
import robot_arm
import ext_renderings
import sys
from pyqtgraph.Qt import QtWidgets


# Main Testing Loop
def main():
    # ---- Create OpenGL Window -----
    app = QtWidgets.QApplication(sys.argv)  # Truthfully, don't know why we need this
    window = ext_renderings.createWindow()

    # ---- Plot Table and Net ----
    ext_renderings.renderTable(window)

    # --------- Create Arm ------------
    arm = robot_arm.RobotArm(window)

    # ------ For Testing ------
    # Rebound Goal
    arm.solverIK.updatePaddleRebound([95, 30, 0])

    while True:

        #  ----- Just Some Testing -----
        arm.solverIK.computeAndUpdate(20, 15, 12)

        # Plot Target
        arm.solverIK.plotTarget()
        arm.solverIK.plotReboundGoal()
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
