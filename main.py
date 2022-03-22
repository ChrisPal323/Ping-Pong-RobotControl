import sys
import numpy as np
import cv2
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets

# Numbers that work well (in inches) with the graph and keep proper proportions
xtable = 108
ytable = 60

xnet = 66
ynet = 6  # Kinda like a z coord after 90 degree rotation


# Init robot arm in frame
def init_arm(w):
    # Init Cube Faces for all
    renderFaces = np.array([[1, 0, 7], [1, 3, 7],
                            [1, 2, 4], [1, 0, 4],
                            [1, 2, 6], [1, 3, 6],
                            [0, 4, 5], [0, 7, 5],
                            [2, 4, 5], [2, 6, 5],
                            [3, 6, 5], [3, 7, 5]])

    # --- Create Arm Base ---
    basePlateLength = 12  # in inches
    baseDistFromTable = -10.5
    basePlateThickness = 0.5
    baseColors = np.array([[0.5, 0.3, 0.2, 1] for i in range(12)])

    # all verts for shapes
    baseStandVerts = np.array([[basePlateLength, 0, 0],  # 0
                               [0, 0, 0],  # 1
                               [0, basePlateLength, 0],  # 2
                               [0, 0, -basePlateThickness],  # 3
                               [basePlateLength, basePlateLength, 0],  # 4
                               [basePlateLength, basePlateLength, -basePlateThickness],  # 5
                               [0, basePlateLength, -basePlateThickness],  # 6
                               [basePlateLength, 0, -basePlateThickness]])  # 7

    # Create same stands
    basePlate = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                              drawEdges=True, edgeColor=(0, 0, 0, 1))

    # Move stands to correct coord
    basePlate.translate(baseDistFromTable - basePlateLength,
                        (ytable / 2) - (basePlateLength / 2),
                        0)

    w.addItem(basePlate)

    # --- Create Base Stands ---
    heightBaseStand = 12.6
    widthBaseStand = 3.5
    baseStandVerts = np.array([[widthBaseStand, 0, 0],  # 0
                               [0, 0, 0],  # 1
                               [0, widthBaseStand, 0],  # 2
                               [0, 0, heightBaseStand],  # 3
                               [widthBaseStand, widthBaseStand, 0],  # 4
                               [widthBaseStand, widthBaseStand, heightBaseStand],  # 5
                               [0, widthBaseStand, heightBaseStand],  # 6
                               [widthBaseStand, 0, heightBaseStand]])  # 7

    # Create same stands
    baseStand1 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                               drawEdges=True, edgeColor=(0, 0, 0, 1))
    baseStand2 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                               drawEdges=True, edgeColor=(0, 0, 0, 1))

    # Translate stands
    baseStand1.translate(baseDistFromTable - (widthBaseStand / 2) - (basePlateLength / 2),
                         (ytable / 2) + (basePlateLength / 2) - widthBaseStand,
                         0)
    baseStand2.translate(baseDistFromTable - (widthBaseStand / 2) - (basePlateLength / 2),
                         (ytable / 2) - (basePlateLength / 2), 0)

    # Add to frame
    w.addItem(baseStand1)
    w.addItem(baseStand2)

    # --- Create Arm One ---
    arm1Length = 22  # in inches
    arm1Width = 1
    arm1Height = 11.57
    arm1Colors = np.array([[1, 0, 1, 1] for i in range(12)])

    arm1Verts = np.array([[arm1Width, 0, 0],  # 0
                          [0, 0, 0],  # 1
                          [0, arm1Width, 0],  # 2
                          [0, 0, arm1Length],  # 3
                          [arm1Width, arm1Width, 0],  # 4
                          [arm1Width, arm1Width, arm1Length],  # 5
                          [0, arm1Width, arm1Length],  # 6
                          [arm1Width, 0, arm1Length]])  # 7

    # Create same stands
    arm1 = gl.GLMeshItem(vertexes=arm1Verts, faces=renderFaces, faceColors=arm1Colors,
                         drawEdges=True, edgeColor=(0, 0, 0, 1))

    # Move stands to correct coord
    arm1.translate(baseDistFromTable - (basePlateLength / 2) - (arm1Width / 2),
                   (ytable / 2) - (arm1Width / 2),
                   arm1Height)

    w.addItem(arm1)

    # --- Create Arm Two ---
    arm2Length = 22  # in inches
    arm2Width = 1
    arm2Colors = np.array([[1, 1, 0, 1] for i in range(12)])

    arm2Verts = np.array([[arm2Width, 0, 0],  # 0
                          [0, 0, 0],  # 1
                          [0, arm2Width, 0],  # 2
                          [0, 0, arm2Length],  # 3
                          [arm2Width, arm2Width, 0],  # 4
                          [arm2Width, arm2Width, arm2Length],  # 5
                          [0, arm2Width, arm2Length],  # 6
                          [arm2Width, 0, arm2Length]])  # 7

    # Create same stands
    arm2 = gl.GLMeshItem(vertexes=arm2Verts, faces=renderFaces, faceColors=arm2Colors,
                         drawEdges=True, edgeColor=(0, 0, 0, 1))

    # Move stands to correct coord
    arm2.translate(baseDistFromTable - (basePlateLength / 2) - (arm1Width / 2),
                   (ytable / 2) - (arm1Width / 2),
                   32.5)

    w.addItem(arm2)

    # --- Create End Joint ---
    arm3Length = 3.25  # meant to be rotated and flapped
    arm3Width = 0.5
    arm3Colors = np.array([[1, 0, 0.6, 1] for i in range(12)])

    arm3Verts = np.array([[arm3Width, 0, 0],  # 0
                          [0, 0, 0],  # 1
                          [0, arm3Width, 0],  # 2
                          [0, 0, arm3Length],  # 3
                          [arm3Width, arm3Width, 0],  # 4
                          [arm3Width, arm3Width, arm3Length],  # 5
                          [0, arm3Width, arm3Length],  # 6
                          [arm3Width, 0, arm3Length]])  # 7

    # Create same stands
    arm3 = gl.GLMeshItem(vertexes=arm3Verts, faces=renderFaces, faceColors=arm3Colors,
                         drawEdges=True, edgeColor=(0, 0, 0, 1))

    # Move stands to correct coord
    arm3.translate(baseDistFromTable - (basePlateLength / 2) - (arm3Width / 2),
                   (ytable / 2) - (arm3Width / 2),
                   54)

    w.addItem(arm3)

    return baseStand1


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

    # ---- Create Objects to be Moved / Plotted ----

    # Create arm
    arm = init_arm(w)


    originTranslation = [[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1],
                         [0, 0, 0]]
    arm.applyTransform(originTranslation, local=False)
    arm.rotate(45, 0, 0, 1, )


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
