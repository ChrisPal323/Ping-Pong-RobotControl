import math
import random
import sys

import numpy
import numpy as np
import cv2
from collections import deque
import pyqtgraph.opengl as gl
from pyqtgraph import Transform3D
from pyqtgraph.Qt import QtGui


class RobotArm:
    """
    Will be used to
    - create rendering of arm
    - keep track of each joint transformations
    - Possibly inverse kinematics (maybe another class)
    """

    # Numbers that work well (in inches) with the graph and keep proper proportions
    xtable = 108
    ytable = 60

    xnet = 66
    ynet = 6  # Kinda like a z coord after 90 degree rotation

    # TODO: Fix this array!
    # Will store all the transformations and rotations done on each object
    # this is for us to reference and revert and transformations done if needed
    transformMatrixList = np.array([deque(maxlen=4),  # Base Plate
                                    deque(maxlen=4),  # Arm 1
                                    deque(maxlen=4),  # Arm 2
                                    deque(maxlen=4),  # Arm 3
                                    None])  # Paddle

    def __init__(self, w):
        # Working Cube Faces for all parts
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
        baseColors = np.array([[0.4, 0.4, 0.4, 1] for i in range(12)])

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
        self.basePlate = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                       drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Center at origin
        self.basePlate.translate(-(basePlateLength / 2), -(basePlateLength / 2), 0)

        # Align with robot
        baseTransform = Transform3D()
        baseTransform.translate(baseDistFromTable - (basePlateLength / 2),
                                (self.ytable / 2),
                                0)
        # Transform to align with robot when other sides are done
        w.addItem(self.basePlate)

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
        self.baseStand1 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                        drawEdges=True, edgeColor=(0, 0, 0, 1))
        self.baseStand2 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                        drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Translate stands to ORIGIN
        self.baseStand1.translate(-widthBaseStand / 2,
                                  (basePlateLength / 2) - widthBaseStand,
                                  0)

        self.baseStand2.translate(-widthBaseStand / 2,
                                  -(basePlateLength / 2),
                                  0)

        # Custom method to transform and store transformations
        self.transformBase(baseTransform)

        # Add to frame
        w.addItem(self.baseStand1)
        w.addItem(self.baseStand2)

        # --- Create Arm One ---
        arm1Length = 22  # in inches
        arm1Width = 1
        arm1Height = 11.57
        arm1Colors = np.array([[0.5, 0.5, 0.5, 1] for i in range(12)])

        arm1Verts = np.array([[arm1Width, 0, 0],  # 0
                              [0, 0, 0],  # 1
                              [0, arm1Width, 0],  # 2
                              [0, 0, arm1Length],  # 3
                              [arm1Width, arm1Width, 0],  # 4
                              [arm1Width, arm1Width, arm1Length],  # 5
                              [0, arm1Width, arm1Length],  # 6
                              [arm1Width, 0, arm1Length]])  # 7

        # Create same stands
        self.arm1 = gl.GLMeshItem(vertexes=arm1Verts, faces=renderFaces, faceColors=arm1Colors,
                                  drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Move stands to ORIGIN
        self.arm1.translate(-(arm1Width / 2),
                            -(arm1Width / 2),
                            0)

        # Align with robot
        arm1Transform = Transform3D()
        arm1Transform.translate(baseDistFromTable - (basePlateLength / 2),
                                (self.ytable / 2),
                                arm1Height)
        self.transformArm1(arm1Transform)
        w.addItem(self.arm1)

        # --- Create Arm Two ---
        arm2Length = 22  # in inches
        arm2Width = 1
        arm2Colors = np.array([[0.6, 0.6, 0.6, 1] for i in range(12)])

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
                       (self.ytable / 2) - (arm1Width / 2),
                       32.5)

        w.addItem(arm2)

        # --- Create End Joint ---
        arm3Length = 3.25  # meant to be rotated and flapped
        arm3Width = 0.5
        arm3Colors = np.array([[0.7, 0.7, 0.7, 1] for i in range(12)])

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
                       (self.ytable / 2) - (arm3Width / 2),
                       54)

        w.addItem(arm3)

        # --- Create Paddle ---
        paddleLength = 6.7  # meant to be rotated and flapped
        paddleWidth = 5.9
        paddleThickness = 0.25
        paddleColors = np.array([[0.5, 0, 0, 1] for i in range(12)])

        paddleVerts = np.array([[paddleLength, 0, 0],  # 0
                                [0, 0, 0],  # 1
                                [0, paddleWidth, 0],  # 2
                                [0, 0, paddleThickness],  # 3
                                [paddleLength, paddleWidth, 0],  # 4
                                [paddleLength, paddleWidth, paddleThickness],  # 5
                                [0, paddleWidth, paddleThickness],  # 6
                                [paddleLength, 0, paddleThickness]])  # 7

        # Create same stands
        paddle = gl.GLMeshItem(vertexes=paddleVerts, faces=renderFaces, faceColors=paddleColors,
                               drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Move stands to correct coord
        paddle.translate(baseDistFromTable - (basePlateLength / 2) - (arm3Width / 2),
                         (self.ytable / 2) - (arm3Width / 2),
                         54 + 4)

        w.addItem(paddle)

    # ------ Base Transforms --------

    # In goes a transformation mat
    # Effects all the other parts of the arm as well
    def transformBase(self, tr):
        # Apply and save transform
        self.basePlate.applyTransform(tr, False)
        self.baseStand1.applyTransform(tr, False)
        self.baseStand2.applyTransform(tr, False)
        self.transformMatrixList[0].append(tr)

    def rotateBase(self, deg):
        # Center to origin
        invMat, ret = self.transformMatrixList[0][-1].inverted()
        self.basePlate.applyTransform(invMat, False)
        self.baseStand1.applyTransform(invMat, False)
        self.baseStand2.applyTransform(invMat, False)

        # Rotate
        self.basePlate.rotate(deg, 0, 0, 1)
        self.baseStand1.rotate(deg, 0, 0, 1)
        self.baseStand2.rotate(deg, 0, 0, 1)

        # Bring back to original cords
        self.basePlate.applyTransform(self.transformMatrixList[0][-1], False)
        self.baseStand1.applyTransform(self.transformMatrixList[0][-1], False)
        self.baseStand2.applyTransform(self.transformMatrixList[0][-1], False)

        # TODO: Have other arms react
        self.rotateArm1(deg, zAxis=True)

    # ------ Arm 1 Transforms --------

    # In goes a transformation mat
    def transformArm1(self, tr):
        # Apply and save transform
        self.arm1.applyTransform(tr, False)
        self.transformMatrixList[1].append(tr)

    def rotateArm1(self, deg, zAxis=False):
        # Center to origin
        invMat, ret = self.transformMatrixList[1][-1].inverted()
        self.arm1.applyTransform(invMat, False)

        if zAxis:
            # Rotate
            self.arm1.rotate(deg, 0, 0, 1)
        else:
            # Rotate TODO: FIX
            self.arm1.rotate(deg, 0, 1, 0)

        # Bring back to original cords
        self.arm1.applyTransform(self.transformMatrixList[1][-1], False)

        # TODO: Have other arms react

    # ------ Arm 2 Transforms--------

    # In goes a transformation mat
    def transformArm2(self, tr):
        # Apply and save transform
        self.transformArm2.applyTransform(tr, False)
        self.transformMatrixList[2].append(tr)

    def rotateArm3(self, deg, zAxis=False):
        # Center to origin
        invMat, ret = self.transformMatrixList[0][-1].inverted()
        self.basePlate.applyTransform(invMat, False)

        if not zAxis:
            # Rotate
            self.arm1.rotate(deg, 0, 1, 0)
        else:
            # Rotate
            self.arm1.rotate(deg, 1, 0, 0)

        # Bring back to original cords
        self.basePlate.applyTransform(self.transformMatrixList[0][-1], False)

        # TODO: Have other arms react

    # ------ Arm 3 Transforms--------

    # In goes a transformation mat
    def transformArm3(self, tr):
        # Apply and save transform
        self.transformArm3.applyTransform(tr, False)
        self.transformMatrixList[3].append(tr)

    def rotateArm3X(self, deg):
        # Center to origin
        invMat, ret = self.transformMatrixList[0][-1].inverted()
        self.basePlate.applyTransform(invMat, False)

        # Rotate
        self.basePlate.rotate(deg, 0, 0, 1)

        # Bring back to original cords
        self.basePlate.applyTransform(self.transformMatrixList[0][-1], False)

        # TODO: Have other arms react

    def rotateArm3Y(self, deg):
        # Center to origin
        invMat, ret = self.transformMatrixList[0][-1].inverted()
        self.basePlate.applyTransform(invMat, False)

        # Rotate
        self.basePlate.rotate(deg, 0, 0, 1)

        # Bring back to original cords
        self.basePlate.applyTransform(self.transformMatrixList[0][-1], False)

        # TODO: Have other arms react
