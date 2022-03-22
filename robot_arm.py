import math
import random
import sys
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

    # In order of arm base to end joint
    # Will store all the transformations and rotations done on each object
    # this is for us to reference and revert and transformations done if needed
    transformMatrixList = np.array([[]])

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
        self.basePlate = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                       drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Center at origin
        self.basePlate.translate(-(basePlateLength / 2), -(basePlateLength / 2), 0)

        # Align with robot
        basePlateTransform = Transform3D().translate(baseDistFromTable - (basePlateLength / 2),
                                                     (self.ytable / 2),
                                                     0)
        self.transformBase(basePlateTransform)

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
        baseStand1 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                   drawEdges=True, edgeColor=(0, 0, 0, 1))
        baseStand2 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                   drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Translate stands
        baseStand1.translate(baseDistFromTable - (widthBaseStand / 2) - (basePlateLength / 2),
                             (self.ytable / 2) + (basePlateLength / 2) - widthBaseStand,
                             0)
        baseStand2.translate(baseDistFromTable - (widthBaseStand / 2) - (basePlateLength / 2),
                             (self.ytable / 2) - (basePlateLength / 2), 0)

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
                       (self.ytable / 2) - (arm1Width / 2),
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
                       (self.ytable / 2) - (arm1Width / 2),
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
                       (self.ytable / 2) - (arm3Width / 2),
                       54)

        w.addItem(arm3)

        return baseStand1

    # In goes a transformation mat
    # Effects all the other parts of the arm as well
    def transformBase(self, tr):
        # Apply and save transform
        self.basePlate.applyTransform(tr, locale=True)
        self.transformMatrixList[0] = tr

    # In goes a transformation mat
    def transformArm1(self, tr):
        # Apply and save transform
        self.transformArm1.applyTransform(tr, locale=True)
        self.transformMatrixList[1] = tr

    # In goes a transformation mat
    def transformArm2(self, tr):
        # Apply and save transform
        self.transformArm2.applyTransform(tr, locale=False)
        self.transformMatrixList[2] = tr

    # In goes a transformation mat
    def transformArm3(self, tr):
        # Apply and save transform
        self.transformArm3.applyTransform(tr, locale=False)
        self.transformMatrixList[3] = tr
