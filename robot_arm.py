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

    # Will store all the initial transformations to get to the arm position
    # this is for us to reference and revert and transformations done if needed
    initTransformMatrixList = np.array([deque(maxlen=4),  # Base Plate
                                        deque(maxlen=4),  # Arm 1
                                        deque(maxlen=4),  # Arm 2
                                        deque(maxlen=4),  # Arm 3
                                        deque(maxlen=4),  # Paddle
                                        None])

    # All joints and they angle
    currentJointAngles = [0,  # Base Plate (just a number)
                          0,  # Arm 1  (x, y, z)
                          0,  # Arm 2  (x, y, z)
                          [0, 0],  # Arm 3  (x, y, z)
                          0]  # Paddle (x, y, z)

    def __init__(self, w):
        # Working Cube Faces for all parts
        renderFaces = np.array([[1, 0, 7], [1, 3, 7],
                                [1, 2, 4], [1, 0, 4],
                                [1, 2, 6], [1, 3, 6],
                                [0, 4, 5], [0, 7, 5],
                                [2, 4, 5], [2, 6, 5],
                                [3, 6, 5], [3, 7, 5]])

        # --- Create Arm Base ---
        self.basePlateLength = 12  # in inches
        self.baseDistFromTable = -10.5
        self.basePlateThickness = 0.5
        baseColors = np.array([[0.4, 0.4, 0.4, 1] for i in range(12)])

        # all verts for shapes
        baseStandVerts = np.array([[self.basePlateLength, 0, 0],  # 0
                                   [0, 0, 0],  # 1
                                   [0, self.basePlateLength, 0],  # 2
                                   [0, 0, -self.basePlateThickness],  # 3
                                   [self.basePlateLength, self.basePlateLength, 0],  # 4
                                   [self.basePlateLength, self.basePlateLength, -self.basePlateThickness],  # 5
                                   [0, self.basePlateLength, -self.basePlateThickness],  # 6
                                   [self.basePlateLength, 0, -self.basePlateThickness]])  # 7

        # Create same stands
        self.basePlate = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                       drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Center at origin
        self.basePlate.translate(-(self.basePlateLength / 2), -(self.basePlateLength / 2), 0)

        # Align with robot
        baseTransform = Transform3D()
        baseTransform.translate(self.baseDistFromTable - (self.basePlateLength / 2),
                                (self.ytable / 2),
                                0)

        # Add to Frame
        w.addItem(self.basePlate)

        # --- Create Base Stands ---
        self.heightBaseStand = 12.6
        self.widthBaseStand = 3.5
        baseStandVerts = np.array([[self.widthBaseStand, 0, 0],  # 0
                                   [0, 0, 0],  # 1
                                   [0, self.widthBaseStand, 0],  # 2
                                   [0, 0, self.heightBaseStand],  # 3
                                   [self.widthBaseStand, self.widthBaseStand, 0],  # 4
                                   [self.widthBaseStand, self.widthBaseStand, self.heightBaseStand],  # 5
                                   [0, self.widthBaseStand, self.heightBaseStand],  # 6
                                   [self.widthBaseStand, 0, self.heightBaseStand]])  # 7

        # Create same stands
        self.baseStand1 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                        drawEdges=True, edgeColor=(0, 0, 0, 1))
        self.baseStand2 = gl.GLMeshItem(vertexes=baseStandVerts, faces=renderFaces, faceColors=baseColors,
                                        drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Translate stands to ORIGIN
        self.baseStand1.translate(-self.widthBaseStand / 2,
                                  (self.basePlateLength / 2) - self.widthBaseStand,
                                  0)

        self.baseStand2.translate(-self.widthBaseStand / 2,
                                  -(self.basePlateLength / 2),
                                  0)

        # Custom method to transform and store transformations
        self.transformBase(baseTransform)

        # Add to Frame
        w.addItem(self.baseStand1)
        w.addItem(self.baseStand2)

        # --- Create Arm One ---
        self.arm1Length = 21  # in inches
        self.arm1Width = 1
        self.arm1Height = 11.57
        arm1Colors = np.array([[0.5, 0.5, 0.5, 1] for i in range(12)])

        arm1Verts = np.array([[self.arm1Width, 0, 0],  # 0
                              [0, 0, 0],  # 1
                              [0, self.arm1Width, 0],  # 2
                              [0, 0, self.arm1Length],  # 3
                              [self.arm1Width, self.arm1Width, 0],  # 4
                              [self.arm1Width, self.arm1Width, self.arm1Length],  # 5
                              [0, self.arm1Width, self.arm1Length],  # 6
                              [self.arm1Width, 0, self.arm1Length]])  # 7

        # Create same stands
        self.arm1 = gl.GLMeshItem(vertexes=arm1Verts, faces=renderFaces, faceColors=arm1Colors,
                                  drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Move stands to ORIGIN
        self.arm1.translate(-(self.arm1Width / 2),
                            -(self.arm1Width / 2),
                            0)

        # Align with robot
        arm1Transform = Transform3D()
        arm1Transform.translate(self.baseDistFromTable - (self.basePlateLength / 2),
                                (self.ytable / 2),
                                self.arm1Height)
        self.transformArm1(arm1Transform)

        # Add to Frame
        w.addItem(self.arm1)

        # --- Create Arm Two ---
        self.arm2Length = 21  # in inches
        self.arm2Width = 1
        self.arm2Height = 32.5
        arm2Colors = np.array([[0.6, 0.6, 0.6, 1] for i in range(12)])

        arm2Verts = np.array([[self.arm2Width, 0, 0],  # 0
                              [0, 0, 0],  # 1
                              [0, self.arm2Width, 0],  # 2
                              [0, 0, self.arm2Length],  # 3
                              [self.arm2Width, self.arm2Width, 0],  # 4
                              [self.arm2Width, self.arm2Width, self.arm2Length],  # 5
                              [0, self.arm2Width, self.arm2Length],  # 6
                              [self.arm2Width, 0, self.arm2Length]])  # 7

        # Create arm 2
        self.arm2 = gl.GLMeshItem(vertexes=arm2Verts, faces=renderFaces, faceColors=arm2Colors,
                                  drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Move arm to ORIGIN
        self.arm2.translate(-(self.arm2Width / 2),
                            -(self.arm2Width / 2),
                            0)

        # Align with robot
        arm2Transform = Transform3D()
        arm2Transform.translate(self.baseDistFromTable - (self.basePlateLength / 2),
                                (self.ytable / 2),
                                self.arm2Height)
        self.transformArm2(arm2Transform)

        # Add to Frame
        w.addItem(self.arm2)

        # --- Create End Joint ---
        self.arm3Length = 3.25  # meant to be rotated and flapped
        self.arm3Width = 1
        self.arm3Height = 53.5
        arm3Colors = np.array([[0.7, 0.7, 0.7, 1] for i in range(12)])

        arm3Verts = np.array([[self.arm3Width, 0, 0],  # 0
                              [0, 0, 0],  # 1
                              [0, self.arm3Width, 0],  # 2
                              [0, 0, self.arm3Length],  # 3
                              [self.arm3Width, self.arm3Width, 0],  # 4
                              [self.arm3Width, self.arm3Width, self.arm3Length],  # 5
                              [0, self.arm3Width, self.arm3Length],  # 6
                              [self.arm3Width, 0, self.arm3Length]])  # 7

        # Create same stands
        self.arm3 = gl.GLMeshItem(vertexes=arm3Verts, faces=renderFaces, faceColors=arm3Colors,
                                  drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Move arm to ORIGIN
        self.arm3.translate(-(self.arm3Width / 2),
                            -(self.arm3Width / 2),
                            0)

        # Align with robot
        arm3Transform = Transform3D()
        arm3Transform.translate(self.baseDistFromTable - (self.basePlateLength / 2),
                                (self.ytable / 2),
                                self.arm3Height)
        self.transformArm3(arm3Transform)

        # Add to Frame
        w.addItem(self.arm3)

        # --- Create Paddle ---
        self.paddleLength = 8.75  # meant to be rotated and flapped
        self.paddleWidth = 6
        self.paddleHeight = 56.75
        self.paddleThickness = 0.25
        self.paddleColors = np.array([[0.5, 0, 0, 1] for i in range(12)])

        paddleVerts = np.array([[self.paddleLength, 0, 0],  # 0
                                [0, 0, 0],  # 1
                                [0, self.paddleWidth, 0],  # 2
                                [0, 0, self.paddleThickness],  # 3
                                [self.paddleLength, self.paddleWidth, 0],  # 4
                                [self.paddleLength, self.paddleWidth, self.paddleThickness],  # 5
                                [0, self.paddleWidth, self.paddleThickness],  # 6
                                [self.paddleLength, 0, self.paddleThickness]])  # 7

        # Create same stands
        self.paddle = gl.GLMeshItem(vertexes=paddleVerts, faces=renderFaces, faceColors=self.paddleColors,
                                    drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Move arm to ORIGIN
        self.paddle.translate(0,
                              -(self.paddleWidth / 2),
                              0)

        # Align with robot
        paddleTransform = Transform3D()
        paddleTransform.translate(self.baseDistFromTable - (self.basePlateLength / 2),
                                  (self.ytable / 2),
                                  self.paddleHeight)
        self.transformPaddle(paddleTransform)

        # Add to Frame
        w.addItem(self.paddle)

    # --------- Global Methods ----------
    def rotateAllJoints(self, base, arm1, arm2, arm3X, arm3Y):
        self.rotateBase(base)
        self.rotateArm1(arm1)
        self.rotateArm2(arm2)
        self.rotateArm3(arm3X, arm3Y)

    # ------ Base Transforms --------

    # In goes a transformation mat
    # Effects all the other parts of the arm as well
    def transformBase(self, tr):
        # Apply and save transform
        self.basePlate.applyTransform(tr, False)
        self.baseStand1.applyTransform(tr, False)
        self.baseStand2.applyTransform(tr, False)
        self.initTransformMatrixList[0].append(tr)

    def rotateBase(self, deg):
        self.basePlate.resetTransform()
        self.baseStand1.resetTransform()
        self.baseStand2.resetTransform()

        # Center at origin
        self.basePlate.translate(-(self.basePlateLength / 2),
                                 -(self.basePlateLength / 2),
                                 0)
        self.baseStand1.translate(-self.widthBaseStand / 2,
                                  (self.basePlateLength / 2) - self.widthBaseStand,
                                  0)
        self.baseStand2.translate(-self.widthBaseStand / 2,
                                  -(self.basePlateLength / 2),
                                  0)

        # Record angle
        self.currentJointAngles[0] = deg

        # Rotate
        self.basePlate.rotate(deg, 0, 0, 1)
        self.baseStand1.rotate(deg, 0, 0, 1)
        self.baseStand2.rotate(deg, 0, 0, 1)

        # Bring back to original cords
        self.basePlate.applyTransform(self.initTransformMatrixList[0][-1], False)
        self.baseStand1.applyTransform(self.initTransformMatrixList[0][-1], False)
        self.baseStand2.applyTransform(self.initTransformMatrixList[0][-1], False)

        # Just to update arm, just uses change in base angle
        self.updateArm1()
        self.updateArm2()
        self.updateArm3()
        self.updatePaddle()

    # ------ Arm 1 Transforms --------

    # In goes a transformation mat
    def transformArm1(self, tr):
        # Apply and save transform
        self.arm1.applyTransform(tr, False)
        self.initTransformMatrixList[1].append(tr)

    def rotateArm1(self, armDeg):
        self.arm1.resetTransform()
        # Move arm to ORIGIN
        self.arm1.translate(-(self.arm1Width / 2),
                            -(self.arm1Width / 2),
                            0)

        # Record angle
        self.currentJointAngles[1] = armDeg

        # rotate and tilt to base stand
        self.arm1.rotate(armDeg, 0, 1, 0)
        self.arm1.rotate(self.currentJointAngles[0], 0, 0, 1)

        # Bring back to original cords
        self.arm1.applyTransform(self.initTransformMatrixList[1][-1], False)

        # Update other arms
        self.updateArm2()
        self.updateArm3()
        self.updatePaddle()

    def updateArm1(self):
        self.rotateArm1(self.currentJointAngles[1])

    # ------ Arm 2 Transforms--------

    # In goes a transformation mat
    def transformArm2(self, tr):
        # Apply and save transform
        self.arm2.applyTransform(tr, False)
        self.initTransformMatrixList[2].append(tr)

    def rotateArm2(self, deg):
        self.arm2.resetTransform()
        # Move arm to ORIGIN
        self.arm2.translate(-(self.arm2Width / 2),
                            -(self.arm2Width / 2),
                            0)

        # Record angle
        self.currentJointAngles[2] = deg

        # Translations due to arm1 moving
        distFromCenter = self.arm1Length * math.sin(self.currentJointAngles[1] * math.pi / 180)
        changeInHeight = self.arm1Length * math.cos(self.currentJointAngles[1] * math.pi / 180) - self.arm1Length

        # rotate FIRST and tilt to base stand
        self.arm2.rotate(deg, 0, 1, 0)

        # Move added cords because of arm 1
        self.arm2.translate(distFromCenter, 0, changeInHeight)

        self.arm2.rotate(self.currentJointAngles[0], 0, 0, 1)

        # Bring back to original cords
        self.arm2.applyTransform(self.initTransformMatrixList[2][-1], False)

        # Have other arms react
        self.updateArm3()
        self.updatePaddle()

    def updateArm2(self):
        self.rotateArm2(self.currentJointAngles[2])

    # ------ Arm 3 Transforms--------

    # In goes a transformation mat
    def transformArm3(self, tr):
        # Apply and save transform
        self.arm3.applyTransform(tr, False)
        self.initTransformMatrixList[3].append(tr)

    def rotateArm3(self, degX, degY):
        self.arm3.resetTransform()
        # Move arm to ORIGIN
        self.arm3.translate(-(self.arm2Width / 2),
                            -(self.arm2Width / 2),
                            0)

        # Record angle
        self.currentJointAngles[3] = [degX, degY]

        # rotate around center for one axis of servos
        self.arm3.rotate(degY, 0, 0, 1)

        # Translations due to arm1 moving
        distFromCenter = self.arm1Length * math.sin(
            self.currentJointAngles[1] * math.pi / 180) + self.arm2Length * math.sin(
            self.currentJointAngles[2] * math.pi / 180)
        changeInHeight = (self.arm1Length * math.cos(
            self.currentJointAngles[1] * math.pi / 180) + self.arm2Length * math.cos(
            self.currentJointAngles[2] * math.pi / 180)) - (self.arm1Length + self.arm2Length)

        # rotate FIRST and tilt
        self.arm3.rotate(degX, 0, 1, 0)

        # Move added cords because of arm 1 and 2
        self.arm3.translate(distFromCenter, 0, changeInHeight)

        # Rotate becuase of base
        self.arm3.rotate(self.currentJointAngles[0], 0, 0, 1)

        # Bring back to original cords
        self.arm3.applyTransform(self.initTransformMatrixList[3][-1], False)

        # Update other limbs
        self.updatePaddle()

    def updateArm3(self):
        # Don't change angle but run other stuff in rotate arm
        self.rotateArm3(self.currentJointAngles[3][0], self.currentJointAngles[3][1])

    # ------ Paddle Transforms--------

    # In goes a transformation mat
    def transformPaddle(self, tr):
        # Apply and save transform
        self.paddle.applyTransform(tr, False)
        self.initTransformMatrixList[4].append(tr)

    def updatePaddle(self):
        # Reset and move to ORIGIN
        self.paddle.resetTransform()
        self.paddle.translate(0,
                              -(self.paddleWidth / 2),
                              0)

        # rotate around center for one axis of servos
        self.paddle.rotate(self.currentJointAngles[3][1], 0, 0, 1)

        # Translations due to arm1 moving
        distFromCenter = self.arm1Length * math.sin(self.currentJointAngles[1] * math.pi / 180) + self.arm2Length * math.sin(self.currentJointAngles[2] * math.pi / 180) + self.arm3Length * math.sin(self.currentJointAngles[3][0] * math.pi / 180)
        changeInHeight = (self.arm1Length * math.cos(self.currentJointAngles[1] * math.pi / 180) + self.arm2Length * math.cos(self.currentJointAngles[2] * math.pi / 180) + self.arm3Length * math.cos(self.currentJointAngles[3][0] * math.pi / 180)) - (self.arm1Length + self.arm2Length + self.arm3Length)

        # rotate FIRST and tilt
        self.paddle.rotate(self.currentJointAngles[3][0], 0, 1, 0)

        # Move added cords because of arm 1 and 2
        self.paddle.translate(distFromCenter, 0, changeInHeight)

        # Rotate because of base
        self.paddle.rotate(self.currentJointAngles[0], 0, 0, 1)

        # Bring back to original cords
        self.paddle.applyTransform(self.initTransformMatrixList[4][-1], False)
