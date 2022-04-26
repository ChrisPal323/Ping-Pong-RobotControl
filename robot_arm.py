import math
import numpy as np
from collections import deque
import pyqtgraph.opengl as gl
from pyqtgraph import Transform3D
import fabrik_solver


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
    initTransformMatrixList = [deque(maxlen=4),  # Base Plate
                               deque(maxlen=4),  # Arm 1
                               deque(maxlen=4),  # Arm 2
                               deque(maxlen=4),  # Arm 3
                               ]

    # ------ RENDER FUNCTIONS START ---------

    def createBase(self):
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
        self.basePlate = gl.GLMeshItem(vertexes=baseStandVerts, faces=self.renderFaces, faceColors=baseColors,
                                       drawEdges=True, edgeColor=(0, 0, 0, 1))

        # Center at origin
        self.basePlate.translate(-(self.basePlateLength / 2), -(self.basePlateLength / 2), 0)

        # Align with robot
        baseTransform = Transform3D()
        baseTransform.translate(self.baseDistFromTable - (self.basePlateLength / 2),
                                (self.ytable / 2),
                                0)

        # Add to Frame
        self.w.addItem(self.basePlate)

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
        self.baseStand1 = gl.GLMeshItem(vertexes=baseStandVerts, faces=self.renderFaces, faceColors=baseColors,
                                        drawEdges=True, edgeColor=(0, 0, 0, 1))
        self.baseStand2 = gl.GLMeshItem(vertexes=baseStandVerts, faces=self.renderFaces, faceColors=baseColors,
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
        self.w.addItem(self.baseStand1)
        self.w.addItem(self.baseStand2)

    def createArm1(self):
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
        self.arm1 = gl.GLMeshItem(vertexes=arm1Verts, faces=self.renderFaces, faceColors=arm1Colors,
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
        # Store for IK
        self.armBasePoint = [self.baseDistFromTable - (self.basePlateLength / 2),
                             (self.ytable / 2),
                             self.arm1Height]

        self.transformArm1(arm1Transform)

        # Add to Frame
        self.w.addItem(self.arm1)

    def createArm2(self):
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
        self.arm2 = gl.GLMeshItem(vertexes=arm2Verts, faces=self.renderFaces, faceColors=arm2Colors,
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
        self.w.addItem(self.arm2)

    def createArm3(self):
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
        self.arm3 = gl.GLMeshItem(vertexes=arm3Verts, faces=self.renderFaces, faceColors=arm3Colors,
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
        self.w.addItem(self.arm3)

    # ------ RENDER FUNCTIONS END ---------

    def __init__(self, w):

        # Set window
        self.w = w

        # All joints and they angle
        self.currentJointAngles = [0,  # Base Plate (just a number)
                                   0,  # Arm 1  (x, y, z)
                                   0,  # Arm 2  (x, y, z)
                                   [0, 0]]  # Arm 3  (x, y, z)

        # All Angles for Motors to Drive each joint
        self.motorJointAngles = [0,  # Base Plate (just a number)
                                 0,  # Arm 1  (x, y, z)
                                 0,  # Arm 2  (x, y, z)
                                 [0, 0]]  # Arm 3  (x, y, z)

        # Working Cube Faces for all parts
        self.renderFaces = np.array([[1, 0, 7], [1, 3, 7],
                                     [1, 2, 4], [1, 0, 4],
                                     [1, 2, 6], [1, 3, 6],
                                     [0, 4, 5], [0, 7, 5],
                                     [2, 4, 5], [2, 6, 5],
                                     [3, 6, 5], [3, 7, 5]])
        # Create Renderings
        self.createBase()
        self.createArm1()
        self.createArm2()
        self.createArm3()

        # ------ Create IK solver for arm ------
        self.solverIK = fabrik_solver.FabrikSolver(self, w)

    # --------- Global Methods ----------

    # Calculate the angles the actuators need to spin
    # From the absolute angles in the rendering software
    def calcActuatorJointAngles(self):
        self.motorJointAngles[0] = self.currentJointAngles[0] % 360
        self.motorJointAngles[1] = self.currentJointAngles[1] % 360
        self.motorJointAngles[2] = (self.currentJointAngles[2] - self.currentJointAngles[1]) % 360
        self.motorJointAngles[3][0] = (self.currentJointAngles[3][0] - self.currentJointAngles[2]) % 360
        self.motorJointAngles[3][1] = self.currentJointAngles[3][1] % 360

    def rotateAllJoints(self, base, arm1, arm2):
        self.rotateBase(base)
        self.rotateArm1(arm1)
        self.rotateArm2(arm2)

    # Returns that angles for home position of the arm
    def getHomePositionAngles(self):
        # return all home position angles
        pass

    # Basic Flick Paddle toward center of opponents side
    # Ray between middle of paddle and middle of opponents side
    # Do math to figure out how a projectile will bounce off the paddle
    # No spin lol, I ain't touchin' that yet...
    def flickPaddle(self):
        pass

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
        self.calcActuatorJointAngles()

    # Run Base Motor
    def articulateBase(self):
        pass

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
        self.calcActuatorJointAngles()

    def updateArm1(self):
        self.rotateArm1(self.currentJointAngles[1])

    # Run Arm1 Motors
    def articulateArm1(self):
        pass

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
        self.calcActuatorJointAngles()

    def updateArm2(self):
        self.rotateArm2(self.currentJointAngles[2])

    # Run Arm2 Motor
    def articulateArm2(self):
        pass

    # ------ Arm 3 Transforms--------

    # In goes a transformation mat
    def transformArm3(self, tr):
        # Apply and save transform
        self.arm3.applyTransform(tr, False)
        self.initTransformMatrixList[3].append(tr)

    def rotateArm3(self, degX=None, degY=None):

        if degX is None:
            degX = self.currentJointAngles[3][0]
        if degY is None:
            degY = self.currentJointAngles[3][1]

        self.arm3.resetTransform()
        # Move arm to ORIGIN
        self.arm3.translate(-(self.arm2Width / 2),
                            -(self.arm2Width / 2),
                            0)

        # Record angle
        self.currentJointAngles[3] = [degX, degY]

        # rotate FIRST and tilt
        self.arm3.rotate(degX, 0, 1, 0)

        # Used to reposition the axis of rotation for arm3 to work as the ones irl
        self.arm3.rotate(-self.currentJointAngles[2], 0, 1, 0)

        # rotate around center for one axis of servos
        self.arm3.rotate(degY, 0, 0, 1)

        self.arm3.rotate(self.currentJointAngles[2], 0, 1, 0)

        # Translations due to arm1 moving
        distFromCenter = self.arm1Length * math.sin(
            self.currentJointAngles[1] * math.pi / 180) + self.arm2Length * math.sin(
            self.currentJointAngles[2] * math.pi / 180)
        changeInHeight = (self.arm1Length * math.cos(
            self.currentJointAngles[1] * math.pi / 180) + self.arm2Length * math.cos(
            self.currentJointAngles[2] * math.pi / 180)) - (self.arm1Length + self.arm2Length)

        # Move added cords because of arm 1 and 2
        self.arm3.translate(distFromCenter, 0, changeInHeight)

        # Rotate because of base
        self.arm3.rotate(self.currentJointAngles[0], 0, 0, 1)

        # Bring back to original cords
        self.arm3.applyTransform(self.initTransformMatrixList[3][-1], False)

        # Update other limbs
        self.calcActuatorJointAngles()

    def updateArm3(self):
        # Don't change angle but run other stuff in rotate arm
        self.rotateArm3(self.currentJointAngles[3][0], self.currentJointAngles[3][1])

    # Run Arm3 Servo!
    def articulateArm3X(self):
        pass

    # Run Arm3 Servo!
    def articulateArm3Y(self):
        pass
