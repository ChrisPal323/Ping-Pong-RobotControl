import math
import sys
import numpy as np
import pyqtgraph.opengl as gl


def unitVector(vector):
    """
        Returns the unit vector of a given input vector.
        Params:
            vector -> input vector.
        Returns:
            numpy.array().
    """
    # Divide the input vector by its magnitude.
    return vector / np.linalg.norm(vector)


class Segment:
    """
        A part of the FabrikSolver to store a part of an inverse kinematics chain.
    """

    def __init__(self, referenceX, referenceY, referenceZ, length, zAngle, yAngle):
        """
            Params:
                referenceX -> x component of the reference point.
                referenceY -> y component of the reference point.
                referenceZ -> Z component of the reference point.
                length -> length of the segment.
                zAngle -> initial angle along the z-axis of the segment.
                yAngle -> initial angle along the y-axis of the segment.
        """

        self.zAngle = zAngle
        self.yAngle = yAngle

        # Current angle
        self.AngleOnZ = None

        # Store the length of the segment.
        self.length = length

        # Calculate new coordinates.
        deltaX = math.cos(math.radians(zAngle)) * length
        deltaY = math.sin(math.radians(zAngle)) * length
        deltaZ = math.sin(math.radians(yAngle)) * length

        # Calculate new coordinates with respect to reference.
        newX = referenceX + deltaX
        newY = referenceY + deltaY
        newZ = referenceZ + deltaZ

        # Store new coordinates.
        self.point = np.array([newX, newY, newZ])


class FabrikSolver:
    """
        An inverse kinematics solver. Uses the Fabrik Algorithm.
    """

    def __init__(self, arm, w, marginOfError=0.01):
        """
            Params:
                baseX -> x component of the base.
                baseY -> y coordinate of the base.
                baseZ -> z coordinate of the base.
                marginOfError -> the margin of error for the algorithm.
        """

        # For plotting target Point
        self.targetIKCords = [0, 0, 0]
        self.targetIKPoint = None
        self.targetIKPlotted = False

        self.targetReboundPoint = None
        self.targetReboundCords = [0, 0, 0]
        self.targetReboundPlotted = False

        # Set window
        self.w = w

        # Create the base of the chain.
        self.basePoint = np.array(arm.armBasePoint)

        # Initialize empty segment array -> [].
        self.segments = []

        # Initialize length of the chain -> 0.
        self.armLength = 0

        # Initialize the margin of error
        self.marginOfError = marginOfError

        # Init Arm!
        self.arm = arm
        # Initial / Home Position
        self.addSegment(arm.arm1Length, 0, 45)
        self.addSegment(arm.arm2Length, 0, 0)

    def addSegment(self, length, zAngle, yAngle):

        """
            Add new segment to chain with respect to the last segment.
            Params:
                length -> length of the segment.
                zAngle -> initial angle of the segment along the z axis.
                yAngle -> initial angle of the segment along the z axis.
        """

        if len(self.segments) > 0:

            segment = Segment(self.segments[-1].point[0], self.segments[-1].point[1], self.segments[-1].point[2],
                              length, zAngle + self.segments[-1].zAngle, self.segments[-1].yAngle + yAngle)
        else:
            # Create a segment of the vector start point, length and angle.
            segment = Segment(self.basePoint[0], self.basePoint[1], self.basePoint[2], length, zAngle, yAngle)

        # Add length to the total arm length.
        self.armLength += segment.length

        # Add the new segment to the list.
        self.segments.append(segment)

    def isReachable(self, targetX, targetY, targetZ):
        """
            Check if a point in space is reachable by the end-effector.
            Params:
                targetX -> the target x coordinate to check.
                targetY -> the target y coordinate to check.
                targetZ -> the target z coordinate to check.
            Returns:
                Boolean.
        """

        if np.linalg.norm(self.basePoint - np.array([targetX, targetY, targetZ])) < self.armLength:
            return True
        return False

    def inMarginOfError(self, targetX, targetY, targetZ):
        """
            Check if the distance of a point in space and the end-effector is smaller than the margin of error.
            Params:
                targetX -> the target x coördinate to check.
                targetY -> the target y coördinate to check.
                targetZ -> the target z coördinate to check.
            Returns:
                Boolean.
        """
        if np.linalg.norm(self.segments[-1].point - np.array([targetX, targetY, targetZ])) < self.marginOfError:
            return True
        return False

    def iterate(self, targetX, targetY, targetZ):
        """
            Do one iteration of the fabrik algorithm. Used in the compute function.
            Use in simulations or other systems who require motion that converges over time.
            Params:
                targetX -> the target x coordinate to move to.
                targetY -> the target y coordinate to move to.
                targetZ -> the target y coordinate to move to.
        """

        target = np.array([targetX, targetY, targetZ])

        # Backwards.
        for i in range(len(self.segments) - 1, 0, -1):

            # On the end, we need to use the end point first to be able to apply the formula.
            # Check if the value of i is equal to the index of the last vector on the arm.
            if i == len(self.segments) - 1:
                # Go one index lower to the next to last vector in the list.
                # Then use the formula with the final vector and multiply the length of the vector by the final index.

                # Replace old vector with new vector.
                self.segments[i - 1].point = (unitVector(self.segments[i - 1].point - target) * self.segments[
                    i].length) + target

            else:
                self.segments[i - 1].point = (unitVector(self.segments[i - 1].point - self.segments[i].point) *
                                              self.segments[i].length) + self.segments[i].point

        # Forwards.
        for i in range(len(self.segments)):
            if i == 0:
                self.segments[i].point = (unitVector(self.segments[i].point - self.basePoint) * self.segments[
                    i].length) + self.basePoint

            elif i == len(self.segments) - 1:
                self.segments[i].point = (unitVector(self.segments[i - 1].point - target) * self.segments[
                    i].length * -1) + self.segments[i - 1].point

            else:
                self.segments[i].point = (unitVector(self.segments[i].point - self.segments[i - 1].point) *
                                          self.segments[i].length) + self.segments[i - 1].point

    # calc the angle of the line with the point before the current segment number and the x or z axis
    def calcJointAngles(self):

        # Top down angle of base, ray from base point to targeted point (x, y) plane
        theta0 = math.atan(
            (self.targetIKCords[1] - self.basePoint[1]) / (self.targetIKCords[0] - self.basePoint[0])) * 180 / math.pi

        # Vertical angles of arms (x, z) plane
        theta1 = math.atan((self.segments[0].point[2] - self.basePoint[2]) / (
                self.segments[0].point[0] - self.basePoint[0])) * 180 / math.pi
        theta2 = math.atan((self.segments[1].point[2] - self.segments[0].point[2]) / (
                self.segments[1].point[0] - self.segments[0].point[0])) * 180 / math.pi
        # Find angle from z axis, not x
        theta1 = 90 - theta1
        theta2 = 90 - theta2

        return theta0, theta1, theta2

    def computeAndUpdate(self, targetX, targetY, targetZ):

        """
            Iterate the fabrik algorithm until the distance from the end-effector to the target is within the margin of error.
            Params:
                target is POS OF BALL
                We want an offset to keep the
                targetX -> the target x coordinate to move to.
                targetY -> the target x coordinate to move to.
                targetZ -> the target z coordinate to move to.
        """
        # Add to object
        self.targetIKCords = [targetX, targetY, targetZ]

        # Check if target is directly in line with base, now only 2D IK, probably should fix later
        if math.atan((self.targetIKCords[1] - self.basePoint[1]) / (
                self.targetIKCords[0] - self.basePoint[0])) * 180 / math.pi == 0:

            # Target is directly in front of arm, do normal IK
            if self.isReachable(targetX, targetY, targetZ):
                while not self.inMarginOfError(targetX, targetY, targetZ):
                    self.iterate(targetX, targetY, targetZ)

                # Done with loop, move joint to IK Point
                base, arm1, arm2 = self.calcJointAngles()
                self.arm.rotateAllJoints(base, arm1, arm2)

            else:
                print("Closest I can get, or not reachable")
        else:
            # Do circle math to calc new distance, Object is NOT in front of arm

            # Get top down distance from target to base point, only XY plane
            targetDist = math.sqrt(
                ((self.basePoint[0] - self.targetIKCords[0]) ** 2) + ((self.basePoint[1] - self.targetIKCords[1]) ** 2))

            # Adjust new target position with new distance
            tempTargetX = targetDist + self.basePoint[0]
            # Line up with base point on Y axis
            tempTargetY = self.basePoint[1]

            # IK with new point, we didn't touch Z
            if self.isReachable(tempTargetX, tempTargetY, targetZ):
                while not self.inMarginOfError(tempTargetX, tempTargetY, targetZ):
                    self.iterate(tempTargetX, tempTargetY, targetZ)

                # Done with loop
                base, arm1, arm2 = self.calcJointAngles()
                # Rotate Base! Should line up now
                self.arm.rotateAllJoints(base, arm1, arm2)

            else:
                print("Closest I can get, or not reachable")
            pass

    def updatePaddleRebound(self, point):
        # Point1 is CurrentIKPoint
        # Point2 is Target Rebound Point

        self.targetReboundCords = point

        # Find XY plane slope and angle between them
        thetaXY = -math.atan((self.targetReboundCords[1] - self.targetIKCords[1]) /
                             (self.targetReboundCords[0] - self.targetIKCords[0])) * 180 / math.pi

        # Compensate for base rotation
        thetaXY += self.arm.currentJointAngles[0]

        self.arm.rotateArm3(90, thetaXY)

    # ---- Plot IK Target Point ----
    def plotTarget(self):

        if self.targetIKPlotted:
            # Move to new target position
            self.targetIKPoint.resetTransform()
            self.targetIKPoint.translate(self.targetIKCords[0], self.targetIKCords[1], self.targetIKCords[2])
        else:
            # Plot Target Point
            md = gl.MeshData.sphere(rows=10, cols=20, radius=0.5)
            self.targetIKPoint = gl.GLMeshItem(
                meshdata=md,
                smooth=True,
                color=(255, 0, 255, 0.3),
                shader="balloon",
                glOptions="additive",
            )
            self.w.addItem(self.targetIKPoint)
            self.targetIKPlotted = True

    # ---- Remove IK Target Point ----
    def removePlottedTarget(self):
        self.w.removeItem(self.targetIKPoint)
        self.targetIKPlotted = False

    # ---- Plot IK Paddle Goal Point ----
    def plotReboundGoal(self):
        if self.targetReboundPlotted:
            self.targetReboundPoint.resetTransform()
            self.targetReboundPoint.translate(self.targetReboundCords[0], self.targetReboundCords[1],
                                              self.targetReboundCords[2])
        else:
            # Plot Target Point
            md = gl.MeshData.sphere(rows=10, cols=20, radius=2)
            self.targetReboundPoint = gl.GLMeshItem(
                meshdata=md,
                smooth=True,
                color=(75, 255, 255, 0.3),
                shader="balloon",
                glOptions="additive",
            )
            self.w.addItem(self.targetReboundPoint)
            self.targetReboundPlotted = True

    # ---- Plot IK Paddle Goal Point ----
    def removeReboundGoal(self):
        self.w.removeItem(self.targetReboundPoint)
        self.targetReboundPlotted = False
