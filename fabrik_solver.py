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


class Segment3D:
    """
        A part of the FabrikSolver3D to store a part of an inverse kinematics chain.
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


class FabrikSolver3D:
    """
        An inverse kinematics solver in 3D. Uses the Fabrik Algorithm.
    """

    def __init__(self, baseX=0, baseY=0, baseZ=0, marginOfError=0.01):
        """
            Params:
                baseX -> x component of the base.
                baseY -> y coordinate of the base.
                baseZ -> z coordinate of the base.

                marginOfError -> the margin of error for the algorithm.
        """

        # For plotting target Point
        self.plotted = False
        self.targetX = 0
        self.targetY = 0
        self.targetZ = 0
        self.targetPoint = None

        # Create the base of the chain.
        self.basePoint = np.array([baseX, baseY, baseZ])

        # Initialize empty segment array -> [].
        self.segments = []

        # Initialize length of the chain -> 0.
        self.armLength = 0

        # Initialize the margin of error
        self.marginOfError = marginOfError

    def addSegment(self, length, zAngle, yAngle):

        """
            Add new segment to chain with respect to the last segment.

            Params:
                length -> length of the segment.
                zAngle -> initial angle of the segment along the z axis.
                yAngle -> initial angle of the segment along the z axis.
        """

        if len(self.segments) > 0:

            segment = Segment3D(self.segments[-1].point[0], self.segments[-1].point[1], self.segments[-1].point[2],
                                length, zAngle + self.segments[-1].zAngle, self.segments[-1].yAngle + yAngle)
        else:
            # Create a segment of the vector start point, length and angle.
            segment = Segment3D(self.basePoint[0], self.basePoint[1], self.basePoint[2], length, zAngle, yAngle)

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
                targetX -> the target x coördinate to move to.
                targetY -> the target y coördinate to move to.
                targetZ -> the target y coördinate to move to.
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

    def compute(self, targetX, targetY, targetZ):

        """
            Iterate the fabrik algorithm until the distance from the end-effector to the target is within the margin of error.

            Params:
                targetX -> the target x coordinate to move to.
                targetY -> the target x coordinate to move to.
                targetZ -> the target z coordinate to move to.
        """
        # Add to object
        self.targetX = targetX
        self.targetY = targetY
        self.targetZ = targetZ

        # Add to hit on the paddle rather than the end effector
        #targetZ = targetZ + 1

        # Counter for keep track of too many iterations
        counter = 0
        if self.isReachable(targetX, targetY, targetZ):
            while not self.inMarginOfError(targetX, targetY, targetZ):
                self.iterate(targetX, targetY, targetZ)
                counter += 1
                if counter > 1000: sys.exit()
        else:
            # closest or not reachable
            pass

    # calc the angle of the line with the point before the current segment number and the y axis
    def calcJointAngles(self):

        theta1 = math.atan((self.segments[0].point[2] - self.basePoint[2]) / (self.segments[0].point[2] - self.basePoint[0])) * 180 / math.pi
        theta2 = math.atan((self.segments[1].point[2] - self.segments[0].point[2]) / (self.segments[1].point[0] - self.segments[0].point[0])) * 180 / math.pi
        theta3 = math.atan((self.segments[2].point[2] - self.segments[1].point[2]) / (self.segments[2].point[0] - self.segments[1].point[0])) * 180 / math.pi

        # Find angle from Y axis, not x
        theta1 = 90 - theta1
        theta2 = 90 - theta2
        theta3 = 90 - theta3

        return theta1, theta2, theta3

    # Plot target Point
    def plotTarget(self, w):

        if self.plotted:
            # Move to new target position
            self.targetPoint.resetTransform()
            self.targetPoint.translate(self.targetX, self.targetY, self.targetZ)
        else:
            # Plot Target Point
            md = gl.MeshData.sphere(rows=10, cols=20, radius=0.5)
            self.targetPoint = gl.GLMeshItem(
                meshdata=md,
                smooth=True,
                color=(255, 255, 0, 0.3),
                shader="balloon",
                glOptions="additive",
            )
            w.addItem(self.targetPoint)
            self.plotted = True

    def removePlottedTarget(self, w):
        w.removeItem(self.targetPoint)
        self.plotted = False
