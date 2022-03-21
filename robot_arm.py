import math
import random
import sys
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2
from collections import deque
from scipy.linalg import qr, svd
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui


class RobotArm:
    """
    Will be used to
    - create rendering of arm
    - keep track of each joint rotation
    - Possibly inverse kinematics (maybe another class)
    """