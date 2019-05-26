import numpy as np

"""Constants"""
__author__ = "tedlin"

# HSV range values for testing sample images
SAMPLE_LOWER = np.array([80, 70, 80])
SAMPLE_UPPER = np.array([100, 300, 300])

# red for real thing
LOWER_LIMIT = np.array([0, 0, 0])
UPPER_LIMIT = np.array([10, 255, 255])

# green for tests
# LOWER_LIMIT = np.array([60, 79, 50])
# UPPER_LIMIT = np.array([80, 255, 255])

# Dimensions in use (some really sketch waterproof camera)
FRAME_X = 320
FRAME_Y = 240
FOV_ANGLE = 57
DEGREES_PER_PIXEL = FOV_ANGLE / FRAME_X
FRAME_CX = 160
FRAME_CY = 120
CAMERA_VERTICAL_ANGLE = 0 # the angle of camera from x plane
TARGET_HEIGHT_DIFFERENCE = 0 # the difference in height (inches) between target and camera on robot
