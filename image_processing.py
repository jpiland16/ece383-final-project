#!/usr/bin/python3

"""
This code can process images of the Connect4 board.

Jonathan Piland
2 December 2023
"""

import rospy
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from datetime import datetime
from dataclasses import dataclass
import numpy as np
from typing import List
from cv_bridge import CvBridge
bridge = CvBridge()

class VideoCamera():

    def __init__(self) -> None:

        blank_image = np.zeros([800, 800, 3], dtype=np.uint8)
        self._current_image = blank_image
        rospy.Subscriber("ur5e/camera1/image_raw", Image, self._callback)

    def _callback(self, msg: Image):
        self._current_image = bridge.imgmsg_to_cv2(msg, "rgb8")

    def get_snapshot(self):
        return self._current_image.astype(np.float32) / 255


def get_column_played(column_heights: List[int], previous_snapshot: np.ndarray, current_snapshot: np.ndarray):
    """
    IMPORTANT NOTE: both images must be on the scale of 0 to 1.
    """

    if np.max(previous_snapshot) > 1:
        raise ValueError("Max of image is too large. Is it on the scale of 0 to 1?")
    if np.max(current_snapshot) > 1:
        raise ValueError("Max of image is too large. Is it on the scale of 0 to 1?")

    image = current_snapshot - previous_snapshot
    image = (image - np.min(image)) / (np.max(image) - np.min(image))

    if len(column_heights) !=  7:
        raise ValueError("Invalid length of column heights array")
    for height in column_heights:
        if (not (0 <= height <= 6)) or (int(height) != height):
            raise ValueError(f"Invalid column height: {height}")
    
    rectangles: List[List[RectangleBounds]] = []

    for column in range(7):
        rectangles.append([])
        for rect in range(6):
            rectangles[-1].append(RectangleBounds(
                125 + column * 88,
                610 - rect * 80,
                125 + column * 88 + 60,
                610 - rect * 80 + 60
            ))

    rectangles_of_interest = []

    for column in range(7):
        if column_heights[column] == 6:
            rectangles_of_interest.append(None)
            continue
        rectangles_of_interest.append(rectangles[column][column_heights[column]])

    deltas = []

    for roi in rectangles_of_interest:
        if roi is None:
            deltas.append(1e99) # large number that will not be the minimum
            continue
        average_change = np.average(image[roi.y1:roi.y2, roi.x1:roi.x2])
        deltas.append(average_change)

    # print(deltas)
    
    return 1 + np.argmin(deltas)


@dataclass
class RectangleBounds:
    x1: int
    y1: int
    x2: int
    y2: int
