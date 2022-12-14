from unittest import result
import cv2
import numpy as np
from image_utils import *
# Let's load a simple image with 3 black squares


class Env:
    def __init__(self):

        self.image = cv2.imread(
            'C:\\Users\\JBSR-6-2021\\Documents\\krssg\\software\\img2.png')
        self.bw, self.bw_start, self.bw_goal = getImages(self.image)
        self.start_pts = getCircles(self.bw_start)
        self.goal_pts = getCircles(self.bw_goal)
        self.x_range = (0, self.image.shape[1])
        self.y_range = (0, self.image.shape[0])
