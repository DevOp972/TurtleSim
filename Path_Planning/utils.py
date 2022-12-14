import cv2
import math
import numpy as np
import os
import sys

import env as env
# from rrt_connect import Node


class Utils:
    def __init__(self):
        self.env = env.Env()
        self.delta = 0.5

    def is_collision(self, start, end):
        # send tuples of start and end

        black = np.zeros(self.env.bw.shape, dtype=np.uint8)
        # print(type(start[0]))
        start = np.array([start[0], self.env.y_range[1]-start[1]])

        end = np.array([end[0], self.env.y_range[1]-end[1]])
        black = cv2.line(black, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), 255, 2)
        black = cv2.bitwise_and(self.env.bw, black)
        if(np.mean(black) == 0):
            return False
        return True
