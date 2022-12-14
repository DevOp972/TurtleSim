import env as env
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import os
import sys


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = env.Env()

    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        img = self.env.image

        fig, ax = plt.subplots()
        # ax.imshow(img, extent=[0, self.env.x_range[1], self.env.y_range[1], 0])

        plt.plot(self.xI[0], self.xI[1], "gs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "rs", linewidth=3)
        plt.title(name)
        # plt.ylim(self.env.y_range[1], 0)
        print("\n")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        ax.imshow(img, extent=[0, self.env.x_range[1], 0, self.env.y_range[1]])
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [
                             node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.01)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [
                             node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [
                             V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [
                             V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.1)

        plt.pause(0.001)

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1]
                     for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show()
