import os
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt


import env
import utils
import plotting


class Node:
    def __init__(self, n):
        # defines a node with x,y and parent node
        self.x = n[0]
        self.y = n[1]
        self.parent = None

# rrtconnect class


class RrtConnect:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max, x_range, y_range, search_radius):
        # define start,destination,step size,max iterations,v1=graph1 from start,v2=graph2 from dest,x_range,y_range
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.V1 = [self.s_start]
        self.V2 = [self.s_goal]
        self.search_radius = search_radius
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = x_range
        self.y_range = y_range
        # self.obs_circle = self.env.obs_circle
        # self.obs_rectangle = self.env.obs_rectangle
        # self.obs_boundary = self.env.obs_boundary

    def planning(self):
        # self is object of rrt connect class

        for i in range(self.iter_max):
            # generate a random node

            node_rand = self.generate_random_node(
                self.s_goal, self.goal_sample_rate)
            # get nearest neighbor in graph1 from node_rand
            node_near = self.nearest_neighbor(self.V1, node_rand)
            # get new node at step size distance from node_near in direction of node_rand
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision([node_near.x, node_near.y], [node_new.x, node_new.y]):
                # if line joining node_new and node_rand do not collide add node new to graph1
                neighbor_index = self.find_near_neighbor(self.V1, node_new)
                self.V1.append(node_new)
                if neighbor_index:
                    self.choose_parent1(node_new, neighbor_index)
                    self.rewire1(node_new, neighbor_index)
                # find nearest neighbor to node_rand in graph2
                node_near_prim = self.nearest_neighbor(self.V2, node_new)
                # get node at step size distance from nearest neighbour towards node_new attached to graph 1
                node_new_prim = self.new_state(node_near_prim, node_new)

                if node_new_prim and not self.utils.is_collision([node_new_prim.x, node_new_prim.y], [node_near_prim.x, node_near_prim.y]):

                    # if path do not collide add to graph2
                    neighbor_index = self.find_near_neighbor(self.V2, node_new)
                    self.V2.append(node_new_prim)
                    if neighbor_index:
                        self.choose_parent2(node_new, neighbor_index)
                        self.rewire2(node_new, neighbor_index)
                #   loop keeps putting nodes in graph2 till a collision occurs or graph joins
                    while True:
                        # get node at step size distance from node_new in graph2 towards node_new of graph1

                        node_new_prim2 = self.new_state(
                            node_new_prim, node_new)
                        if node_new_prim2 and not self.utils.is_collision([node_new_prim2.x, node_new_prim2.y], [node_new_prim.x, node_new_prim.y]):
                            # add to graph2
                            self.V2.append(node_new_prim2)
                            # set node_new_prim as a node with  parent as node_new_prim but coordinate of prim2
                            node_new_prim = self.change_node(
                                node_new_prim, node_new_prim2)
                        else:
                            break

                        if self.is_node_same(node_new_prim, node_new):
                            # graphs joined
                            break

                if self.is_node_same(node_new_prim, node_new):

                    return self.extract_path(node_new, node_new_prim)
                # join the paths

            # add nodes to the graph with lesser no. of nodes?
            if len(self.V2) < len(self.V1):
                list_mid = self.V2
                self.V2 = self.V1
                self.V1 = list_mid

        return None

    @staticmethod
    def change_node(node_new_prim, node_new_prim2):
        node_new = Node((node_new_prim2.x, node_new_prim2.y))
        node_new.parent = node_new_prim

        return node_new

    @staticmethod
    def is_node_same(node_new_prim, node_new):
        if node_new_prim.x == node_new.x and node_new_prim.y == node_new.y:
            return True

        return False

    def generate_random_node(self, sample_goal, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return sample_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    @staticmethod
    def extract_path(node_new, node_new_prim):
        path1 = [(node_new.x, node_new.y)]
        node_now = node_new

        while node_now.parent is not None:
            node_now = node_now.parent
            path1.append((node_now.x, node_now.y))

        path2 = [(node_new_prim.x, node_new_prim.y)]
        node_now = node_new_prim
        while node_now.parent is not None:
            node_now = node_now.parent
            path2.append((node_now.x, node_now.y))

        return list(list(reversed(path1)) + path2)

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def choose_parent1(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.V1[i], node_new)
                for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.V1[cost_min_index]

    def choose_parent2(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.V2[i], node_new)
                for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.V2[cost_min_index]

    def rewire1(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.V1[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def rewire2(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.V2[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def search_goal_parent(self, graph):
        dist_list = [math.hypot(n.x - self.s_goal.x,
                                n.y - self.s_goal.y) for n in graph]
        node_index = [i for i in range(
            len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(graph[i]) for i in node_index
                         if not self.utils.is_collision([graph[i].x, graph[i].y], [self.s_goal.x, self.s_goal.y])]
            return node_index[int(np.argmin(cost_list))]

        return len(graph) - 1

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def find_near_neighbor(self, graph, node_new):
        n = len(graph) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(
            nd.x - node_new.x, nd.y - node_new.y) for nd in graph]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and not self.utils.is_collision([
            node_new.x, node_new.y], [graph[ind].x, graph[ind].y])]

        return dist_table_index

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:

            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost


def main():
    en = env.Env()
    x_start = (en.start_pts[1][0], en.start_pts[1][1])  # Starting node

    x_goal = (en.goal_pts[0][0], en.goal_pts[0][1])  # Goal node

    rrt_conn = RrtConnect(x_start, x_goal, 10, 0.1,
                          8000, en.x_range, en.y_range, 10)

    path = rrt_conn.planning()
    f = open("path2.txt", "w")
    for (x, y) in path:
        f.write(str(x)+" "+str(y)+"\n")
    f.close()
    rrt_conn.plotting.animation_connect(
        rrt_conn.V1, rrt_conn.V2, path, "RRT_CONNECT")


if __name__ == '__main__':
    main()
