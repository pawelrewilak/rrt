import math
import random

from PIL import Image, ImageOps
import matplotlib.pyplot as plt
import numpy as np

img = Image.open('./images/mapa.png')
img = ImageOps.grayscale(img)
img = ImageOps.invert(img)

np_img = np.array(img)
np_img = np_img / 255.0
np_img[np_img > 0.5] = 1.0
plt.set_cmap('binary')
plt.imshow(np_img)

plt.show()


class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

class Tree():
    def __init__(self, start_node):
        self.nodes = [start_node]


    def add_node(self,new_node,parrent_node):
        self.nodes.append(new_node)
        parrent_node.childer.append(new_node)
        new_node.parrent = parrent_node
        return 

    def nearest(self,new_node):
        best = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = math.dist((node.locationX, node.locationY),(new_node.locationX, new_node.locationY))
            if dist < min_dist:
                min_dist = dist
                best = node

        return best
    
    def path_recovery(self,final_node):
        path = []

        while final_node.parent is not None:
            path.append(final_node)
            final_node = final_node.parent
        path.reverse()
        return path
    
    def distance(self,node1,node2):
        return math.dist((node1.locationX,node1.locationY), (node2.locationX,node2.locationY))

def kolizja(p1,p2,mapa):
    x1, y1 = p1.locationX, p1.locationY
    x2, y2 = p2.locationX, p2.locationY
    num_points = max(abs(x1-x2), abs(y1-y2))

    num_points = max(abs(x2 - x1), abs(y2 - y1))
    for i in range(num_points + 1):
        t = i / max(num_points, 1)
        x = int(x1 + (x2 - x1) * t)
        y = int(y1 + (y2 - y1) * t)
        if mapa[y, x] == 0:
            return False
    return True


def samp_point_elipse(start, goal, par_1 = 1.5, par_2 = 0.4):
    d = math.dist(start,goal)
    a = par_1 * d
    b = par_2 * d

    sr_x = (start[0] + goal[0])  / 2
    sr_y = (start[1] + goal[1])  / 2
    theta = math.atan2(goal[1] - start[1], goal[0] - start[0])

    phi = random.uniform(0, 2 * math.pi)
    r = random.uniform(0, 1)
    x_norm= a * r * math.cos(phi)
    y_norm = b * r * math.sin(phi)

    X = x_norm * math.cos(theta) - y_norm * math.sin(theta) + sr_x
    Y = x_norm * math.sin(theta) + y_norm * math.cos(theta) + sr_y

    return treeNode(X,Y)

def rrt_elipse(start, goal, mapa, step_len = 0.05, max_iter = 1000, tolerance = 0.1,goal_bias = 0.01):

    start_node = treeNode(start[0],start[1])
    tree = Tree(start_node)

    for i in range(max_iter):

        if random.random() < goal_bias:
            sampl_node = treeNode(goal[0], goal[1])

        else:
            sampl_node = samp_point_elipse(start,goal)

        pot_parrent = tree.nearest(sampl_node)

        if tree.distance(pot_parrent,sampl_node) <= step_len and kolizja(pot_parrent,sampl_node) is False:
            new_node = sampl_node
            tree.add_node(new_node,pot_parrent)
            
        if tree.distance(pot_parrent,sampl_node) > step_len:
            d = tree.distance(pot_parrent,sampl_node)
            stepX = ((sampl_node.locationX - pot_parrent.locationX) / d) * step_len
            stepY = ((sampl_node.locationY - pot_parrent.locationY) / d) *step_len

            new_node = treeNode(pot_parrent.locationX + stepX, pot_parrent.locationY + stepY)
            if not kolizja(pot_parrent,new_node):
                tree.add_node(new_node,pot_parrent)

        if tree.distance(new_node,goal) < tolerance:
            return tree.path_recovery
    
    return "Path not found"








