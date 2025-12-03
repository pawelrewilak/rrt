import math

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


def kolizja(p1,p2,mapa):
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])
    num_points = max(abs(x1-x2), abs(y1-y2))

    num_points = max(abs(x2 - x1), abs(y2 - y1))
    for i in range(num_points + 1):
        t = i / max(num_points, 1)
        x = int(x1 + (x2 - x1) * t)
        y = int(y1 + (y2 - y1) * t)
        if mapa[y, x] == 0:
            return False
    return True

def distance(p1,p2):
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])

    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def angle(p1,p2):
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])

    return math.atan2(y2-y1, x2-x1)

def nearestNode(p):
    dystans = []

    for i in range(len(tree)):
        dist = distance(p, tree[i])
        dystans.append(dist)

    return dystans.index(min(dystans))


def rrt(start, goal, mapa, step_len, max_iter, tolerance):

