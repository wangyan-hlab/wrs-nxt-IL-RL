#!/usr/bin/python

"""
The script is written following
http://myenigma.hatenablog.com/entry/2016/03/23/092002
the original file was 2d

# rrt is much faster than rrt connect

author: weiwei
date: 20170609
"""

import random
import copy
import math
import numpy as np


class DDRRT(object):

    def __init__(self, start, goal, ctcallback, expanddis=.5,
                 goalsamplerate=10, maxiter=1000, maxtime = 15.0):
        """

        :param start: nd point, list
        :param goal: nd point, list
        :param ctcallback: an instance of the class in ctcallback.py
        :param expandDis: how much to expand along the vector randpoint - nearestnode
        :param goalsamplerate: bias to set randpoint to be goal
        :param maxIter:

        author: weiwei
        date: 20170609
        """

        self.__start = np.asarray(start)
        self.__end = np.asarray(goal)

        self.__ctcallback = ctcallback
        self.__expanddis = expanddis
        self.__goalsamplerate = goalsamplerate
        self.__maxiter = maxiter
        self.__maxtime = maxtime

        self.__nodeliststart = []
        self.__obstaclelist = []

    @property
    def start(self):
        # read-only property
        return self.__start

    @property
    def end(self):
        # read-only property
        return self.__end

    @property
    def nodeliststart(self):
        # read-only property
        return self.__nodeliststart

    @property
    def ctcallback(self):
        # read-only property
        return self.__ctcallback

    @property
    def expanddis(self):
        # read-only property
        return self.__expanddis

    @property
    def obstaclelist(self):
        # read-only property
        return self.__obstaclelist

    @property
    def nodelist(self):
        # read-only property
        return self.__nodelist

    def setarmid(self, armid):
        self.__ctcallback.setarmid(armid)

    def planning(self, obstaclelist=[], animation=False):
        """
        Pathplanning

        animation: flag for animation on or off

        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...],
                       samples [[joint0, joint1, ...], [joint0, joint1, ...], ...]
        """

        self.__obstaclelist = obstaclelist
        itercount = 0

        # one sampled point is defined as [point, iscollided]
        sampledpoints = []
        self.__nodelist = [Node(self.__start)]

        while True:
            if itercount > self.__maxiter:
                print("failed to find a path")
                break

            # Random Sampling
            while True:
                randpoint = []
                if random.randint(0, 100) > self.__goalsamplerate:
                    for i, jntrng in enumerate(self.__ctcallback.jointlimits):
                        randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                    randpoint = np.asarray(randpoint)
                else:
                    randpoint = copy.deepcopy(self.__end)

                # Find nearest node
                nind = self.__getNearestListIndex(self.__nodelist, randpoint)
                vec = randpoint-self.__nodelist[nind].point
                dvec = np.linalg.norm(vec)
                if dvec < self.__nodelist[nind].radius:
                    break

            vec = vec/dvec

            # expand tree
            nearestnode = self.__nodelist[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind
            newnode.radius = float('inf')
            if self.__ctcallback.iscollided(newnode.point, obstaclelist):
                self.__nodelist[nind].radius = 3*math.sqrt((self.__expanddis**2)*self.__start.size)
                sampledpoints.append([newnode.point, True])
                if animation:
                    drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')
                continue
            else:
                sampledpoints.append([newnode.point, False])
                self.__nodelist.append(newnode)
                if animation:
                    drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')
                # check goal
                d = np.linalg.norm(newnode.point - self.__end)
                if d <= self.__expanddis:
                    print("reaching the goal")
                    break
            itercount += 1

        path = [self.__end.tolist()]
        lastindex = len(self.__nodelist) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelist[lastindex].parent is not None:
            node = self.__nodelist[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())
        path = path[::-1]

        return [path, sampledpoints]

    def __getNearestListIndex(self, nodelist, randpoint):
        dlist = [np.linalg.norm(randpoint-node.point) for node in nodelist]
        minind = dlist.index(min(dlist))
        return minind

class Node():
    """
    RRT Node
    """

    def __init__(self, point):
        """

        :param point: nd point, numpyarray

        radius is added for dynamic domain
        the algorithm follows http://msl.cs.uiuc.edu/~lavalle/papers/YerJaiSimLav05.pdf

        author: weiwei
        date: 20170613
        """

        self.point = point
        self.parent = None
        self.radius = float('inf')

class ctcallback():
    def __init__(self):
        self.__jointlimits = [[-2.0, 15.0], [-2.0, 15.0]]
        pass

    @property
    def jointlimits(self):
        return self.__jointlimits

    def iscollided(self, point, obstaclelist):
        for (obpos, size) in obstaclelist:
            d = np.linalg.norm(np.asarray(obpos) - point)
            if d <= size/2.0:
                return True  # collision

        return False  # safe


def drawwspace(planner, obstaclelist, randconfiguration=None, newconfiguration = None, newconfmark = '^r'):
    """
    Draw Graph
    """
    plt.clf()
    ax = plt.gca()
    ax.set_aspect('equal', 'box')
    plt.grid(True)
    plt.xlim(-4.0, 17.0)
    plt.ylim(-4.0, 17.0)
    for (point, size) in obstaclelist:
        ax.add_patch(plt.Circle((point[0], point[1]), size/2.0, color='k'))
    if randconfiguration is not None:
        plt.plot(randconfiguration[0], randconfiguration[1], "^k")
    if newconfiguration is not None:
        plt.plot(newconfiguration[0], newconfiguration[1], newconfmark)
    for node in planner.nodelist:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodelist[node.parent].point[0]],
                     [node.point[1], planner.nodelist[node.parent].point[1]], '-g')
        if node.radius < float('inf'):
            plt.plot([node.point[0], planner.nodelist[node.parent].point[0]],
                     [node.point[1], planner.nodelist[node.parent].point[1]], '-y')
    plt.plot(planner.start[0], planner.start[1], "xr")
    plt.plot(planner.end[0], planner.end[1], "xr")
    plt.pause(.001)


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import motionplanning.smoother as sm
    plt.pause(5)
    smoother = sm.Smoother()

    # ====Search Path with RRT====
    obstaclelist = [
        ((5, 5), 3),
        ((3, 6), 3),
        ((3, 8), 3),
        ((3, 10), 3),
        ((7, 5), 3),
        ((9, 5), 3)
    ]  # [x,y,size]
    # Set Initial parameters
    ctcallback = ctcallback()
    rrt = DDRRT(start=[0.0, 0.0], goal=[5.0, 10.0], ctcallback=ctcallback, expanddis=.5,
                 goalsamplerate=30, maxiter=1000, maxtime = 15.0)

    import time
    tic = time.time()
    path, sampledpoints = rrt.planning(obstaclelist=obstaclelist, animation=True)
    toc = time.time()
    print(toc-tic)

    # Draw final path
    drawwspace(rrt, obstaclelist)
    plt.plot([point[0] for point in path], [point[1] for point in path], '-k')
    pathsm = smoother.pathsmoothing(path, rrt, 30)
    plt.plot([point[0] for point in pathsm], [point[1] for point in pathsm], '-r')
    plt.grid(True)
    plt.pause(0.001)  # Need for Mac
    plt.show()