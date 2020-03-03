#!/usr/bin/python

"""
The script is written following
http://myenigma.hatenablog.com/entry/2016/03/23/092002
the original file was 2d

author: weiwei
date: 20170609
"""

import gc
import random
import copy
import math
import numpy as np
import rtree
import time

class DDRRTConnect(object):

    def __init__(self, start, goal, ctcallback, expanddis=.5,
                 starttreesamplerate=10, endtreesamplerate=100, maxiter=1000, maxtime = 15.0):
        """

        :param start: nd point, list
        :param goal: nd point, list
        :param ctcallback: an instance of the class in ctcallback.py
        :param expandDis: how much to expand along the vector randpoint - nearestnode
        :param starttreesamplerate: bias to set randpoint to be goal
        :param endtreesamplerate: bias to set randpoint to be start
        :param maxIter:

        :param the last three parameters are for robotsim robots

        author: weiwei
        date: 20170609
        """

        self.__start = np.asarray(start)
        self.__end = np.asarray(goal)

        self.__ctcallback = ctcallback
        self.__expanddis = expanddis
        self.__starttreesamplerate = starttreesamplerate
        self.__endtreesamplerate = endtreesamplerate
        self.__maxiter = maxiter
        self.__maxtime = maxtime

        self.__nodeliststart = []
        self.__nodelistend = []
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
    def nodelistend(self):
        # read-only property
        return self.__nodelistend

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

    def setarmid(self, armid):
        self.__ctcallback.setarmid(armid)

    def planning(self, obstaclelist=[], animation=False):
        """
        Pathplanning

        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...]
        """

        self.__obstaclelist = obstaclelist
        itercount = 0

        # one sampled point: [point, iscollided]
        sampledpoints = []

        self.__nodeliststart = [Node(self.__start)]
        self.__nodelistend = [Node(self.__end)]

        starttreegoal = self.__end
        endtreegoal = self.__start
        tic = time.time()
        while True:
            toc = time.time()
            if self.__maxtime > 0.0:
                if toc-tic > self.__maxtime:
                    print("Too much planning time! Failed to find a path.")
                    return [False, False]
            if itercount > self.__maxiter:
                print("Reach to maximum iteration! Failed to find a path.")
                return [False, False]

            # Random Sampling
            while True:
                randpoint = []
                if random.randint(0, 100) > self.__starttreesamplerate:
                    for i,jntrng in enumerate(self.__ctcallback.jointlimits):
                        randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                    randpoint = np.asarray(randpoint)
                else:
                    randpoint = copy.deepcopy(starttreegoal)

                # Find nearest node
                nind = self.__getNearestListIndex(self.__nodeliststart, randpoint)
                vec = randpoint-self.__nodeliststart[nind].point
                dvec = np.linalg.norm(vec)
                # print(dvec, self.__nodeliststart[nind].radius)
                if dvec <= self.__nodeliststart[nind].radius and dvec > 1e-6:
                    # print("got a new start node")
                    break
            vec = vec/dvec

            # expand tree
            nearestnode = self.__nodeliststart[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind
            newnode.radius = float('inf')

            if animation:
                drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

            if self.__ctcallback.iscollided(newnode.point, obstaclelist):
                self.__nodeliststart[newnode.parent].radius = math.sqrt(((5*self.__expanddis)**2)*self.__start.size)
                sampledpoints.append([newnode.point, True])
                if animation:
                    drawwspace(self, obstaclelist, randpoint, newnode.point, '^b')
                bswap = False
                # if collided, try the other tree
                while True:
                    while True:
                        randpoint = []
                        if random.randint(0, 100) > self.__endtreesamplerate:
                            for i, jntrng in enumerate(self.__ctcallback.jointlimits):
                                randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                            randpoint = np.asarray(randpoint)
                        else:
                            randpoint = copy.deepcopy(endtreegoal)

                        # Find nearest node
                        nind = self.__getNearestListIndex(self.__nodelistend, randpoint)
                        vec = randpoint - self.__nodelistend[nind].point
                        dvec = np.linalg.norm(vec)
                        if dvec < self.__nodelistend[nind].radius and dvec > 1e-6:
                            # print("got a new end node")
                            break
                    vec = vec / dvec

                    # expand tree
                    nearestnode = self.__nodelistend[nind]
                    newnode = copy.deepcopy(nearestnode)
                    newnode.point += self.__expanddis * vec
                    newnode.parent = nind
                    newnode.radius = float('inf')

                    if animation:
                        drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

                    if self.__ctcallback.iscollided(newnode.point, obstaclelist):
                        self.__nodelistend[newnode.parent].radius = math.sqrt(((5*self.__expanddis)**2)*self.__end.size)
                        sampledpoints.append([newnode.point, True])
                        if animation:
                            drawwspace(self, obstaclelist, randpoint, newnode.point, '^b')
                        bswap = True
                        break
                    else:
                        sampledpoints.append([newnode.point, False])
                        self.__nodelistend.append(newnode)
                        starttreegoal = newnode.point# check goal
                        if animation:
                            drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

                        d = np.linalg.norm(newnode.point - endtreegoal)
                        if d <= self.__expanddis:
                            print("reaching endtree goal")
                            bswap = False
                            break
                if bswap:
                    continue
                else:
                    break
            else:
                sampledpoints.append([newnode.point, False])
                self.__nodeliststart.append(newnode)
                endtreegoal = newnode.point
                if animation:
                    drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

                # check goal
                d = np.linalg.norm(newnode.point - starttreegoal)
                if d <= self.__expanddis:
                    print("reaching starttree goal")
                    break

            itercount += 1

        path = []
        lastindex = len(self.__nodelistend) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelistend[lastindex].parent is not None:
            node = self.__nodelistend[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__end.tolist())
        path = path[::-1]
        lastindex = len(self.__nodeliststart) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodeliststart[lastindex].parent is not None:
            node = self.__nodeliststart[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())
        path = path[::-1]

        print("Planning is done")
        return [path, sampledpoints]

    def planning2(self, objcm, obstaclelist, animation=False):
        """
        Pathplanning

        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...]
        """

        self.__obstaclelist = obstaclelist
        itercount = 0

        # one sampled point: [point, iscollided]
        sampledpoints = []

        self.__nodeliststart = [Node(self.__start)]
        self.__nodelistend = [Node(self.__end)]

        starttreegoal = self.__end
        endtreegoal = self.__start
        tic = time.time()
        while True:
            toc = time.time()
            if self.__maxtime > 0.0:
                if toc-tic > self.__maxtime:
                    print("Too much planning time! Failed to find a path.")
                    return [False, False]
            if itercount > self.__maxiter:
                print("Reach to maximum iteration! Failed to find a path.")
                return [False, False]

            # Random Sampling
            while True:
                randpoint = []
                if random.randint(0, 100) > self.__starttreesamplerate:
                    for i,jntrng in enumerate(self.__ctcallback.jointlimits):
                        randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                    randpoint = np.asarray(randpoint)
                else:
                    randpoint = copy.deepcopy(starttreegoal)

                # Find nearest node
                nind = self.__getNearestListIndex(self.__nodeliststart, randpoint)
                vec = randpoint-self.__nodeliststart[nind].point
                dvec = np.linalg.norm(vec)
                # print(dvec, self.__nodeliststart[nind].radius)
                if dvec <= self.__nodeliststart[nind].radius and dvec > 1e-6:
                    # print("got a new start node")
                    break
            vec = vec/dvec

            # expand tree
            nearestnode = self.__nodeliststart[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind
            newnode.radius = float('inf')

            if animation:
                drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

            if self.__ctcallback.iscollided(newnode.point, objcm, obstaclelist):
                self.__nodeliststart[newnode.parent].radius = math.sqrt(((5*self.__expanddis)**2)*self.__start.size)
                sampledpoints.append([newnode.point, True])
                if animation:
                    drawwspace(self, obstaclelist, randpoint, newnode.point, '^b')
                bswap = False
                # if collided, try the other tree
                while True:
                    while True:
                        randpoint = []
                        if random.randint(0, 100) > self.__endtreesamplerate:
                            for i, jntrng in enumerate(self.__ctcallback.jointlimits):
                                randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                            randpoint = np.asarray(randpoint)
                        else:
                            randpoint = copy.deepcopy(endtreegoal)

                        # Find nearest node
                        nind = self.__getNearestListIndex(self.__nodelistend, randpoint)
                        vec = randpoint - self.__nodelistend[nind].point
                        dvec = np.linalg.norm(vec)
                        if dvec < self.__nodelistend[nind].radius and dvec > 1e-6:
                            # print("got a new end node")
                            break
                    vec = vec / dvec

                    # expand tree
                    nearestnode = self.__nodelistend[nind]
                    newnode = copy.deepcopy(nearestnode)

                    # print(self.__expanddis, newnode.point)

                    newnode.point += self.__expanddis * vec
                    newnode.parent = nind
                    newnode.radius = float('inf')

                    if animation:
                        drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

                    if self.__ctcallback.iscollided(newnode.point, objcm, obstaclelist):
                        self.__nodelistend[newnode.parent].radius = math.sqrt(((5*self.__expanddis)**2)*self.__end.size)
                        sampledpoints.append([newnode.point, True])
                        if animation:
                            drawwspace(self, obstaclelist, randpoint, newnode.point, '^b')
                        bswap = True
                        break
                    else:
                        sampledpoints.append([newnode.point, False])
                        self.__nodelistend.append(newnode)
                        starttreegoal = newnode.point# check goal
                        if animation:
                            drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

                        d = np.linalg.norm(newnode.point - endtreegoal)
                        if d <= self.__expanddis:
                            print("reaching endtree goal")
                            bswap = False
                            break
                if bswap:
                    continue
                else:
                    break
            else:
                sampledpoints.append([newnode.point, False])
                self.__nodeliststart.append(newnode)
                endtreegoal = newnode.point
                if animation:
                    drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

                # check goal
                d = np.linalg.norm(newnode.point - starttreegoal)
                if d <= self.__expanddis:
                    print("reaching starttree goal")
                    break

            itercount += 1

        path = []
        lastindex = len(self.__nodelistend) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelistend[lastindex].parent is not None:
            node = self.__nodelistend[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__end.tolist())
        path = path[::-1]
        lastindex = len(self.__nodeliststart) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodeliststart[lastindex].parent is not None:
            node = self.__nodeliststart[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())
        path = path[::-1]

        print("Planning is done")
        return [path, sampledpoints]

    def planninghold(self, objcm, relmat, obstaclelist=[]):
        """
        Pathplanning with object held in hand
        relmat = [relpose, relrot]

        :param objcm collisionmodel
        :param obstaclelist collisionmodel list
        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...]

        author: weiwei
        date: 20190313
        """

        self.__obstaclelist = obstaclelist
        itercount = 0

        # one sampled point: [point, iscollided]
        sampledpoints = []

        self.__nodeliststart = [Node(self.__start)]
        self.__nodelistend = [Node(self.__end)]

        starttreegoal = self.__end
        endtreegoal = self.__start
        tic = time.time()
        while True:
            toc = time.time()
            if self.__maxtime > 0.0:
                if toc-tic > self.__maxtime:
                    print("Too much planning time! Failed to find a path.")
                    return [False, False]
            if itercount > self.__maxiter:
                print("Reach to maximum iteration! Failed to find a path.")
                return [False, False]

            # Random Sampling
            while True:
                randpoint = []
                if random.randint(0, 100) > self.__starttreesamplerate:
                    for i,jntrng in enumerate(self.__ctcallback.jointlimits):
                        randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                    randpoint = np.asarray(randpoint)
                else:
                    randpoint = copy.deepcopy(starttreegoal)

                # Find nearest node
                nind = self.__getNearestListIndex(self.__nodeliststart, randpoint)
                vec = randpoint-self.__nodeliststart[nind].point
                dvec = np.linalg.norm(vec)
                if dvec < self.__nodeliststart[nind].radius and dvec > 1e-6:
                    break
            vec = vec/dvec

            # expand tree
            nearestnode = self.__nodeliststart[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind
            newnode.radius = float('inf')

            if self.__ctcallback.iscollidedHold(newnode.point, [objcm], [relmat], obstaclelist):
                self.__nodeliststart[nind].radius = math.sqrt(((5*self.__expanddis)**2)*self.__start.size)
                sampledpoints.append([newnode.point, True])
                # if collided, try the other tree
                while True:
                    while True:
                        randpoint = []
                        if random.randint(0, 100) > self.__endtreesamplerate:
                            for i, jntrng in enumerate(self.__ctcallback.jointlimits):
                                randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                            randpoint = np.asarray(randpoint)
                        else:
                            randpoint = copy.deepcopy(endtreegoal)

                        # Find nearest node
                        nind = self.__getNearestListIndex(self.__nodelistend, randpoint)
                        vec = randpoint - self.__nodelistend[nind].point
                        dvec = np.linalg.norm(vec)
                        if dvec < self.__nodelistend[nind].radius and dvec > 1e-6:
                            break
                    vec = vec / dvec

                    # expand tree
                    nearestnode = self.__nodelistend[nind]
                    newnode = copy.deepcopy(nearestnode)
                    newnode.point += self.__expanddis * vec
                    newnode.parent = nind
                    newnode.radius = float('inf')

                    if self.__ctcallback.iscollidedHold(newnode.point, [objcm], [relmat], obstaclelist):
                        self.__nodelistend[newnode.parent].radius = math.sqrt(((5*self.__expanddis)**2)*self.__end.size)
                        sampledpoints.append([newnode.point, True])
                        bswap = True
                        break
                    else:
                        sampledpoints.append([newnode.point, False])
                        self.__nodelistend.append(newnode)
                        starttreegoal = newnode.point# check goal

                        d = np.linalg.norm(newnode.point - endtreegoal)
                        if d <= self.__expanddis:
                            print("reaching endtree goal")
                            bswap = False
                            break

                if bswap:
                    continue
                else:
                    break
            else:
                sampledpoints.append([newnode.point, False])
                self.__nodeliststart.append(newnode)
                endtreegoal = newnode.point

                # check goal
                d = np.linalg.norm(newnode.point - starttreegoal)
                if d <= self.__expanddis:
                    print("reaching starttree goal")
                    break

            itercount += 1

        path = []
        lastindex = len(self.__nodelistend) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelistend[lastindex].parent is not None:
            node = self.__nodelistend[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__end.tolist())
        path = path[::-1]
        lastindex = len(self.__nodeliststart) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodeliststart[lastindex].parent is not None:
            node = self.__nodeliststart[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())
        path = path[::-1]

        print("Planning hold is done")
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
    # plt.grid(True)
    plt.xlim(-4.0, 17.0)
    plt.ylim(-4.0, 17.0)
    for (point, size) in obstaclelist:
        ax.add_patch(plt.Circle((point[0], point[1]), size/2.0, color='k'))
    if randconfiguration is not None:
        plt.plot(randconfiguration[0], randconfiguration[1], "^k")
    if newconfiguration is not None:
        plt.plot(newconfiguration[0], newconfiguration[1], newconfmark)
    for node in planner.nodeliststart:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodeliststart[node.parent].point[0]],
                     [node.point[1], planner.nodeliststart[node.parent].point[1]], '-g')
        if node.radius < float('inf'):
            plt.plot([node.point[0], planner.nodeliststart[node.parent].point[0]],
                     [node.point[1], planner.nodeliststart[node.parent].point[1]], '-r')
    for node in planner.nodelistend:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodelistend[node.parent].point[0]],
                     [node.point[1], planner.nodelistend[node.parent].point[1]], '-b')
        if node.radius < float('inf'):
            plt.plot([node.point[0], planner.nodelistend[node.parent].point[0]],
                     [node.point[1], planner.nodelistend[node.parent].point[1]], '-r')
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
    rrt = DDRRTConnect(start=[0.0, 0.0], goal=[5.0, 10.0], ctcallback=ctcallback,
                     starttreesamplerate=30, endtreesamplerate=30,
                     expanddis=.5, maxiter=5000, maxtime = 200.0)
    tic = time.time()
    path, sampledpoints = rrt.planning(obstaclelist=obstaclelist, animation=True)
    toc = time.time()
    print(toc-tic)

    # Draw final path
    drawwspace(rrt, obstaclelist)
    plt.plot([point[0] for point in path], [point[1] for point in path], '-k')
    pathsm = smoother.pathsmoothing(path, rrt, 30)
    # plt.plot([point[0] for point in pathsm], [point[1] for point in pathsm], '-r')
    # plt.grid(True)
    plt.pause(0.001)  # Need for Mac
    plt.show()