import numpy as np
import random
import math

class Smoother(object):
    
    def __init__(self, objcm):
        self.__objcm = objcm

    # def __init__(self):
    #     pass

    def __linecdchecker(self, start, goal):
        """

        :param start:
        :param goal:
        :return:

        author: weiwei
        date: 20180519
        """

        nps = np.array(start).reshape(-1,1)
        npg = np.array(goal).reshape(-1,1)
        nele = math.ceil(np.linalg.norm((nps-npg))/self.__expanddis)
        # nele = math.ceil((abs(npg-nps)/self.__expanddis).max())
        ratio = np.linspace(0, 1, nele, endpoint=False)

        jointslist = (nps+(npg-nps)*ratio).T.tolist()
        for joints in jointslist:
            iscollided = self.__ctcallback.iscollided(joints, self.__objcm, self.__obstaclecmlist)
            # iscollided = self.__ctcallback.iscollided(joints, self.__obstaclecmlist)
            if iscollided:
                return False, []
        return True, jointslist

    def __linecdcheckerhold(self, start, goal, objcm, relmat):
        """

        :param start:
        :param goal:
        :return:

        author: weiwei
        date: 20180519
        """

        nps = np.array(start).reshape(-1,1)
        npg = np.array(goal).reshape(-1,1)
        nele = math.ceil(np.linalg.norm((nps-npg))/self.__expanddis)
        # nele = math.ceil((abs(npg-nps)/self.__expanddis).max())
        ratio = np.linspace(0, 1, nele, endpoint=False)

        jointslist = (nps+(npg-nps)*ratio).T.tolist()
        for joints in jointslist:
            iscollided = self.__ctcallback.iscollidedHold(joints, [objcm], [relmat], self.__obstaclecmlist)
            if iscollided:
                return False, []
        return True, jointslist

    def pathsmoothing(self, path, planner, maxiter = 50):
        """
        the path and planner are necessary parameters
        the following member variables of planner will be used for smoothing
        1. ctcallback
        2. expanddis
        3. obstaclelist

        :param path:
        :param planner:
        :return:

        author: weiweiz
        date: 20180519
        """

        print("Smoothing...")

        self.__ctcallback = planner.ctcallback
        self.__expanddis = planner.expanddis
        self.__obstaclecmlist = planner.obstaclelist

        pathlength = len(path)
        if(pathlength <= 3):
            result, addpath = self.__linecdchecker(path[0], path[-1])
            if result:
                path = [path[0]]+addpath+[path[-1]]
            return path

        for i in range(maxiter):
            pickpoint0 = random.randint(0, pathlength-3)
            pickpoint1 = random.randint(pickpoint0+1, pathlength-1)
            result, addpath = self.__linecdchecker(path[pickpoint0], path[pickpoint1])
            if result:
                path = path[:pickpoint0]+addpath+path[pickpoint1:]
                pathlength = len(path)
                if pathlength <= 3:
                    break

        print("Smoothing is done")
        return path

    def pathsmoothinghold(self, path, planner, objcm, relmat, maxiter = 50):
        """
        the path and planner are necessary parameters
        the following member variables of planner will be used for smoothing
        1. ctcallback
        2. expanddis
        3. obstaclelist

        :param path:
        :param planner:
        :return:

        author: weiweiz
        date: 20180519
        """

        print("Smoothing hold...")

        self.__ctcallback = planner.ctcallback
        self.__expanddis = planner.expanddis
        self.__obstaclecmlist = planner.obstaclelist

        pathlength = len(path)
        if(pathlength <= 3):
            result, addpath = self.__linecdchecker(path[0], path[-1])
            if result:
                path = [path[0]]+addpath+[path[-1]]
            return path

        for i in range(maxiter):
            pickpoint0 = random.randint(0, pathlength-3)
            pickpoint1 = random.randint(pickpoint0+1, pathlength-1)
            result, addpath = self.__linecdcheckerhold(path[pickpoint0], path[pickpoint1], objcm, relmat)
            if result:
                path = path[:pickpoint0]+addpath+path[pickpoint1:]
                pathlength = len(path)
                if pathlength <= 3:
                    break

        print("Smoothing hold is done")
        return path
