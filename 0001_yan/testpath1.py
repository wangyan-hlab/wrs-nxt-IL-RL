import os
from panda3d.core import *
import pandaplotutils.pandactrl as pc
import pandaplotutils.pandageom as pg
import bldgfsettingnear
import environment.bulletcdhelper as bcdh
import motionplanning.rrt.ddrrtconnect as ddrrtc
import motionplanning.smoother as sm
import motionplanning.ctcallback as ctcb
import numpy as np
import utiltools.robotmath as rm
import math


class CtCallback(object):
    def __init__(self):
        self.__jointlimits = [[-135.0, -45.0], [-1.0, 1.0], [30.0, 140.0],
                              [-2.0, 2.0], [0.0, 13.0], [-2.0, 2.0]]
        pass

    @property
    def jointlimits(self):
        return self.__jointlimits

    def iscollided(self, xyzrpy, objcm, obstaclecmlist):
        """

        :param object:
        :param obstaclecdnplist:
        :return:

        author: weiwei, hao
        date: 20190429
        """
        objcm.setPos(xyzrpy[0],xyzrpy[1],xyzrpy[2])
        objcm.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5])

        checker = bcdh.MCMchecker()
        result = checker.isMeshMeshListCollided(objcm, obstaclecmlist)
        if result:
            return True
        return False

        # oocnp = NodePath("collision nodepath")
        # objcnplist = []
        # objcnplist.append(objcm.copycdnpTo(oocnp))
        # for obstaclecm in obstaclecmlist:
        #     obstaclecm.copycdnpTo(oocnp)
        # ctrav = CollisionTraverser()
        # chan = CollisionHandlerQueue()
        # for objcnp in objcnplist:
        #     objcnp.node().setFromCollideMask(BitMask32(0x1))
        #     objcnp.setCollideMask(BitMask32(0x2))
        #     ctrav.addCollider(objcnp, chan)
        # ctrav.traverse(oocnp)
        # if chan.getNumEntries() > 0:
        #     print("objects-others collision")
        #     return True
        # return False


if __name__=="__main__":
    base = pc.World(camp=[0, 800, 0], lookatp=[0, 0, 0], up=[0, 0, 1], fov=40, w=1920, h=1080)
    env = bldgfsettingnear.Env()
    # self.env.reparentTo(base.render)
    objname = "new_LSHAPE.stl"
    objcm = env.loadobj(objname)
    groove = env.loadobj("new_GROOVE.stl")
    posG = base.pg.npToMat4(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                            np.array([0, 0, 0]))
    virtualgoalpos = np.array([posG[3][0], posG[3][1], posG[3][2]])
    virtualgoalrot = rm.euler_from_matrix(pg.npToMat3(np.array([[posG[0][0], posG[0][1], posG[0][2]],
                                                                [posG[1][0], posG[1][1], posG[1][2]],
                                                                [posG[2][0], posG[2][1], posG[2][2]]])))
    groove.setPos(virtualgoalpos[0], virtualgoalpos[1], virtualgoalpos[2])
    groove.setRPY(virtualgoalrot[0], virtualgoalrot[1], virtualgoalrot[2])
    groove.setColor(0, 0, 1, 0.4)
    groove.reparentTo(base.render)

    # relpos = np.array([-60.14147593, -0.43612779, 40.08426847])
    # relrot = np.array([[0.9672951, 0.003912, 0.25362363],
    #                    [-0.00265115, 0.99998247, -0.00531303],
    #                    [-0.25363992, 0.00446691, 0.9672884]])
    relpos = np.array([-56.28811304,  -3.55672667,  40.3790682])
    relrot = np.array([[ 0.96447225, -0.01227223,  0.2638989 ],
       [ 0.02384718,  0.99888664, -0.04070254],
       [-0.26310561,  0.04554969,  0.96369113]])
    relmat = base.pg.npToMat4(relrot, relpos)
    posL = relmat * posG
    print(posL)
    virtualgoalpos0 = np.array([posL[3][0], posL[3][1], posL[3][2]])
    virtualgoalrot0 = rm.euler_from_matrix(pg.npToMat3(np.array([[posL[0][0], posL[0][1], posL[0][2]],
                                                                 [posL[1][0], posL[1][1], posL[1][2]],
                                                                 [posL[2][0], posL[2][1], posL[2][2]]])))
    print(virtualgoalrot0[0], virtualgoalrot0[1], virtualgoalrot0[2])
    # objcm.setPos(-63.11388425,  0,  44.4996245)
    # objcm.setRPY(0,15,0)
    objcm.setPos(-60,  0,  40.5)
    objcm.setRPY(0, 14, 0)
    # objcm.setPos(-45.2, 0, 30.2)
    # objcm.setRPY(0, 0, 0)
    objcm.setColor(1, 1, 0, 0.7)
    objcm.reparentTo(base.render)

    obscmlist = []
    obscmlist.append(groove)
    checker = bcdh.MCMchecker()
    result = checker.isMeshMeshListCollided(objcm, obscmlist)
    print(result)

    # # planner = ddrrtc.DDRRTConnect()
    # ctcallback = CtCallback()
    # start = np.array([virtualgoalpos0[0], virtualgoalpos0[1], virtualgoalpos0[2],
    #                   virtualgoalrot0[0], virtualgoalrot0[1], virtualgoalrot0[2]], dtype=float)
    # print(start)
    # goal = np.array([-45, 0, 30, 0, 0, 0], dtype=float)
    # print(goal)
    # planner = ddrrtc.DDRRTConnect(start=start, goal=goal, ctcallback=ctcallback,
    #                               starttreesamplerate=30, endtreesamplerate=30,
    #                               expanddis=.5, maxiter=5000, maxtime=200.0)
    #
    # obscmlist = []
    # obscmlist.append(groove)
    # # obstaclelist = bcdh.genBulletCDMeshList(obscmlist)
    # [path, sampledpoints] = planner.planning2(objcm, obscmlist)
    # smoother = sm.Smoother()
    # path = smoother.pathsmoothing(path, planner)
    # print(path)

    base.run()
