import os
import copy
import utiltools.robotmath as rm
import environment.collisionmodel as cm

class Env(object):

    def __init__(self, betransparent=False):
        """
        load obstacles model
        separated by category

        :param base:
        author: weiwei
        date: 20181205
        """

        self.__this_dir, _ = os.path.split(__file__)

        # table
        self.__tablepath = os.path.join(self.__this_dir, "obstacles", "yumi_tablenotop.stl")
        self.__tablecm = cm.CollisionModel(self.__tablepath, betransparent)
        self.__tablecm.setColor(.55, .55, .5, 1.0)
        self.__tablecm.setColor(.45, .45, .35, 1.0)

        # housing
        ## housing pillar
        ## TODO these models could be replaced by trimesh.primitives
        self.__beam2100path = os.path.join(self.__this_dir, "obstacles", "yumi_column60602100.stl")
        self.__beam540path = os.path.join(self.__this_dir, "obstacles", "yumi_column6060540.stl")

        self.__pillarrgtcm = cm.CollisionModel(self.__beam2100path)
        self.__pillarrgtcm.setColor(.5, .5, .55, 1.0)
        self.__pillarrgtcm.setPos(-327.0, -240.0, -1015.0)
        self.__pillarlftcm = copy.deepcopy(self.__pillarrgtcm)
        self.__pillarlftcm.setColor(.5, .5, .55, 1.0)
        self.__pillarlftcm.setPos(-327.0, 240.0, -1015.0)
        ## housing rgt
        self.__rowbackcm = cm.CollisionModel(self.__beam540path)
        self.__rowbackcm.setColor(.5, .5, .55, 1.0)
        self.__rowbackcm.setPos(-327.0, 0.0, 1085.0)
        self.__rowrgtcm = copy.deepcopy(self.__rowbackcm)
        self.__rowrgtcm.setColor(.5, .5, .55, 1.0)
        homomat = self.__rowrgtcm.gethomomat()
        homomat[:3,:3] = rm.rodrigues([0,0,1],90)
        self.__rowrgtcm.sethomomat(homomat)
        self.__rowrgtcm.setPos(-27.0, -240.0, 1085.0)
        self.__rowlftcm = copy.deepcopy(self.__rowbackcm)
        self.__rowlftcm.setColor(.5, .5, .55, 1.0)
        homomat = self.__rowlftcm.gethomomat()
        homomat[:3,:3] = rm.rodrigues([0,0,1],90)
        self.__rowlftcm.sethomomat(homomat)
        self.__rowlftcm.setPos(-27.0, 240.0, 1085.0)
        self.__rowfrontcm = copy.deepcopy(self.__rowbackcm)
        self.__rowfrontcm.setColor(.5, .5, .55, 1.0)
        self.__rowfrontcm.setPos(273.0, -0.0, 1085.0)

        self.__battached = False
        self.__changableobslist = []

    def reparentTo(self, nodepath):
        if not self.__battached:
            # table
            self.__tablecm.reparentTo(nodepath)
            # housing
            ## housing pillar
            self.__pillarrgtcm.reparentTo(nodepath)
            self.__pillarlftcm.reparentTo(nodepath)
            self.__rowbackcm.reparentTo(nodepath)
            self.__rowrgtcm.reparentTo(nodepath)
            self.__rowlftcm.reparentTo(nodepath)
            self.__rowfrontcm.reparentTo(nodepath)
            self.__battached = True

    def loadobj(self, name):
        self.__objpath = os.path.join(self.__this_dir, "objects", name)
        self.__objcm = cm.CollisionModel(self.__objpath, type="ball")
        return self.__objcm

    def getstationaryobslist(self):
        """
        generate the collision model for stationary obstacles

        :return:

        author: weiwei
        date: 20180811
        """

        stationaryobslist = [self.__tablecm, self.__pillarrgtcm, self.__pillarlftcm,
                             self.__rowbackcm, self.__rowrgtcm,
                             self.__rowlftcm, self.__rowfrontcm]
        return stationaryobslist

    def getchangableobslist(self):
        """
        get the collision model for changable obstacles

        :return:

        author: weiwei
        date: 20190313
        """
        return self.__changableobslist

    def addchangableobs(self, nodepath, objcm, pos, rot):
        """

        :param objcm: CollisionModel
        :param pos: nparray 1x3
        :param rot: nparray 3x3
        :return:

        author: weiwei
        date: 20190313
        """

        self.__changableobslist.append(objcm)
        objcm.reparentTo(nodepath)
        objcm.setMat(base.pg.npToMat4(rot, pos))

    def removechangableobs(self, objcm):
        if objcm in self.__changableobslist:
            objcm.remove()


if __name__ == '__main__':
    import utiltools.robotmath as rm
    import numpy as np
    from panda3d.core import *
    import pandaplotutils.pandactrl as pandactrl
    import robotsim.yumi.yumi as robot
    import robotsim.yumi.yumimesh as robotmesh
    import manipulation.grip.yumiintegrated.yumiintegrated as yumiintegrated
    import trimesh

    base = pandactrl.World(camp=[2700, -2000, 2000], lookatp=[0, 0, 500])

    env = Env()
    env.reparentTo(base.render)

    base.pggen.plotAxis(base.render)

    obscmlist = env.getstationaryobslist()
    for obscm in obscmlist:
        obscm.showcn()

    objcm = env.loadobj("bunnysim.stl")
    objcm.setColor(.2, .5, 0, 1)
    objcm.setPos(400, -200, 100)
    objcm.reparentTo(base.render)
    objcm.showcn()

    objpos = np.array([400, -300, 100])
    objrot = rm.rodrigues([0, 1, 0], 45)
    objcm2 = env.loadobj("housing.stl")
    objcm2.setColor(1, .5, 0, 1)
    env.addchangableobs(base.render, objcm2, objpos, objrot)

    objcm3 = cm.CollisionModel(trimesh.primitives.Box(box_center=[500,100,100], box_extents=[200,50,100]))
    objcm3.setColor(1,0,0,1)
    objcm3.reparentTo(base.render)
    objcm3.showcn()

    hndfa = yumiintegrated.YumiIntegratedFactory()
    rgthnd = hndfa.genHand()
    lfthnd = hndfa.genHand()
    rbt = robot.YumiRobot(rgthnd=rgthnd, lfthnd=lfthnd)
    rbtmg = robotmesh.YumiMesh()
    rbtmnp = rbtmg.genmnp(rbt, toggleendcoord=True)
    rbtmnp.reparentTo(base.render)

    base.run()
