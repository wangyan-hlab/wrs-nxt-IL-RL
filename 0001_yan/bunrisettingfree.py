"""
This file defines the obstacles

There are three types of objects in the obstacles:
1. Stationary: tables/aluminium frames
2. Changable: additional obstacles
3. Object

Stationary obstacles will be modeled as CollisionModel.
Only their bounding boxes are used for collision detection.
Changable and stationary objects are modeled as CollisionModelShare.
Either their bounding boxes or their ball approximations could be used for collision detection.
In most of the time, we use ball approximations.

author: weiwei
date: 20190313
"""

import os
import pandaplotutils.pandactrl as pandactrl
import robotsim.ur3dual.ur3dual as ur3dualsim
import robotsim.ur3dual.ur3dualmesh as ur3dualsimmesh
import manipulation.grip.robotiq85.rtq85nm as rtq85nm
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

        self.__this_dir, this_filename = os.path.split(__file__)

        # body
        self.__bodypath = os.path.join(self.__this_dir, "obstacles", "ur3body.stl")
        self.__bodycm = cm.CollisionModel(self.__bodypath, betransparent)
        self.__bodycm.setColor(.3, .3, .3, 1.0)

        # table
        self.__tablepath = os.path.join(self.__this_dir, "obstacles", "ur3dtablefree.stl")
        self.__tablecm = cm.CollisionModel(self.__tablepath, betransparent)
        self.__tablecm.setColor(.55, .55, .5, 1.0)
        self.__tablecm.setPos(50.0, 0.0, 0.0)

        # housing
        ## housing pillar
        self.__beam1200path = os.path.join(self.__this_dir, "obstacles", "ur3housing1200.stl")
        self.__beam1500path = os.path.join(self.__this_dir, "obstacles", "ur3housing1500.stl")
        self.__beam2100path = os.path.join(self.__this_dir, "obstacles", "ur3housing2100.stl")

        self.__pillarfrontrgtcm = cm.CollisionModel(self.__beam2100path)
        self.__pillarfrontrgtcm.setColor(.5, .5, .55, 1.0)
        self.__pillarfrontrgtcm.setPos(860.0 + 50.0 + 30.0, -780.0, 0.0)
        self.__pillarfrontlftcm = cm.CollisionModel(self.__beam2100path)
        self.__pillarfrontlftcm.setColor(.5, .5, .55, 1.0)
        self.__pillarfrontlftcm.setPos(860.0 + 50.0 + 30.0, 780.0, 0.0)
        self.__pillarbackrgtcm = cm.CollisionModel(self.__beam2100path)
        self.__pillarbackrgtcm.setColor(.5, .5, .55, 1.0)
        self.__pillarbackrgtcm.setPos(860 + 50 - 1200, -780.0, 0.0)
        self.__pillarbacklftcm = cm.CollisionModel(self.__beam2100path)
        self.__pillarbacklftcm.setColor(.5, .5, .55, 1.0)
        self.__pillarbacklftcm.setPos(860 + 50 - 1200, 780.0, 0.0)
        ## housing rgt
        self.__rowrgtbottomcm = cm.CollisionModel(self.__beam1200path)
        self.__rowrgtbottomcm.setColor(.5, .5, .55, 1.0)
        self.__rowrgtbottomcm.setPos(860 + 50 - 600, -780.0, 75.0)
        self.__rowrgtmiddlecm = cm.CollisionModel(self.__beam1200path)
        self.__rowrgtmiddlecm.setColor(.5, .5, .55, 1.0)
        self.__rowrgtmiddlecm.setPos(860 + 50 - 600, -780.0, 867.0)
        self.__rowrgttopcm = cm.CollisionModel(self.__beam1200path)
        self.__rowrgttopcm.setColor(.5, .5, .55, 1.0)
        self.__rowrgttopcm.setPos(860 + 50 - 600, -780.0, 2100.0 - 60.0)
        ## housing lft
        self.__rowlftbottomcm = cm.CollisionModel(self.__beam1200path)
        self.__rowlftbottomcm.setColor(.5, .5, .55, 1.0)
        self.__rowlftbottomcm.setPos(860 + 50 - 600, 780.0, 75.0)
        self.__rowlfttopcm = cm.CollisionModel(self.__beam1200path)
        self.__rowlfttopcm.setColor(.5, .5, .55, 1.0)
        self.__rowlfttopcm.setPos(860 + 50 - 600, 780.0, 2100.0 - 60.0)
        ## housing front
        self.__rowfrontbottomcm = cm.CollisionModel(self.__beam1500path)
        self.__rowfrontbottomcm.setColor(.5, .5, .55, 1.0)
        self.__rowfrontbottomcm.setPos(860 + 50 + 30, 0.0, 75.0)
        self.__rowfrontmiddlecm = cm.CollisionModel(self.__beam1500path)
        self.__rowfrontmiddlecm.setColor(.5, .5, .55, 1.0)
        self.__rowfrontmiddlecm.setPos(860 + 50 + 30, 0.0, 867.0)
        # self.__rowfronttopcm = cm.CollisionModel(self.__beam1500path)
        # self.__rowfronttopcm.setColor(.6,.6,.6,1.0)
        # self.__rowfronttopcm.setPos(860+50+30, 0.0, 2100.0-60.0)
        ## housing back
        self.__rowbackbottomcm = cm.CollisionModel(self.__beam1500path)
        self.__rowbackbottomcm.setColor(.5, .5, .55, 1.0)
        self.__rowbackbottomcm.setPos(860 + 50 - 1200, 0.0, 2100.0 - 530.0 - 60.0)
        self.__rowbackmiddlecm = cm.CollisionModel(self.__beam1500path)
        self.__rowbackmiddlecm.setColor(.5, .5, .55, 1.0)
        self.__rowbackmiddlecm.setPos(860 + 50 - 1200, 0.0, 2100.0 - 264.0 - 60.0)
        self.__rowbacktopcm = cm.CollisionModel(self.__beam1500path)
        self.__rowbacktopcm.setColor(.5, .5, .55, 1.0)
        self.__rowbacktopcm.setPos(860 + 50 - 1200, 0.0, 2100.0 - 60.0)

        self.__battached = False
        self.__changableobslist = []

    def reparentTo(self, nodepath):
        if not self.__battached:
            # body
            self.__bodycm.reparentTo(nodepath)
            # table
            self.__tablecm.reparentTo(nodepath)
            # housing
            ## housing pillar
            self.__pillarfrontrgtcm.reparentTo(nodepath)
            self.__pillarfrontlftcm.reparentTo(nodepath)
            self.__pillarbackrgtcm.reparentTo(nodepath)
            self.__pillarbacklftcm.reparentTo(nodepath)
            self.__rowrgtbottomcm.reparentTo(nodepath)
            self.__rowrgtmiddlecm.reparentTo(nodepath)
            self.__rowrgttopcm.reparentTo(nodepath)
            self.__rowlftbottomcm.reparentTo(nodepath)
            self.__rowlfttopcm.reparentTo(nodepath)
            self.__rowfrontbottomcm.reparentTo(nodepath)
            self.__rowfrontmiddlecm.reparentTo(nodepath)
            # self.__rowfronttopcm.reparentTo(nodepath)
            self.__rowbackbottomcm.reparentTo(nodepath)
            self.__rowbackmiddlecm.reparentTo(nodepath)
            self.__rowbacktopcm.reparentTo(nodepath)
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

        stationaryobslist = [self.__tablecm, self.__pillarfrontrgtcm, self.__pillarfrontlftcm,
                             self.__pillarbackrgtcm, self.__pillarbacklftcm,
                             self.__rowrgtbottomcm, self.__rowrgtmiddlecm, self.__rowrgttopcm,
                             self.__rowlftbottomcm, self.__rowlfttopcm,
                             self.__rowfrontbottomcm, self.__rowfrontmiddlecm,  # self.__rowfronttopcm,
                             self.__rowbackbottomcm, self.__rowbackmiddlecm, self.__rowbacktopcm]
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
    import utils.robotmath as rm
    import numpy as np
    from panda3d.core import *

    base = pandactrl.World(camp=[2700, 300, 2700], lookatp=[0, 0, 1000])
    env = Env()
    env.reparentTo(base.render)
    objcm = env.loadobj("bunnysim.stl")

    objcm.setColor(.2, .5, 0, 1)
    objcm.setPos(400, -200, 1200)
    objcm.reparentTo(base.render)
    objcm.showcn()
    obscmlist = env.getstationaryobslist()
    for obscm in obscmlist:
        obscm.showcn()

    objpos = np.array([400, -300, 1200])
    objrot = rm.rodrigues([0, 1, 0], 45)
    objcm2 = env.loadobj("housing.stl")
    objcm2.setColor(1, .5, 0, 1)
    env.addchangableobs(base.render, objcm2, objpos, objrot)

    import trimesh
    objcm3 = cm.CollisionModel(trimesh.primitives.Box(box_center=[500,100,1500], box_extents=[200,50,100]))
    objcm3.setColor(1,0,0,1)
    objcm3.reparentTo(base.render)
    objcm3.showcn()

    robotsim = ur3dualsim.Ur3DualRobot()
    rgthnd = rtq85nm.newHand(hndid="rgt")
    lfthnd = rtq85nm.newHand(hndid="lft")
    robotmeshgen = ur3dualsimmesh.Ur3DualMesh(rgthand=rgthnd, lfthand=lfthnd)
    robotmesh = robotmeshgen.genmnp(robotsim, toggleendcoord=False)
    robotmesh.reparentTo(base.render)

    base.run()
