import os
import pandaplotutils.pandactrl as pandactrl
import robotsim.nextage.nxt as robotsim
import robotsim.nextage.nxtmesh as robotmesh
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

        # table
        self.__tablepath = os.path.join(self.__this_dir, "obstacles", "nxttable.stl")
        self.__tablecm = cm.CollisionModel(self.__tablepath, betransparent)
        self.__tablecm.setColor(.6,.6,.6,1.0)
        self.__tablecm.setPos(190.0, 0.0, 0.0)

        # shelf
        self.__belowfrontbeampath = os.path.join(self.__this_dir, "obstacles", "nxt_belowfront_beam_800x30x30.stl")
        self.__belowleftbeampath = os.path.join(self.__this_dir, "obstacles", "nxt_belowleft_beam_600x30x30.stl")
        self.__frontleftbeampath = os.path.join(self.__this_dir, "obstacles", "nxt_frontleft_beam_850x30x30.stl")
        self.__topleftbeampath = os.path.join(self.__this_dir, "obstacles", "nxt_topleft_beam_300x30x30.stl")

        self.__framebelowrgtcm = cm.CollisionModel(self.__belowleftbeampath)
        self.__framebelowrgtcm.setColor(.6,.6,.6,1.0)
        self.__framebelowrgtcm.setPos(300.0+190.0, -415.0, 912.0)
        self.__framebelowlftcm = cm.CollisionModel(self.__belowleftbeampath)
        self.__framebelowlftcm.setColor(.6,.6,.6,1.0)
        self.__framebelowlftcm.setPos(300.0+190.0, 415.0, 912.0)
        self.__framebelowfrontcm = cm.CollisionModel(self.__belowfrontbeampath)
        self.__framebelowfrontcm.setColor(.6,.6,.6,1.0)
        self.__framebelowfrontcm.setPos(600.0-15.0+190.0, 0.0, 912.0)
        self.__framebelowbackcm = cm.CollisionModel(self.__belowfrontbeampath)
        self.__framebelowbackcm.setColor(.6,.6,.6,1.0)
        self.__framebelowbackcm.setPos(15.0+190.0, 0.0, 912.0)
        self.__framefrontrgtcm = cm.CollisionModel(self.__frontleftbeampath)
        self.__framefrontrgtcm.setColor(.6,.6,.6,1.0)
        self.__framefrontrgtcm.setPos(600.0-15.0+190.0, -415.0, 912.0+425.0)
        self.__framefrontlftcm = cm.CollisionModel(self.__frontleftbeampath)
        self.__framefrontlftcm.setColor(.6,.6,.6,1.0)
        self.__framefrontlftcm.setPos(600.0-15.0+190.0, 415.0, 912.0+425.0)
        self.__frametoprgtcm = cm.CollisionModel(self.__topleftbeampath)
        self.__frametoprgtcm.setColor(.6,.6,.6,1.0)
        self.__frametoprgtcm.setPos(600.0-150.0+190.0, -415.0, 912.0+850.0+15.0)
        self.__frametoplftcm = cm.CollisionModel(self.__topleftbeampath)
        self.__frametoplftcm.setColor(.6,.6,.6,1.0)
        self.__frametoplftcm.setPos(600.0-150.0+190.0, 415.0, 912.0+850.0+15.0)
        self.__frametopfrontcm = cm.CollisionModel(self.__belowfrontbeampath)
        self.__frametopfrontcm.setColor(.6,.6,.6,.3)
        self.__frametopfrontcm.setPos(600.0-15.0+190.0, 0.0, 912.0+850.0+15.0)
        self.__frametopbackcm = cm.CollisionModel(self.__belowfrontbeampath)
        self.__frametopbackcm.setColor(.6,.6,.6,.3)
        self.__frametopbackcm.setPos(600.0-300.0+15.0+190.0, 0.0, 912.0+850.0+15.0)

        # housing
        ## housing pillar
        # self.__beam1200path = os.path.join(self.__this_dir, "obstacles", "ur3housing1200.stl")
        # self.__beam1500path = os.path.join(self.__this_dir, "obstacles", "ur3housing1500.stl")
        # self.__beam2100path = os.path.join(self.__this_dir, "obstacles", "ur3housing2100.stl")
        #
        # self.__pillarfrontrgtcm = cm.CollisionModel(self.__beam2100path)
        # self.__pillarfrontrgtcm.setColor(.6,.6,.6,1.0)
        # self.__pillarfrontrgtcm.setPos(860.0+50.0+30.0, -780.0, 0.0)
        # self.__pillarfrontlftcm = cm.CollisionModel(self.__beam2100path)
        # self.__pillarfrontlftcm.setColor(.6,.6,.6,1.0)
        # self.__pillarfrontlftcm.setPos(860.0+50.0+30.0, 780.0, 0.0)
        # self.__pillarbackrgtcm = cm.CollisionModel(self.__beam2100path)
        # self.__pillarbackrgtcm.setColor(.6,.6,.6,1.0)
        # self.__pillarbackrgtcm.setPos(860+50-1200, -780.0, 0.0)
        # self.__pillarbacklftcm = cm.CollisionModel(self.__beam2100path)
        # self.__pillarbacklftcm.setColor(.6,.6,.6,1.0)
        # self.__pillarbacklftcm.setPos(860+50-1200, 780.0, 0.0)
        # ## housing rgt
        # self.__rowrgtbottomcm = cm.CollisionModel(self.__beam1200path)
        # self.__rowrgtbottomcm.setColor(.6,.6,.6,1.0)
        # self.__rowrgtbottomcm.setPos(860+50-600, -780.0, 75.0)
        # self.__rowrgtmiddlecm = cm.CollisionModel(self.__beam1200path)
        # self.__rowrgtmiddlecm.setColor(.6,.6,.6,1.0)
        # self.__rowrgtmiddlecm.setPos(860+50-600, -780.0, 867.0)
        # self.__rowrgttopcm = cm.CollisionModel(self.__beam1200path)
        # self.__rowrgttopcm.setColor(.6,.6,.6,1.0)
        # self.__rowrgttopcm.setPos(860+50-600, -780.0, 2100.0-60.0)
        # ## housing lft
        # self.__rowlftbottomcm = cm.CollisionModel(self.__beam1200path)
        # self.__rowlftbottomcm.setColor(.6,.6,.6,1.0)
        # self.__rowlftbottomcm.setPos(860+50-600, 780.0, 75.0)
        # self.__rowlfttopcm = cm.CollisionModel(self.__beam1200path)
        # self.__rowlfttopcm.setColor(.6,.6,.6,1.0)
        # self.__rowlfttopcm.setPos(860+50-600, 780.0, 2100.0-60.0)
        # ## housing front
        # self.__rowfrontbottomcm = cm.CollisionModel(self.__beam1500path)
        # self.__rowfrontbottomcm.setColor(.6,.6,.6,1.0)
        # self.__rowfrontbottomcm.setPos(860+50+30, 0.0, 75.0)
        # self.__rowfrontmiddlecm = cm.CollisionModel(self.__beam1500path)
        # self.__rowfrontmiddlecm.setColor(.6,.6,.6,1.0)
        # self.__rowfrontmiddlecm.setPos(860+50+30, 0.0, 867.0)
        # # self.__rowfronttopcm = cm.CollisionModel(self.__beam1500path)
        # # self.__rowfronttopcm.setColor(.6,.6,.6,1.0)
        # # self.__rowfronttopcm.setPos(860+50+30, 0.0, 2100.0-60.0)
        # ## housing back
        # self.__rowbackbottomcm = cm.CollisionModel(self.__beam1500path)
        # self.__rowbackbottomcm.setColor(.6,.6,.6,1.0)
        # self.__rowbackbottomcm.setPos(860+50-1200, 0.0, 2100.0-530.0-60.0)
        # self.__rowbackmiddlecm = cm.CollisionModel(self.__beam1500path)
        # self.__rowbackmiddlecm.setColor(.6,.6,.6,1.0)
        # self.__rowbackmiddlecm.setPos(860+50-1200, 0.0, 2100.0-264.0-60.0)
        # self.__rowbacktopcm = cm.CollisionModel(self.__beam1500path)
        # self.__rowbacktopcm.setColor(.6,.6,.6,1.0)
        # self.__rowbacktopcm.setPos(860+50-1200, 0.0, 2100.0-60.0)

        self.__battached = False
        self.__changableobslist = []

    def reparentTo(self, nodepath):
        if not self.__battached:
            # table
            self.__tablecm.reparentTo(nodepath)
            # shelf
            self.__framebelowrgtcm.reparentTo(nodepath)
            self.__framebelowlftcm.reparentTo(nodepath)
            self.__framebelowfrontcm.reparentTo(nodepath)
            self.__framebelowbackcm.reparentTo(nodepath)
            self.__framefrontrgtcm.reparentTo(nodepath)
            self.__framefrontlftcm.reparentTo(nodepath)
            self.__frametoprgtcm.reparentTo(nodepath)
            self.__frametoplftcm.reparentTo(nodepath)
            self.__frametopfrontcm.reparentTo(nodepath)
            self.__frametopbackcm.reparentTo(nodepath)
            ## housing pillar
            # self.__pillarfrontrgtcm.reparentTo(nodepath)
            # self.__pillarfrontlftcm.reparentTo(nodepath)
            # self.__pillarbackrgtcm.reparentTo(nodepath)
            # self.__pillarbacklftcm.reparentTo(nodepath)
            # self.__rowrgtbottomcm.reparentTo(nodepath)
            # self.__rowrgtmiddlecm.reparentTo(nodepath)
            # self.__rowrgttopcm.reparentTo(nodepath)
            # self.__rowlftbottomcm.reparentTo(nodepath)
            # self.__rowlfttopcm.reparentTo(nodepath)
            # self.__rowfrontbottomcm.reparentTo(nodepath)
            # self.__rowfrontmiddlecm.reparentTo(nodepath)
            # # self.__rowfronttopcm.reparentTo(nodepath)
            # self.__rowbackbottomcm.reparentTo(nodepath)
            # self.__rowbackmiddlecm.reparentTo(nodepath)
            # self.__rowbacktopcm.reparentTo(nodepath)
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

        # stationaryobslist = [self.__tablecm, self.__pillarfrontrgtcm, self.__pillarfrontlftcm,
        #                      self.__pillarbackrgtcm, self.__pillarbacklftcm,
        #                      self.__rowrgtbottomcm, self.__rowrgtmiddlecm, self.__rowrgttopcm,
        #                      self.__rowlftbottomcm, self.__rowlfttopcm,
        #                      self.__rowfrontbottomcm, self.__rowfrontmiddlecm, #self.__rowfronttopcm,
        #                      self.__rowbackbottomcm, self.__rowbackmiddlecm, self.__rowbacktopcm,]

        stationaryobslist = [self.__tablecm, self.__framebelowrgtcm, self.__framebelowlftcm,
                             self.__framebelowfrontcm, self.__framebelowbackcm,
                             self.__framefrontrgtcm, self.__framefrontlftcm, self.__frametoprgtcm,
                             self.__frametoplftcm, self.__frametopfrontcm,
                             self.__frametopbackcm]
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

    base = pandactrl.World(camp=[2700,300,2700], lookatp=[0,0,1000])
    env = Env()
    env.reparentTo(base.render)
    objcm = env.loadobj("bunnysim.stl")

    objcm.setColor(.2,.5,0,1)
    objcm.setPos(400,-200,1200)
    objcm.reparentTo(base.render)
    objcm.showcn()
    obscmlist = env.getstationaryobslist()
    for obscm in obscmlist:
        obscm.showcn()

    objpos = np.array([400,-300,1200])
    objrot = rm.rodrigues([0,1,0], 45)
    objcm2 = env.loadobj("housing.stl")
    objcm2.setColor(1,.5,0,1)
    env.addchangableobs(base.render, objcm2, objpos, objrot)

    robotsim = robotsim.NxtRobot()
    rgthnd = rtq85nm.newHand(hndid = "rgt", ftsensoroffset=0)
    lfthnd = rtq85nm.newHand(hndid = "lft", ftsensoroffset=0)
    robotmeshgen = robotmesh.NxtMesh(rgthand = rgthnd, lfthand = lfthnd)
    robotmesh = robotmeshgen.genmnp(robotsim, toggleendcoord=False)
    robotmesh.reparentTo(base.render)

    base.pggen.plotAxis(base.render, spos=Vec3(400,-300,900))
    base.pggen.plotAxis(base.render, spos=Vec3(300,0,1300))

    base.run()