#!/usr/bin/python

import os
import itertools

import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.core import *
import copy

import trimesh
from pandaplotutils import pandageom as pg
from utiltools import collisiondetection as npcd
from environment import collisiondetection as cmcd
from database import dbcvt as dc
from matplotlib import collections as mc
from operator import add

import networkx as nx
import math
import manipulation.regrasp.floatingposes as floatingposes


# regriptppfp means regrip using tabletop placements and floating poses
class RegripTppFp():

    def __init__(self, objpath, robot, handpkg, gdb, base, obstaclecmlist, transitoption = "useplacement"):
        """

        :param objpath:
        :param robot:
        :param handpkg:
        :param gdb:
        :param base:
        :param obstaclecmlist: a list of collisionmodels as boxes, their type must be "box"
        :param armoption: 0 both handover and placement; 1 handover only; 2 placement only (single arm)
        :param transitoption: "useplacement", "usehandover", "useboth"
        """

        if(transitoption not in ["useplacement", "usehandover", "useboth"]):
            print("The transitoption of RegripTppFp must be one of the three values in the API!")
            raise Exception("The transitoption is illegal!")

        self.gdb = gdb
        self.handpkg = handpkg
        self.robot = robot
        self.base = base
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        self.regg = nx.Graph()

        self.ndiscreterot = 0
        self.nplacements = 0
        self.globalgripids = []

        # for removing the grasps at start and goal
        self.robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])

        # obstacles to remove hand
        self.bulletworld = BulletWorld()
        self.boxesbullnode = cmcd.genBulletCDBoxes(obstaclecmlist)
        self.bulletworld.attachRigidBody(self.boxesbullnode)

        # toggle the following code to debug bulletworld
        # ''' start '''
        # debugNode = BulletDebugNode('Debug')
        # debugNode.showWireframe(True)
        # debugNode.showConstraints(True)
        # debugNode.showBoundingBoxes(False)
        # debugNode.showNormals(False)
        # debugNP = base.render.attachNewNode(debugNode)
        # debugNP.show()
        # self.bulletworld.setGravity(Vec3(0, 0, -9.81))
        # self.bulletworld.setDebugNode(debugNP.node())
        # def update(task):
        #     dt = globalClock.getDt()
        #     self.bulletworld.doPhysics(dt)
        #     return task.cont
        # taskMgr.add(update, 'update')
        # ''' end '''

        self.startrgtnodeids = []
        self.startlftnodeids = []
        self.goalrgtnodeids = []
        self.goallftnodeids = []
        self.shortestpaths = None

        self.startrotmat4 = pg.np4ToMat4(np.eye(4))
        self.goalrotmat4 = pg.np4ToMat4(np.eye(4))

        # load retraction distances
        self.rethanda, self.retworlda, self.worlda = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda should computed by assembly planner

        self.floatingposes = floatingposes.FloatingPoses(objpath, gdb, handpkg, base)
        self.floatingposes.loadIKfeasibleGPfromDB(robot)

        # loadfreeairgrip
        self.__loadFreeAirGrip()
        self.__buildGraphs(armname = "rgt", transitoption = transitoption)
        self.__buildGraphs(armname = "lft", transitoption = transitoption)
        self.__bridgeGraph()
        self.reggbk = copy.deepcopy(self.regg)

        # shortestpaths
        self.directshortestpaths_startrgtgoalrgt = []
        self.directshortestpaths_startrgtgoallft = []
        self.directshortestpaths_startlftgoalrgt = []
        self.directshortestpaths_startlftgoallft = []

        # for fast path plot
        self.gnodesplotpos = {}

    def reset(self):
        self.regg = copy.deepcopy(self.reggbk)
        # shortestpaths
        self.directshortestpaths_startrgtgoalrgt = []
        self.directshortestpaths_startrgtgoallft = []
        self.directshortestpaths_startlftgoalrgt = []
        self.directshortestpaths_startlftgoallft = []

        self.startrgtnodeids = []
        self.startlftnodeids = []
        self.goalrgtnodeids = []
        self.goallftnodeids = []
        self.shortestpaths = None

        self.startrotmat4 = pg.np4ToMat4(np.eye(4))
        self.goalrotmat4 = pg.np4ToMat4(np.eye(4))

    def __loadFreeAirGrip(self):
        """
        load self.freegripids, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, handname = self.handpkg.getHandName())
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripids = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def __buildGraphs(self, armname="rgt", transitoption="useboth"):
        """
        load tabletopgrips
        retraction distance are also loaded from database

        :param robot: an robot defined in robotsim.hrp5 or robotsim.nextage
        :param gdb: an object of the database.GraspDB class
        :param idarm: value = 1 "lft" or 2 "rgt", which arm to use
        :param transitoption: "useplacement", "usehandover", "useboth"
        :return:

        author: weiwei
        date: 20170112
        """

        # load idarm
        idarm = self.gdb.loadIdArm(armname)

        # get the global grip ids
        # and prepare the global edges
        # for each globalgripid, find all its tabletopids (pertaining to placements)
        globalidsedges = {}
        for ggid in self.freegripids:
            globalidsedges[str(ggid)] = []
            self.globalgripids.append(ggid)

        if((transitoption is "useboth") or (transitoption is "useplacement")):
            sql = "SELECT tabletopplacements.idtabletopplacements, angle.value, \
                    tabletopplacements.idfreetabletopplacement, tabletopplacements.tabletopposition, \
                    tabletopplacements.rotmat FROM \
                    tabletopplacements,freetabletopplacement,angle,object WHERE \
                    tabletopplacements.idangle=angle.idangle AND \
                    tabletopplacements.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                    freetabletopplacement.idobject=object.idobject AND \
                    object.name LIKE '%s' AND angle.value IN (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)" \
                  % self.dbobjname
            result = self.gdb.execute(sql)
            if len(result) != 0:
                tpsrows = np.array(result)
                # nubmer of discreted rotation
                self.angles = list(set(map(float, tpsrows[:, 1])))
                # for plotting
                self.fttpsids = list(set(map(int, tpsrows[:, 2])))
                self.nfttps = len(self.fttpsids)

                idrobot = self.gdb.loadIdRobot(self.robot)
                for i, idtps in enumerate(tpsrows[:, 0]):
                    sql = "SELECT tabletopgrips.idtabletopgrips, tabletopgrips.contactpnt0, tabletopgrips.contactpnt1, \
                            tabletopgrips.rotmat, tabletopgrips.jawwidth, tabletopgrips.idfreeairgrip, \
                            iktabletopgrips.jnts, iktabletopgrips.jnts_handa, iktabletopgrips.jnts_worlda \
                            FROM tabletopgrips,iktabletopgrips,freeairgrip,hand WHERE tabletopgrips.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                            freeairgrip.idhand = hand.idhand AND\
                            tabletopgrips.idtabletopgrips=iktabletopgrips.idtabletopgrips AND \
                            tabletopgrips.idtabletopplacements = %d AND iktabletopgrips.idrobot=%d AND \
                            iktabletopgrips.feasibility='True' AND iktabletopgrips.feasibility_handa='True' \
                            AND iktabletopgrips.feasibility_worlda='True' AND iktabletopgrips.idarm = %d AND hand.name LIKE '%s'" \
                          % (int(idtps), idrobot, idarm, self.handpkg.getHandName())
                    resultttgs = self.gdb.execute(sql)
                    if len(resultttgs) == 0:
                        continue
                    localidedges = []
                    for ttgsrow in resultttgs:
                        ttgsid = int(ttgsrow[0])
                        ttgscct0 = dc.strToV3(ttgsrow[1])
                        ttgscct1 = dc.strToV3(ttgsrow[2])
                        ttgsrotmat = dc.strToMat4(ttgsrow[3])
                        ttgsjawwidth = float(ttgsrow[4])
                        ttgsidfreeair = int(ttgsrow[5])
                        ttgsfgrcenter = (ttgscct0 + ttgscct1) / 2
                        handa = -ttgsrotmat.getRow3(2)
                        ttgsfgrcenterhanda = ttgsfgrcenter + handa * self.rethanda
                        ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda * self.retworlda
                        ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                        ttgsfgrcenternp_handa = pg.v3ToNp(ttgsfgrcenterhanda)
                        ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                        ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                        objrotmat4 = dc.strToMat4(tpsrows[:, 4][i])
                        objrotmat4worlda = Mat4(objrotmat4)
                        objrotmat4worlda.setRow(3, objrotmat4.getRow3(3) + self.worlda * self.retworlda)
                        ttgsjnts = [self.robot.initjnts[0], np.array(dc.strToList(ttgsrow[6]))]
                        ttgsjnts_handa = [self.robot.initjnts[0], np.array(dc.strToList(ttgsrow[7]))]
                        ttgsjnts_worlda = [self.robot.initjnts[0], np.array(dc.strToList(ttgsrow[8]))]
                        self.regg.add_node(armname + str(ttgsid), fgrcenter=ttgsfgrcenternp,
                                           fgrcenterhanda=ttgsfgrcenternp_handa,
                                           fgrcenterworlda=ttgsfgrcenternp_worlda,
                                           jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                           globalgripid=ttgsidfreeair,
                                           armjnts = ttgsjnts,
                                           armjntshanda = ttgsjnts_handa,
                                           armjntsworlda = ttgsjnts_worlda,
                                           freetabletopplacementid=int(tpsrows[:, 2][i]),
                                           tabletopplacementrotmat=objrotmat4,
                                           tabletopplacementrotmathanda=objrotmat4,
                                           tabletopplacementrotmatworlda=objrotmat4worlda,
                                           angle=float(tpsrows[:, 1][i]), tabletopposition=dc.strToV3(tpsrows[:, 3][i]))
                        globalidsedges[str(ttgsidfreeair)].append(armname + str(ttgsid))
                        localidedges.append(armname + str(ttgsid))
                    # print list(itertools.combinations(ttgrows[:,0], 2))
                    for edge in list(itertools.combinations(localidedges, 2)):
                        self.regg.add_edge(*edge, weight=1, edgetype='transit')
                if len(globalidsedges) == 0:
                    raise ValueError("Plan tabletopgrips first!")

        if((transitoption is "useboth") or (transitoption is "usehandover")):
            # add floatingposes
            for fpind, objrotmat4 in enumerate(self.floatingposes.gridsfloatingposemat4s):
                for pairind, hndrotmat4pair in enumerate(self.floatingposes.floatinggrippairshndmat4s[fpind]):
                    iele = 0
                    if armname == 'lft':
                        iele = 1
                    fpgid = self.floatingposes.floatinggrippairsids[fpind][pairind][iele]
                    fpgidfreeair = self.floatingposes.floatinggrippairsidfreeairs[fpind][pairind][iele]
                    ccts = self.floatingposes.floatinggrippairscontacts[fpind][pairind][iele]
                    hndrotmat4 = hndrotmat4pair[iele]
                    fpgfgrcenter = (Vec3(ccts[0][0], ccts[0][1], ccts[0][2]) + Vec3(ccts[1][0], ccts[1][1],
                                                                                    ccts[1][2])) / 2
                    fpgfgrcenterhanda = fpgfgrcenter - hndrotmat4.getRow3(2) * self.rethanda
                    fpgfgrcenternp = pg.v3ToNp(fpgfgrcenter)
                    fpgfgrcenterhandanp = pg.v3ToNp(fpgfgrcenterhanda)
                    jawwidth = self.floatingposes.floatinggrippairsjawwidths[fpind][pairind][iele]
                    fpjnts = [self.robot.initjnts[0], np.array(self.floatingposes.floatinggrippairsjnts[fpind][pairind][iele])]
                    fpjnts_handa = [self.robot.initjnts[0], np.array(self.floatingposes.floatinggrippairsjnts_handa[fpind][pairind][iele])]
                    hndrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                    fprotmat4 = objrotmat4
                    self.regg.add_node('ho' + armname + str(fpgid), fgrcenter=fpgfgrcenternp,
                                       fgrcenterhanda=fpgfgrcenterhandanp, jawwidth=jawwidth,
                                       hndrotmat3np=hndrotmat3np,
                                       armjnts=fpjnts,
                                       armjntshanda=fpjnts_handa,
                                       floatingposerotmat4=fprotmat4,
                                       floatingposerotmat4handa=fprotmat4, floatingposeind=fpind,
                                       floatingposegrippairind=pairind, globalgripid=fpgidfreeair)
                    globalidsedges[str(fpgidfreeair)].append('ho' + armname + str(fpgid))
            for globalidedgesid in globalidsedges:
                for edge in list(itertools.combinations(globalidsedges[globalidedgesid], 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype='transfer')

    def __bridgeGraph(self):
        for fpind, objrotmat4 in enumerate(self.floatingposes.gridsfloatingposemat4s):
            for pairind, hndrotmat4pair in enumerate(self.floatingposes.floatinggrippairshndmat4s[fpind]):
                fpgid0 = self.floatingposes.floatinggrippairsids[fpind][pairind][0]
                fpgid1 = self.floatingposes.floatinggrippairsids[fpind][pairind][1]
                self.regg.add_edge('horgt'+str(fpgid0), 'holft'+str(fpgid1), weight = 1, edgetype = 'handovertransit')

    def __addendsglgrasp(self, rotmat4, cond, graspgid, toolvec3 = None):
        """
        add a start with a singel grasp to the regg

        :param rotmat4:
        :param cond: one from "startrgt", "startlft", "goallft", "goalrgt"
        :param graspmat4:
        :return:
        """

        if toolvec3 is None:
            toola = Vec3(0,0,1)
        else:
            toola = rotmat4.xformVec(toolvec3)

        # the nodeids is also for quick access
        nodeids = []
        if cond == "startrgt":
            self.startrgtnodeids = []
            nodeids = self.startrgtnodeids
        elif cond == "startlft":
            self.startlftnodeids = []
            nodeids = self.startlftnodeids
        elif cond == "goalrgt":
            self.goalrgtnodeids = []
            nodeids = self.goalrgtnodeids
        elif cond == "goallft":
            self.goallftnodeids = []
            nodeids = self.goallftnodeids
        else:
            raise Exception("Wrong conditions!")
        # the node id of a globalgripid
        nodeidofglobalid= {}
        for j, rotmat in enumerate(self.freegriprotmats):
            # vector between object and robot ee
            if self.freegripids[j] == graspgid:
                ttgsrotmat = rotmat * rotmat4
                # check if the hand collide with tabletop
                tmphnd = self.robothand
                # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
                initmat = tmphnd.getMat()
                initjawwidth = tmphnd.jawwidth
                # set jawwidth to 80 to avoid collision with surrounding obstacles
                # set to gripping with is unnecessary
                # tmprtq85.setJawwidth(self.freegripjawwidth[j])
                tmphnd.setJawwidth(50)
                tmphnd.setMat(pandanpmat4 = ttgsrotmat)
                # tmphnd.setMat(npmat4=ttgsrotmat)
                # add hand model to bulletworld
                hndbullnode = npcd.genBulletCDMeshMultiNp(tmphnd.handnp)
                result = self.bulletworld.contactTest(hndbullnode)
                tmphnd.setMat(pandanpmat4 = initmat)
                # tmphnd.setMat(npmat4=ttgsrotmat)
                tmphnd.setJawwidth(initjawwidth)
                if not result.getNumContacts():
                    ttgscct0=rotmat4.xformPoint(self.freegripcontacts[j][0])
                    ttgscct1=rotmat4.xformPoint(self.freegripcontacts[j][1])
                    ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                    handa = -ttgsrotmat.getRow3(2)
                    ttgsfgrcenterhanda = ttgsfgrcenter + handa*self.rethanda
                    ttgsfgrcenterworlda = ttgsfgrcenter + toola*self.retworlda
                    ttgsjawwidth = self.freegripjawwidth[j]
                    ttgsidfreeair = self.freegripids[j]
                    ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                    ttgsfgrcenternp_handa = pg.v3ToNp(ttgsfgrcenterhanda)
                    ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                    ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                    ikr = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np, armname = cond[-3:])
                    ikr_handa = self.robot.numikr(ttgsfgrcenternp_handa, ttgsrotmat3np, armname = cond[-3:])
                    ikr_worlda = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np, armname = cond[-3:])
                    if (ikr is not None) and (ikr_handa is not None) and (ikr_worlda is not None):
                        # note the tabletopposition here is not the contact for the intermediate states
                        # it is the zero pos
                        tabletopposition = rotmat4.getRow3(3)
                        startrotmat4worlda = Mat4(rotmat4)
                        startrotmat4worlda.setRow(3, rotmat4.getRow3(3)+toola*self.retworlda)
                        startrotmat4worldaworldz = Mat4(startrotmat4worlda)
                        startrotmat4worldaworldz.setRow(3, startrotmat4worlda.getRow3(3)+self.worlda*self.retworlda)
                        self.regg.add_node(cond+str(j), fgrcenter=ttgsfgrcenternp,
                                           fgrcenterhanda = ttgsfgrcenternp_handa,
                                           fgrcenterworlda = ttgsfgrcenternp_worlda,
                                           jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                           armjnts = ikr,
                                           armjntshanda = ikr_handa,
                                           armjntsworlda = ikr_worlda,
                                           globalgripid = ttgsidfreeair,
                                           freetabletopplacementid = 'na',
                                           tabletopplacementrotmat = rotmat4,
                                           tabletopplacementrotmathanda = rotmat4,
                                           tabletopplacementrotmathandaworldz = 'na',
                                           tabletopplacementrotmatworlda = startrotmat4worlda,
                                           tabletopplacementrotmatworldaworldz = startrotmat4worldaworldz,
                                           angle = 'na', tabletopposition = tabletopposition)
                        nodeidofglobalid[ttgsidfreeair]=cond+str(j)
                        nodeids.append(cond+str(j))
                        # tmprtq85.reparentTo(base.render)
                    break

        if len(self.startrgtnodeids) == 0:
            print("No available starting grip for " + cond[-3:] + " hand!")

        # add edge
        # add transit edge
        for edge in list(itertools.combinations(nodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = cond+'transit')
        # add transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if reggnode.startswith(cond[-3:]) or reggnode.startswith('ho'+cond[-3:]):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalid.keys():
                    nodeid = nodeidofglobalid[globalgripid]
                    self.regg.add_edge(nodeid, reggnode, weight=1, edgetype = cond+'transfer')

    def __addend(self, rotmat4, cond="startrgt", ctvec = Vec3(0,0,1), ctangle = 60.0, toolvec3 = None):
        """
        add a start or a goal for the regg

        :param rotmat4:
        :param cond: the specification of the rotmat4: "startrgt", "startlft", "goalrgt", "goallft"
        :param ctvec, ctangle: the conditions of filtering, the candidate hand z must have a smaller angle with vec
        :param toolvec: the direction to move the tool in the last step, it is described in the local coordinate system of the object
        :return:

        author: weiwei
        date: 20180925
        """

        if toolvec3 is None:
            toola = Vec3(0,0,1)
        else:
            toola = rotmat4.xformVec(toolvec3)

        # the nodeids is also for quick access
        nodeids = []
        if cond == "startrgt":
            self.startrgtnodeids = []
            nodeids = self.startrgtnodeids
        elif cond == "startlft":
            self.startlftnodeids = []
            nodeids = self.startlftnodeids
        elif cond == "goalrgt":
            self.goalrgtnodeids = []
            nodeids = self.goalrgtnodeids
        elif cond == "goallft":
            self.goallftnodeids = []
            nodeids = self.goallftnodeids
        else:
            raise Exception("Wrong conditions!")
        # the node id of a globalgripid
        nodeidofglobalid= {}
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * rotmat4
            # filtering
            handa = -Vec3(ttgsrotmat.getRow3(2))
            # vector between object and robot ee
            if handa.angleDeg(ctvec) > ctangle:
                continue
            # check if the hand collide with obstacles
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            tmphnd = self.robothand
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            # tmphnd.setJawwidth(80)
            tmphnd.setJawwidth(50)
            tmphnd.setMat(pandanpmat4 = ttgsrotmat)
            # tmphnd.setMat(npmat4=ttgsrotmat)
            hndbullnode = npcd.genBulletCDMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            tmphnd.setMat(pandanpmat4 = initmat)
            # tmphnd.setMat(npmat4=initmat)
            tmphnd.setJawwidth(initjawwidth)
            if not result.getNumContacts():
                ttgscct0=rotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=rotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                handa = -ttgsrotmat.getRow3(2)
                ttgsfgrcenterhanda = ttgsfgrcenter + handa*self.rethanda
                ttgsfgrcenterworlda = ttgsfgrcenter + toola*self.retworlda
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripids[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handa = pg.v3ToNp(ttgsfgrcenterhanda)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikr = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np, armname = cond[-3:])
                ikr_handa = self.robot.numikr(ttgsfgrcenternp_handa, ttgsrotmat3np, armname = cond[-3:])
                ikr_worlda = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np, armname = cond[-3:])
                # ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np, armname = 'rgt')
                if (ikr is not None) and (ikr_handa is not None) and (ikr_worlda is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = rotmat4.getRow3(3)
                    startrotmat4worlda = Mat4(rotmat4)
                    startrotmat4worlda.setRow(3, rotmat4.getRow3(3)+toola*self.retworlda)
                    startrotmat4worldaworldz = Mat4(startrotmat4worlda)
                    startrotmat4worldaworldz.setRow(3, startrotmat4worlda.getRow3(3)+self.worlda*self.retworlda)
                    self.regg.add_node(cond+str(j), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhanda = ttgsfgrcenternp_handa,
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       armjnts = ikr,
                                       armjntshanda = ikr_handa,
                                       armjntsworlda = ikr_worlda,
                                       globalgripid = ttgsidfreeair,
                                       freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = rotmat4,
                                       tabletopplacementrotmathanda = rotmat4,
                                       tabletopplacementrotmathandaworldz = 'na',
                                       tabletopplacementrotmatworlda = startrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = startrotmat4worldaworldz,
                                       angle = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalid[ttgsidfreeair]=cond+str(j)
                    nodeids.append(cond+str(j))
                    # tmprtq85.reparentTo(base.render)

        if len(nodeids) == 0:
            print("No available " + cond[:-3] + " grip for " + cond[-3:] + " hand!")

        # add edge
        # add transit edge
        for edge in list(itertools.combinations(nodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = cond+'transit')
        # add transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if reggnode.startswith(cond[-3:]) or reggnode.startswith('ho'+cond[-3:]):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalid.keys():
                    nodeid = nodeidofglobalid[globalgripid]
                    self.regg.add_edge(nodeid, reggnode, weight=1, edgetype = cond+'transfer')

    def __addstartgoal(self, startrotmat4, goalrotmat4, starttoolvec3 = None, goaltoolvec3 = None):
        """
        TODO: for compacity purpose
        add start and goal for the regg

        :param startrotmat4 and goalrotmat4: both are 4by4 panda3d matrix
        :param starttoolvec3, goaltoolvec3 determine the retract for the start and goal, or the
        direction to move the tool in the first and last steps. The vector is described in the local coordinate system of the object
        :return:

        author: weiwei
        date: 20161216, sapporo
        """

        self.startrotmat4 = startrotmat4
        self.goalrotmat4 = goalrotmat4

        ### start rgt
        self.__addend(startrotmat4, cond="startrgt", toolvec3=starttoolvec3)
        self.__addend(startrotmat4, cond="startlft", toolvec3=starttoolvec3)
        self.__addend(goalrotmat4, cond="goalrgt", toolvec3=goaltoolvec3)
        self.__addend(goalrotmat4, cond="goallft", toolvec3=goaltoolvec3)

        # add start to goal direct edges rgt-rgt
        for startnodeid in self.startrgtnodeids:
            for goalnodeid in self.goalrgtnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype = 'startgoalrgttransfer')

        # add start to goal direct edges lft-lft
        for startnodeid in self.startlftnodeids:
            for goalnodeid in self.goallftnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype = 'startgoallfttransfer')

    def addStartGoal(self, startrotmat4, goalrotmat4, choice, startgraspgid = None,
                     goalgraspgid = None, starttoolvec = None, goaltoolvec = None):
        """
        add start and goal to the grasph
        if start/goalgrasppose is not None, the only pose will be used
        the pose is defined by a numpy 4x4 homomatrix

        :param startrotmat4:
        :param goalrotmat4:
        :param choice in "startrgtgoallft" "startrgtgoalrgt" "startlftgoalrgt" "startrgtgoallft"
        :param startgraspgid:
        :param goalgraspgid:
        :param starttoolvec
        :param goaltoolvec there are three choices for the tool vecs: None indicates global z, [0,0,0] indicates no tool vec
        :return:

        author: weiwei
        date: 20180925
        """

        if starttoolvec is not None:
            starttoolVec3 = Vec3(starttoolvec[0], starttoolvec[1], starttoolvec[2])
        else:
            starttoolVec3 = None
        if goaltoolvec is not None:
            goaltoolVec3 = Vec3(goaltoolvec[0], goaltoolvec[1], goaltoolvec[2])
        else:
            goaltoolVec3 = None

        self.startrotmat4 = startrotmat4
        self.goalrotmat4 = goalrotmat4

        self.choice = choice

        startchoice = choice[:8]
        goalchoice = choice[8:]
        if startgraspgid is None:
            print("startgraspgid is None, all grasps are candidates")
            self.__addend(startrotmat4, cond = startchoice, toolvec3=starttoolVec3)
        else:
            print("startgraspgid is ", startgraspgid, " this exact grasp is the candidate")
            self.__addendsglgrasp(startrotmat4, cond = startchoice, graspgid = startgraspgid, toolvec3=starttoolVec3)

        if goalgraspgid is None:
            print("goalgraspgid is None, all grasps are candidates")
            self.__addend(goalrotmat4, goalchoice, toolvec3=goaltoolVec3)
        else:
            print("goalgraspgid is ", goalgraspgid, " this exact grasp is the candidate")
            self.__addendsglgrasp(goalrotmat4, goalchoice, graspgid = goalgraspgid, toolvec3=goaltoolVec3)

        # add start to goal direct edges rgt-rgt
        for startnodeid in self.startrgtnodeids:
            for goalnodeid in self.goalrgtnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype='startgoalrgttransfer')

        # add start to goal direct edges lft-lft
        for startnodeid in self.startlftnodeids:
            for goalnodeid in self.goallftnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype='startgoallfttransfer')

    def updateshortestpath(self):
        """
        this function is assumed to be called after start and goal are set

        :return:
        """

        # startrgt goalrgt
        if len(self.startrgtnodeids) > 0 and len(self.goalrgtnodeids) > 0:
            print("Number of start grasps: ", len(self.startrgtnodeids), "; Number of goal grasps: ", len(self.goalrgtnodeids))
            startgrip = self.startrgtnodeids[0]
            goalgrip = self.goalrgtnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source=startgrip, target=goalgrip)
            self.directshortestpaths_startrgtgoalrgt = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path) - 1:
                            continue
                        else:
                            self.directshortestpaths_startrgtgoalrgt.append(path[i - 1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startrgtgoalrgt[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startrgtgoalrgt[-1] = self.directshortestpaths_startrgtgoalrgt[-1][
                                                                           :i + 1]
                            break
            except:
                raise Exception('No startrgtgoalrgt')

        # startrgt goallft
        if len(self.startrgtnodeids) > 0 and len(self.goallftnodeids) > 0:
            print("Number of start grasps: ", len(self.startrgtnodeids), "; Number of goal grasps: ", len(self.goallftnodeids))
            startgrip = self.startrgtnodeids[0]
            goalgrip = self.goallftnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source=startgrip, target=goalgrip)
            self.directshortestpaths_startrgtgoallft = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path) - 1:
                            continue
                        else:
                            self.directshortestpaths_startrgtgoallft.append(path[i - 1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startrgtgoallft[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startrgtgoallft[-1] = self.directshortestpaths_startrgtgoallft[-1][
                                                                           :i + 1]
                            break
            except:
                raise Exception('No startrgtgoallft')

        # startlft goalrgt
        if len(self.startlftnodeids) > 0 and len(self.goalrgtnodeids) > 0:
            print("Number of start grasps: ", len(self.startlftnodeids), "; Number of goal grasps: ", len(self.goalrgtnodeids))
            startgrip = self.startlftnodeids[0]
            goalgrip = self.goalrgtnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source=startgrip, target=goalgrip)
            self.directshortestpaths_startlftgoalrgt = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path) - 1:
                            continue
                        else:
                            self.directshortestpaths_startlftgoalrgt.append(path[i - 1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startlftgoalrgt[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startlftgoalrgt[-1] = self.directshortestpaths_startlftgoalrgt[-1][
                                                                           :i + 1]
                            break
            except:
                raise Exception('No startlftgoalrgt')

        # startlft goallft
        if len(self.startlftnodeids) > 0 and len(self.goallftnodeids) > 0:
            print("Number of start grasps: ", len(self.startlftnodeids), "; Number of goal grasps: ", len(self.goallftnodeids))
            startgrip = self.startlftnodeids[0]
            goalgrip = self.goallftnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source=startgrip, target=goalgrip)
            self.directshortestpaths_startlftgoallft = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path) - 1:
                            continue
                        else:
                            self.directshortestpaths_startlftgoallft.append(path[i - 1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startlftgoallft[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startlftgoallft[-1] = self.directshortestpaths_startlftgoallft[-1][
                                                                           :i + 1]
                            break
            except:
                raise Exception('No startlftgoallft')

    def findshortestpath(self, startrotmat4, goalrotmat4, bagain = False):
        """
        TODO: deprecated, for compacity purpose

        :param startrotmat4:
        :param goalrotmat4:
        :param bagain:
        :return:

        author: weiwei
        """

        self.shortestpaths = None
        self.directshortestpaths_startrgtgoalrgt = []
        self.directshortestpaths_startrgtgoallft = []
        self.directshortestpaths_startlftgoalrgt = []
        self.directshortestpaths_startlftgoallft = []

        if bagain == False:
            self.__addstartgoal(startrotmat4, goalrotmat4)

        # startrgt goalrgt
        if len(self.startrgtnodeids) > 0 and len(self.goalrgtnodeids) > 0:
            startgrip = self.startrgtnodeids[0]
            goalgrip = self.goalrgtnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startrgtgoalrgt = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startrgtgoalrgt.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startrgtgoalrgt[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startrgtgoalrgt[-1]=self.directshortestpaths_startrgtgoalrgt[-1][:i+1]
                            break
            except:
                assert('No startrgt goalrgt path')

        # startrgt goallft
        if len(self.startrgtnodeids) > 0 and len(self.goallftnodeids) > 0:
            startgrip = self.startrgtnodeids[0]
            goalgrip = self.goallftnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startrgtgoallft = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startrgtgoallft.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startrgtgoallft[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startrgtgoallft[-1]=self.directshortestpaths_startrgtgoallft[-1][:i+1]
                            break
            except:
                assert('No startrgt goallft path')

        # startlft goalrgt
        if len(self.startlftnodeids) > 0 and len(self.goalrgtnodeids) > 0:
            startgrip = self.startlftnodeids[0]
            goalgrip = self.goalrgtnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startlftgoalrgt = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startlftgoalrgt.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startlftgoalrgt[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startlftgoalrgt[-1]=self.directshortestpaths_startlftgoalrgt[-1][:i+1]
                            break
            except:
                assert('No startlft goalrgt path')

        # startlft goallft
        if len(self.startlftnodeids) > 0 and len(self.goallftnodeids) > 0:
            startgrip = self.startlftnodeids[0]
            goalgrip = self.goallftnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startlftgoallft = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startlftgoallft.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startlftgoallft[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startlftgoallft[-1]=self.directshortestpaths_startlftgoallft[-1][:i+1]
                            break
            except:
                assert('No startlft goallft path')

    def removeBadNodes(self, nodelist):
        """
        remove the invalidated nodes to prepare for a new plan

        :param nodelist: a list of invalidated nodes
        :return:

        author: weiwei
        date: 20170920
        """

        print("Removing nodes ", nodelist)
        self.regg.remove_nodes_from(nodelist)
        for node in nodelist:
            if node.startswith('startrgt'):
                try:
                    self.startrgtnodeids.remove(node)
                except KeyError:
                    pass
            if node.startswith('startlft'):
                try:
                    self.startlftnodeids.remove(node)
                except KeyError:
                    pass
            if node.startswith('goalrgt'):
                try:
                    self.goalrgtnodeids.remove(node)
                except KeyError:
                    pass
            if node.startswith('goallft'):
                try:
                    self.goallftnodeids.remove(node)
                except KeyError:
                    pass

    def removeBadEdge(self, node0, node1):
        """
        remove an invalidated edge to prepare for a new plan

        :param node0, node1 two ends of an edge
        :return:

        author: weiwei
        date: 20190423
        """

        print("Removing edge ", node0, node1)
        self.regg.remove_edge(node0, node1)

    def plotgraph(self, pltfig):
        """
        plot the graph without start and goal

        :param pltfig: the matplotlib object
        :return:

        author: weiwei
        date: 20161217, sapporos
        """

        # biggest circle: grips; big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = {}
        xydiscreterotspos = {}
        self.xyzglobalgrippos = {}
        for i, ttpsid in enumerate(self.fttpsids):
            xydiscreterotspos[ttpsid]={}
            self.xyzglobalgrippos[ttpsid]={}
            xypos = [radiusplacement*math.cos(2*math.pi/self.nfttps*i),
                     radiusplacement*math.sin(2*math.pi/self.nfttps*i)]
            xyplacementspos[ttpsid] = xypos
            for j, anglevalue in enumerate(self.angles):
                self.xyzglobalgrippos[ttpsid][anglevalue]={}
                xypos = [radiusrot*math.cos(math.radians(anglevalue)), radiusrot*math.sin(math.radians(anglevalue))]
                xydiscreterotspos[ttpsid][anglevalue] = \
                    [xyplacementspos[ttpsid][0]+xypos[0], xyplacementspos[ttpsid][1]+xypos[1]]
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip*math.cos(2*math.pi/len(self.globalgripids)* k),
                             radiusgrip*math.sin(2*math.pi/len(self.globalgripids)*k)]
                    self.xyzglobalgrippos[ttpsid][anglevalue][globalgripid]=\
                        [xydiscreterotspos[ttpsid][anglevalue][0]+xypos[0],
                         xydiscreterotspos[ttpsid][anglevalue][1]+xypos[1], 0]

        # for start and goal grasps poses:
        self.xyzglobalgrippos_startgoal={}
        for k, globalgripid in enumerate(self.globalgripids):
            xypos = [radiusgrip * math.cos(2 * math.pi / len(self.globalgripids) * k),
                     radiusgrip * math.sin(2 * math.pi / len(self.globalgripids) * k)]
            self.xyzglobalgrippos_startgoal[globalgripid] = [xypos[0],xypos[1],0]

        # for handover
        nfp = len(self.floatingposes.gridsfloatingposemat4s)
        xdist = 10
        x = range(300,501,xdist)
        y = range(-50,50,100*xdist/nfp)

        transitedges = []
        transferedges = []
        hotransitedges = []
        hotransferedges = []
        startrgttransferedges = []
        startlfttransferedges = []
        goalrgttransferedges = []
        goallfttransferedges = []
        startgoalrgttransferedges = []
        startgoallfttransferedges = []
        startrgttransitedges = []
        goalrgttransitedges = []
        startlfttransitedges = []
        goallfttransitedges = []
        counter = 0
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            counter = counter + 1
            if counter > 100000:
                break
            xyzpos0 = [0,0,0]
            xyzpos1 = [0,0,0]
            if (reggedgedata['edgetype'] is 'transit') or (reggedgedata['edgetype'] is 'transfer'):
                if nid0.startswith('ho'):
                    fpind0 = self.regg.node[nid0]['floatingposeind']
                    fpgpind0 = self.regg.node[nid0]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind0])
                    xpos = x[fpind0 % len(x)]
                    ypos = y[fpind0/len(x)]
                    xyzpos0 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind0)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind0)+ypos, 0]
                    if nid0.startswith('horgt'):
                        xyzpos0[1] = xyzpos0[1]-100
                    if nid0.startswith('holft'):
                        xyzpos0[1] = xyzpos0[1]+100
                else:
                    fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                    anglevalue0 = self.regg.node[nid0]['angle']
                    ggid0 = self.regg.node[nid0]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][ggid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                    if nid0.startswith('rgt'):
                        xyzpos0[1] = xyzpos0[1]-800
                    if nid0.startswith('lft'):
                        xyzpos0[1] = xyzpos0[1]+800
                if nid1.startswith('ho'):
                    fpind1 = self.regg.node[nid1]['floatingposeind']
                    fpgpind1 = self.regg.node[nid1]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind1])
                    xpos = x[fpind1 % len(x)]
                    ypos = y[fpind1/len(x)]
                    xyzpos1 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind1)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind1)+ypos, 0]
                    if nid1.startswith('horgt'):
                        xyzpos1[1] = xyzpos1[1]-100
                    if nid1.startswith('holft'):
                        xyzpos1[1] = xyzpos1[1]+100
                else:
                    fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                    anglevalue1 = self.regg.node[nid1]['angle']
                    ggid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][ggid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                    if nid1.startswith('rgt'):
                        xyzpos1[1] = xyzpos1[1]-800
                    if nid1.startswith('lft'):
                        xyzpos1[1] = xyzpos1[1]+800
                # 3d
                # if reggedgedata['edgetype'] is 'transit':
                #     transitedges.append([xyzpos0, xyzpos1])
                # if reggedgedata['edgetype'] is 'transfer':
                #     transferedges.append([xyzpos0, xyzpos1])
                #2d
                # move the basic graph to x+600
                xyzpos0[0] = xyzpos0[0]+600
                xyzpos1[0] = xyzpos1[0]+600
                if reggedgedata['edgetype'] is 'transit':
                    transitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'transfer':
                    if nid0.startswith('ho') or nid1.startswith('ho'):
                        hotransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                    else:
                        transferedges.append([xyzpos0[:2], xyzpos1[:2]])
            elif (reggedgedata['edgetype'] is 'handovertransit'):
                fpind0 = self.regg.node[nid0]['floatingposeind']
                fpgpind0 = self.regg.node[nid0]['floatingposegrippairind']
                nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind0])
                xpos = x[fpind0 % len(x)]
                ypos = y[fpind0/len(x)]
                xyzpos0 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind0)+xpos,
                         radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind0)+ypos, 0]
                if nid0.startswith('horgt'):
                    xyzpos0[1] = xyzpos0[1]-100
                if nid0.startswith('holft'):
                    xyzpos0[1] = xyzpos0[1]+100
                fpind1 = self.regg.node[nid1]['floatingposeind']
                fpgpind1 = self.regg.node[nid1]['floatingposegrippairind']
                nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind1])
                xpos = x[fpind1 % len(x)]
                ypos = y[fpind1/len(x)]
                xyzpos1 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind1)+xpos,
                         radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind1)+ypos, 0]
                if nid1.startswith('horgt'):
                    xyzpos1[1] = xyzpos1[1]-100
                if nid1.startswith('holft'):
                    xyzpos1[1] = xyzpos1[1]+100
                # move the basic graph to x+600
                xyzpos0[0] = xyzpos0[0]+600
                xyzpos1[0] = xyzpos1[0]+600
                hotransitedges.append([xyzpos0[:2], xyzpos1[:2]])
            elif reggedgedata['edgetype'].endswith('transit'):
                gid0 = self.regg.node[nid0]['globalgripid']
                gid1 = self.regg.node[nid1]['globalgripid']
                tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                xyzpos0 = map(add, self.xyzglobalgrippos_startgoal[gid0],
                              [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                xyzpos1 = map(add, self.xyzglobalgrippos_startgoal[gid1],
                              [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                if reggedgedata['edgetype'] is 'startrgttransit':
                    startrgttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'goalrgttransit':
                    goalrgttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'startlfttransit':
                    startlfttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'goallfttransit':
                    goallfttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
            elif reggedgedata['edgetype'].endswith('transfer'):
                if nid0.startswith('ho'):
                    fpind0 = self.regg.node[nid0]['floatingposeind']
                    fpgpind0 = self.regg.node[nid0]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind0])
                    xpos = x[fpind0 % len(x)]
                    ypos = y[fpind0/len(x)]
                    xyzpos0 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind0)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind0)+ypos, 0]
                    if nid0.startswith('horgt'):
                        xyzpos0[1] = xyzpos0[1]-100
                    if nid0.startswith('holft'):
                        xyzpos0[1] = xyzpos0[1]+100
                    xyzpos0[0] = xyzpos0[0]+600
                elif nid0.startswith('rgt') or nid0.startswith('lft'):
                    fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                    anglevalue0 = self.regg.node[nid0]['angle']
                    ggid0 = self.regg.node[nid0]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][ggid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                    if nid0.startswith('rgt'):
                        xyzpos0[1] = xyzpos0[1]-800
                    if nid0.startswith('lft'):
                        xyzpos0[1] = xyzpos0[1]+800
                    xyzpos0[0] = xyzpos0[0]+600
                else:
                    gid0 = self.regg.node[nid0]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    xyzpos0 = map(add, self.xyzglobalgrippos_startgoal[gid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                if nid1.startswith('ho'):
                    fpind1 = self.regg.node[nid1]['floatingposeind']
                    fpgpind1 = self.regg.node[nid1]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind1])
                    xpos = x[fpind1 % len(x)]
                    ypos = y[fpind1/len(x)]
                    xyzpos1 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind1)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind1)+ypos, 0]
                    if nid1.startswith('horgt'):
                        xyzpos1[1] = xyzpos1[1]-100
                    if nid1.startswith('holft'):
                        xyzpos1[1] = xyzpos1[1]+100
                    xyzpos1[0] = xyzpos1[0]+600
                elif nid1.startswith('lft') or nid1.startswith('rgt'):
                    fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                    anglevalue1 = self.regg.node[nid1]['angle']
                    ggid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][ggid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                    if nid1.startswith('rgt'):
                        xyzpos1[1] = xyzpos1[1]-800
                    if nid1.startswith('lft'):
                        xyzpos1[1] = xyzpos1[1]+800
                    xyzpos1[0] = xyzpos1[0]+600
                else:
                    ggid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos1 = map(add, self.xyzglobalgrippos_startgoal[ggid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                if reggedgedata['edgetype'].startswith('startgoalrgt'):
                    startgoalrgttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('startgoallft'):
                    startgoallfttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('startrgt'):
                    startrgttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('startlft'):
                    startlfttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('goalrgt'):
                    goalrgttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('goallft'):
                    goallfttransferedges.append([xyzpos0[:2], xyzpos1[:2]])

            self.gnodesplotpos[nid0] = xyzpos0[:2]
            self.gnodesplotpos[nid1] = xyzpos1[:2]
        #3d
        # transitec = mc3d.Line3DCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        # transferec = mc3d.Line3DCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        #2d
        transitec = mc.LineCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        hotransitec = mc.LineCollection(hotransitedges, colors=[1,0,1,.1], linewidths=1)
        hotransferec = mc.LineCollection(hotransferedges, colors=[.5,.5,0,.03], linewidths=1)
        # transfer
        startrgttransferec = mc.LineCollection(startrgttransferedges, colors=[.7,0,0,.3], linewidths=1)
        startlfttransferec = mc.LineCollection(startlfttransferedges, colors=[.3,0,0,.3], linewidths=1)
        goalrgttransferec = mc.LineCollection(goalrgttransferedges, colors=[0,0,.7,.3], linewidths=1)
        goallfttransferec = mc.LineCollection(goallfttransferedges, colors=[0,0,.3,.3], linewidths=1)
        startgoalrgttransferec = mc.LineCollection(startgoalrgttransferedges, colors=[0,0,.7,.3], linewidths=1)
        startgoallfttransferec = mc.LineCollection(startgoallfttransferedges, colors=[0,0,.3,.3], linewidths=1)
        # transit
        startrgttransitec = mc.LineCollection(startrgttransitedges, colors=[0,.5,1,.3], linewidths=1)
        startlfttransitec = mc.LineCollection(startlfttransitedges, colors=[0,.2,.4,.3], linewidths=1)
        goalrgttransitec = mc.LineCollection(goalrgttransitedges, colors=[0,.5,1,.3], linewidths=1)
        goallfttransitec = mc.LineCollection(goallfttransitedges, colors=[0,.2,.4,.3], linewidths=1)

        ax = pltfig.add_subplot(111)
        ax.add_collection(transferec)
        ax.add_collection(transitec)
        ax.add_collection(hotransferec)
        ax.add_collection(hotransitec)
        ax.add_collection(startrgttransferec)
        ax.add_collection(startlfttransferec)
        ax.add_collection(goalrgttransferec)
        ax.add_collection(goallfttransferec)
        ax.add_collection(startgoalrgttransferec)
        ax.add_collection(startgoallfttransferec)
        # ax.add_collection(startrgttransitec)
        # ax.add_collection(startlfttransitec)
        # ax.add_collection(goalrgttransitec)
        # ax.add_collection(goallfttransitec)

        # for reggnode, reggnodedata in self.regg.nodes(data=True):
        #     placementid =  reggnodedata['placementid']
        #     angleid = reggnodedata['angleid']
        #     globalgripid = reggnodedata['globalgripid']
        #    tabletopposition = reggnodedata['tabletopposition']
        #     xyzpos = map(add, xyzglobalgrippos[placementid][angleid][str(globalgripid)],[tabletopposition[0], tabletopposition[1], tabletopposition[2]])
        #     plt.plot(xyzpos[0], xyzpos[1], 'ro')

    def plotshortestpath(self, pltfig, id = 0, choice = 'startrgtgoalrgt'):
        """
        plot the shortest path

        about transit and transfer:
        The tabletoppositions of start and goal are the local zero of the mesh model
        in contrast, the tabletoppositions of the other nodes in the graph are the local zero of the supporting facet
        if tabletopposition start == tabletop position goal
        there are two possibilities:
        1) start and goal are the same, then it is transit
        2) start and goal are different, then it is tranfer
        Note that start and the second will never be the same since they are in different coordinate systems.
        It is reasonable since the shortest path will never let the start go to the same position again.
        if the second item is not the goal, the path between the first and second items is
        sure to be a transfer path

        :param id: which path to plot
        :param choice: startrgtgoalrgt/startrgtgoallft/startlftgoalrgt/startlftgoallft
        :return:

        author: weiwei
        date: 20170302
        """

        directshortestpaths = []
        if choice is 'startrgtgoalrgt':
            directshortestpaths = self.directshortestpaths_startrgtgoalrgt
        elif choice is 'startrgtgoallft':
            directshortestpaths = self.directshortestpaths_startrgtgoallft
        elif choice is 'startlftgoalrgt':
            directshortestpaths = self.directshortestpaths_startlftgoalrgt
        elif choice is 'startlftgoallft':
            directshortestpaths = self.directshortestpaths_startlftgoallft
        for i,path in enumerate(directshortestpaths):
            if i == id:
                pathedgestransit = []
                pathedgestransfer = []
                pathlength = len(path)
                for pnidx in range(pathlength-1):
                    nid0 = path[pnidx]
                    nid1 = path[pnidx+1]
                    if self.regg[nid0][nid1]['edgetype'].endswith('transit'):
                        pathedgestransit.append([self.gnodesplotpos[nid0], self.gnodesplotpos[nid1]])
                    if self.regg[nid0][nid1]['edgetype'].endswith('transfer'):
                        pathedgestransfer.append([self.gnodesplotpos[nid0], self.gnodesplotpos[nid1]])
                pathtransitec = mc.LineCollection(pathedgestransit, colors=[.5, 1, 0, 1], linewidths=5)
                pathtransferec = mc.LineCollection(pathedgestransfer, colors=[0, 1, 0, 1], linewidths=5)

                ax = pltfig.gca()
                ax.add_collection(pathtransitec)
                ax.add_collection(pathtransferec)

    def plotgraphp3d(self):
        """
        draw the graph in panda3d

        :param base:
        :return:

        author: weiwei
        date: 20161216, osaka itami airport
        """

        # big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = []
        xydiscreterotspos = []
        xyzglobalgrippos = []
        for i in range(self.nplacements):
            xydiscreterotspos.append([])
            xyzglobalgrippos.append([])
            xypos = [radiusplacement*math.cos(2*math.pi/self.nplacements*i), radiusplacement*math.sin(2*math.pi/self.nplacements*i)]
            xyplacementspos.append(xypos)
            for j in range(self.ndiscreterot):
                xyzglobalgrippos[-1].append({})
                xypos = [radiusrot*math.cos(2*math.pi/self.ndiscreterot* j), radiusrot*math.sin(2*math.pi/self.ndiscreterot * j)]
                xydiscreterotspos[-1].append([xyplacementspos[-1][0]+xypos[0],xyplacementspos[-1][1]+xypos[1]])
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip*math.cos(2*math.pi/len(self.globalgripids)* k), radiusgrip*math.sin(2*math.pi/len(self.globalgripids)*k)]
                    xyzglobalgrippos[-1][-1][globalgripid]=[xydiscreterotspos[-1][-1][0]+xypos[0],xydiscreterotspos[-1][-1][1]+xypos[1], 0]

        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
            anglevalue0 = self.regg.node[nid0]['angle']
            gid0 = self.regg.node[nid0]['globalgripid']
            fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
            angelvalue1 = self.regg.node[nid1]['angle']
            gid1 = self.regg.node[nid1]['globalgripid']
            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
            xyzpos0 = map(add, xyzglobalgrippos[fttpid0][anglevalue0][str(gid0)],
                          [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
            xyzpos1 = map(add, xyzglobalgrippos[fttpid1][angelvalue1][str(gid1)],
                          [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
            # 3d
            if reggedgedata['edgetype'] is 'transit':
                transitedges.append([xyzpos0, xyzpos1])
            if reggedgedata['edgetype'] is 'transfer':
                transferedges.append([xyzpos0, xyzpos1])
        #3d
        transitecnp = pg.makelsnodepath(transitedges, rgbacolor=[0,1,1,1])
        transferecnp = pg.makelsnodepath(transferedges, rgbacolor=[0,0,0,.1])

        transitecnp.reparentTo(self.base.render)
        transferecnp.reparentTo(self.base.render)