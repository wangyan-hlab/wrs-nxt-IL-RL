#!/usr/bin/python

import os
import itertools

import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import trimesh
from pandaplotutils import pandageom as pg
from utiltools import collisiondetection as cd
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from operator import add
from database import dbaccess as db, dbcvt as dc

import networkx as nx
import math
from manipulation.assembly.asstwoobj import TwoObjAss as Toass

class GraphTpp(object):

    def __init__(self, objpath, robot, handpkg, gdb, armname):
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        self.regg = nx.Graph()

        self.gdb = gdb
        self.robot = robot
        self.handpkg = handpkg

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        self.globalgripids = []
        self.fttpsids = []
        self.nfttps = 0
        self.gnodesplotpos = {}

        self.gdb = gdb
        self.robot = robot
        self.handpkg = handpkg

        self.__loadGripsToBuildGraph(armname)

    def __loadGripsToBuildGraph(self, armname = "rgt"):
        """
        load tabletopgrips
        retraction distance are also loaded from database

        :param robot: an robot defined in robotsim.hrp5 or robotsim.nextage
        :param gdb: an object of the database.GraspDB class
        :param idarm: value = 1 "lft" or 2 "rgt", which arm to use
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
        sql = "SELECT idfreeairgrip FROM freeairgrip,object WHERE freeairgrip.idobject=object.idobject AND \
                object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) == 0:
            raise ValueError("Plan freeairgrip first!")
        for ggid in result:
            globalidsedges[str(ggid[0])] = []
            self.globalgripids.append(ggid[0])
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
            self.angles = list(set(map(float, tpsrows[:,1])))
            # for plotting
            self.fttpsids = list(set(map(int, tpsrows[:,2])))
            self.nfttps = len(self.fttpsids)

            idrobot = self.gdb.loadIdRobot(self.robot)
            for i, idtps in enumerate(tpsrows[:,0]):
                sql = "SELECT tabletopgrips.idtabletopgrips, tabletopgrips.contactpnt0, tabletopgrips.contactpnt1, \
                       tabletopgrips.rotmat, tabletopgrips.jawwidth, tabletopgrips.idfreeairgrip \
                       FROM tabletopgrips,ik,freeairgrip,hand WHERE tabletopgrips.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                       freeairgrip.idhand = hand.idhand AND\
                       tabletopgrips.idtabletopgrips=ik.idtabletopgrips AND \
                       tabletopgrips.idtabletopplacements = %d AND ik.idrobot=%d AND \
                       ik.feasibility='True' AND ik.feasibility_handx='True' AND ik.feasibility_handxworldz='True' \
                       AND ik.feasibility_worlda='True' AND ik.feasibility_worldaworldz='True' AND ik.idarm = %d \
                       AND hand.name LIKE '%s'" \
                      % (int(idtps), idrobot, idarm, self.handpkg.getHandName())
                resultttgs = self.gdb.execute(sql)
                if len(resultttgs)==0:
                    continue
                localidedges = []
                for ttgsrow in resultttgs:
                    ttgsid = int(ttgsrow[0])
                    ttgscct0 = dc.strToV3(ttgsrow[1])
                    ttgscct1 = dc.strToV3(ttgsrow[2])
                    ttgsrotmat = dc.strToMat4(ttgsrow[3])
                    ttgsjawwidth = float(ttgsrow[4])
                    ttgsidfreeair = int(ttgsrow[5])
                    ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                    handx = ttgsrotmat.getRow3(0)
                    ttgsfgrcenterhandx = ttgsfgrcenter + handx*self.rethandx
                    ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                    ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda*self.retworlda
                    ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda+ self.worldz*self.retworldz
                    ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                    ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                    ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                    ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                    ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                    ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                    objrotmat4 = dc.strToMat4(tpsrows[:,4][i])
                    objrotmat4worlda = Mat4(objrotmat4)
                    objrotmat4worlda.setRow(3, objrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    objrotmat4worldaworldz = Mat4(objrotmat4worlda)
                    objrotmat4worldaworldz.setRow(3, objrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node(armname+str(ttgsid), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhandx = ttgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = ttgsfgrcenternp_handxworldz,
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = ttgsfgrcenternp_worldaworldz,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       globalgripid = ttgsidfreeair, freetabletopplacementid = int(tpsrows[:,2][i]),
                                       tabletopplacementrotmat = objrotmat4,
                                       tabletopplacementrotmathandx = objrotmat4,
                                       tabletopplacementrotmathandxworldz = objrotmat4,
                                       tabletopplacementrotmatworlda = objrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = objrotmat4worldaworldz,
                                       angle = float(tpsrows[:,1][i]), tabletopposition = dc.strToV3(tpsrows[:,3][i]))
                    globalidsedges[str(ttgsidfreeair)].append(armname+str(ttgsid))
                    localidedges.append(armname+str(ttgsid))
                # print list(itertools.combinations(ttgrows[:,0], 2))
                for edge in list(itertools.combinations(localidedges, 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transit')
            if len(globalidsedges) == 0:
                raise ValueError("Plan tabletopgrips first!")
            for globalidedgesid in globalidsedges.keys():
                for edge in list(itertools.combinations(globalidsedges[globalidedgesid], 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transfer')

        # gen plot pos
        # biggest circle: grips; big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = {}
        xydiscreterotspos = {}
        xyzglobalgrippos = {}
        for i, ttpsid in enumerate(self.fttpsids):
            xydiscreterotspos[ttpsid] = {}
            xyzglobalgrippos[ttpsid] = {}
            xypos = [radiusplacement * math.cos(2 * math.pi / self.nfttps * i),
                     radiusplacement * math.sin(2 * math.pi / self.nfttps * i)]
            xyplacementspos[ttpsid] = xypos
            for j, anglevalue in enumerate(self.angles):
                xyzglobalgrippos[ttpsid][anglevalue] = {}
                xypos = [radiusrot * math.cos(math.radians(anglevalue)), radiusrot * math.sin(math.radians(anglevalue))]
                xydiscreterotspos[ttpsid][anglevalue] = \
                    [xyplacementspos[ttpsid][0] + xypos[0], xyplacementspos[ttpsid][1] + xypos[1]]
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip * math.cos(2 * math.pi / len(self.globalgripids) * k),
                             radiusgrip * math.sin(2 * math.pi / len(self.globalgripids) * k)]
                    xyzglobalgrippos[ttpsid][anglevalue][globalgripid] = \
                        [xydiscreterotspos[ttpsid][anglevalue][0] + xypos[0],
                         xydiscreterotspos[ttpsid][anglevalue][1] + xypos[1], 0]
        for nid in self.regg.nodes():
            fttpid = self.regg.node[nid]['freetabletopplacementid']
            anglevalue = self.regg.node[nid]['angle']
            ggid = self.regg.node[nid]['globalgripid']
            tabletopposition = self.regg.node[nid]['tabletopposition']
            xyzpos = map(add, xyzglobalgrippos[fttpid][anglevalue][ggid],
                          [tabletopposition[0], tabletopposition[1], tabletopposition[2]])
            self.gnodesplotpos[nid] = xyzpos[:2]

    def plotGraph(self, pltax, offset = [0,0]):
        """

        :param pltax:
        :param offset: where to plot the graph
        :return:
        """

        # add offset
        for i in self.gnodesplotpos.keys():
            self.gnodesplotpos[i] = map(add, self.gnodesplotpos[i], offset)

        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            if reggedgedata['edgetype'] is 'transit':
                transitedges.append([self.gnodesplotpos[nid0][:2], self.gnodesplotpos[nid1][:2]])
            if reggedgedata['edgetype'] is 'transfer':
                transferedges.append([self.gnodesplotpos[nid0][:2], self.gnodesplotpos[nid1][:2]])
        transitec = mc.LineCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        pltax.add_collection(transferec)
        pltax.add_collection(transitec)


class RegripTpp(object):

    def __init__(self, objpath, nxtrobot, handpkg, gdb, armname):

        self.graphtpp = GraphTpp(objpath, nxtrobot, handpkg, gdb, armname)
        self.armname = armname

        self.gdb = gdb
        self.robot = nxtrobot
        self.hand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = -53)
        self.bulletworld.attachRigidBody(self.planebullnode)

        # add tabletop plane model to bulletworld
        this_dir, this_filename = os.path.split(__file__)
        # TODO: change the shape of nxt.egg
        ttpath = Filename.fromOsSpecific(os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "supports", "tabletop_nxt.egg"))
        self.ttnodepath = NodePath("tabletop")
        ttl = loader.loadModel(ttpath)
        ttl.instanceTo(self.ttnodepath)

        self.endnodeids  = []

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        self.gnodesplotpos = {}

        self.freegripid = []
        self.freegripcontacts = []
        self.freegripnormals = []
        self.freegriprotmats = []
        self.freegripjawwidth = []

        # for start and goal grasps poses:
        self.radiusgrip = 1
        self.__xyzglobalgrippos_startgoal={}
        for k, globalgripid in enumerate(self.graphtpp.globalgripids):
            xypos = [self.radiusgrip * math.cos(2 * math.pi / len(self.graphtpp.globalgripids) * k),
                     self.radiusgrip * math.sin(2 * math.pi / len(self.graphtpp.globalgripids) * k)]
            self.__xyzglobalgrippos_startgoal[globalgripid] = [xypos[0],xypos[1],0]

        self.__loadFreeAirGrip()

    @property
    def dbobjname(self):
        # read-only property
        return self.graphtpp.dbobjname

    def __loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def addEnd(self, rotmat4):
        # the node id of a globalgripid in end
        nodeidofglobalidinend = {}
        # the endnodeids is also for quick access
        self.endnodeids = []
        for j, rotmat in enumerate(self.freegriprotmats):
            grotmat = rotmat * rotmat4
            # for collision detection, we move the object back to x=0,y=0
            objrotmatx0y0 = Mat4(rotmat4)
            objrotmatx0y0.setCell(3,0,0)
            objrotmatx0y0.setCell(3,1,0)
            grotmatx0y0 = rotmat * objrotmatx0y0
            # check if the hand collide with tabletop
            tmphand = self.hand
            initmat = tmphand.getMat()
            initjawwidth = tmphand.jawwidth
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            # tmphand.setJawwidth(self.freegripjawwidth[j])
            tmphand.setJawwidth(tmphand.jawwidthopen)
            tmphand.setMat(grotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                gcct0=rotmat4.xformPoint(self.freegripcontacts[j][0])
                gcct1=rotmat4.xformPoint(self.freegripcontacts[j][1])
                handx = grotmat.getRow3(0)
                # panda3d format
                gfgrcenter = (gcct0+gcct1)/2
                gfgrcenterhandx = gfgrcenter + handx*self.rethandx
                # handxworldz is not necessary for start
                # gfgrcenterhandxworldz = gfgrcenterhandx + self.worldz*self.retworldz
                gfgrcenterworlda = gfgrcenter + self.worlda*self.retworlda
                gfgrcenterworldaworldz = gfgrcenterworlda+ self.worldz*self.retworldz
                gjawwidth = self.freegripjawwidth[j]
                gidfreeair = self.freegripid[j]
                # numpy format
                gfgrcenternp = pg.v3ToNp(gfgrcenter)
                gfgrcenternp_handx = pg.v3ToNp(gfgrcenterhandx)
                # handxworldz is not necessary for start
                # gfgrcenternp_handxworldz = pg.v3ToNp(gfgrcenterhandxworldz)
                gfgrcenternp_worlda = pg.v3ToNp(gfgrcenterworlda)
                gfgrcenternp_worldaworldz = pg.v3ToNp(gfgrcenterworldaworldz)
                grotmat3np = pg.mat3ToNp(grotmat.getUpper3())
                ikc = self.robot.numikr(gfgrcenternp, grotmat3np, armid = self.armname)
                ikcx = self.robot.numikr(gfgrcenternp_handx, grotmat3np, armid = self.armname)
                ikca = self.robot.numikr(gfgrcenternp_worlda, grotmat3np, armid = self.armname)
                ikcaz = self.robot.numikr(gfgrcenternp_worldaworldz, grotmat3np, armid = self.armname)
                if (ikc is not None) and (ikcx is not None) and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = rotmat4.getRow3(3)
                    rotmat4worlda = Mat4(rotmat4)
                    rotmat4worlda.setRow(3, rotmat4.getRow3(3)+self.worlda*self.retworlda)
                    rotmat4worldaworldz = Mat4(rotmat4worlda)
                    rotmat4worldaworldz.setRow(3, rotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.graphtpp.regg.add_node('end'+self.armname+str(j), fgrcenter=gfgrcenternp,
                                       fgrcenterhandx = gfgrcenternp_handx,
                                       fgrcenterhandxworldz = 'na',
                                       fgrcenterworlda = gfgrcenternp_worlda,
                                       fgrcenterworldaworldz = gfgrcenternp_worldaworldz,
                                       jawwidth=gjawwidth, hndrotmat3np=grotmat3np,
                                       globalgripid = gidfreeair, freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = rotmat4,
                                       tabletopplacementrotmathandx = rotmat4,
                                       tabletopplacementrotmathandxworldz = 'na',
                                       tabletopplacementrotmatworlda = rotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = rotmat4worldaworldz,
                                       angle = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidinend[gidfreeair]='end'+self.armname+str(j)
                    self.endnodeids.append('end'+self.armname+str(j))
            tmphand.setMat(initmat)
            tmphand.setJawwidth(initjawwidth)

        if len(self.endnodeids) == 0:
            raise ValueError("No available end grip at " + self.armname)

        # add start transit edge
        for edge in list(itertools.combinations(self.endnodeids, 2)):
            self.graphtpp.regg.add_edge(*edge, weight = 1, edgetype = 'endtransit')
        # add start transfer edge
        for reggnode, reggnodedata in self.graphtpp.regg.nodes(data=True):
            if reggnode.startswith(self.armname):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidinend.keys():
                    endnodeid = nodeidofglobalidinend[globalgripid]
                    self.graphtpp.regg.add_edge(endnodeid, reggnode, weight=1, edgetype = 'endtransfer')

        for nid in self.graphtpp.regg.nodes():
            if nid.startswith('end'):
                ggid = self.graphtpp.regg.node[nid]['globalgripid']
                tabletopposition = self.graphtpp.regg.node[nid]['tabletopposition']
                xyzpos = map(add, self.__xyzglobalgrippos_startgoal[ggid],
                              [tabletopposition[0], tabletopposition[1], tabletopposition[2]])
                self.gnodesplotpos[nid] = xyzpos[:2]

    def deleteEnd(self):
        for nid in list(self.graphtpp.regg.nodes()):
            if nid.startswith('end'):
                self.graphtpp.regg.remove_node(nid)

    def plotGraph(self, pltax, gtppoffset = [0,0]):
        """

        :param pltax:
        :param endname:
        :param gtppoffset: where to plot graphtpp, see the plotGraph function of GraphTpp class
        :return:
        """

        self.graphtpp.plotGraph(pltax, offset = gtppoffset)
        self.__plotEnds(pltax)

    def __plotEnds(self, pltax):
        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.graphtpp.regg.edges(data=True):
            if nid0.startswith('end'):
                pos0 = self.gnodesplotpos[nid0][:2]
            else:
                pos0 = self.graphtpp.gnodesplotpos[nid0][:2]
            if nid1.startswith('end'):
                pos1 = self.gnodesplotpos[nid1][:2]
            else:
                pos1 = self.graphtpp.gnodesplotpos[nid1][:2]
            if (reggedgedata['edgetype'] == 'endtransit'):
                transitedges.append([pos0, pos1])
            elif (reggedgedata['edgetype'] is 'endtransfer'):
                transferedges.append([pos0, pos1])
        transitec = mc.LineCollection(transitedges, colors = [.5,0,0,.3], linewidths = 1)
        transferec = mc.LineCollection(transferedges, colors = [1,0,0,.3], linewidths = 1)
        pltax.add_collection(transferec)
        pltax.add_collection(transitec)


class RegripTppAss(object):

    def __init__(self, base, obj0path, obj0Mat4, obj1path, obj1Mat4, assDirect1to0, gdb, robot, handpkg):
        """
        see parameters of assembly/asstwoobj

        :param base:
        :param obj0path:
        :param obj0Mat4:
        :param obj1path:
        :param obj1Mat4:
        :param assDirect1to0:
        :param gdb:
        :param robot:
        :param handpkg:

        author: weiwei
        date: 20160308
        """

        self.robot = robot
        self.robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        self.toass = Toass(base, obj0path, obj0Mat4, obj1path, obj1Mat4, assDirect1to0, gdb, handpkg)
        self.toass.loadIKFeasibleAGPairsFromDB(robot)
        self.regghalf = [RegripTpp(obj0path, robot, handpkg, gdb, 'rgt'), RegripTpp(obj1path, robot, handpkg, gdb, 'lft')]
        self.objrotmat4s = [obj0Mat4, obj1Mat4]
        self.retass = [assDirect1to0, -assDirect1to0]

        self.regg = []
        self.gnodesplotpos = {}
        self.composedgnodesplotpos = {}

        self.directshortestpaths = []

    def findshortestpath(self, obj0SRotmat4, obj1SRotmat4):
        self.__addEnds(obj0SRotmat4, obj1SRotmat4)
        # startrgt goalrgt
        if len(self.regghalf[0].endnodeids) > 0 and len(self.regghalf[1].endnodeids) > 0:
            startgrip = self.regghalf[0].endnodeids[0]
            goalgrip = self.regghalf[1].endnodeids[0]
            shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths = []
            print(shortestpaths)
            for path in shortestpaths:
                for i, pathnode in enumerate(path):
                    if pathnode.startswith('endrgt') and i < len(path)-1:
                        continue
                    else:
                        self.directshortestpaths.append(path[i-1:])
                        break
                for i, pathnode in enumerate(self.directshortestpaths[-1]):
                    if i > 0 and pathnode.startswith('endlft'):
                        self.directshortestpaths[-1]=self.directshortestpaths[-1][:i+1]
                        break

    def plotGraph(self, pltax, offset0 = [600, -800], offset1 = [600, 800]):
        self.regghalf[0].plotGraph(pltax, offset0)
        self.regghalf[1].plotGraph(pltax, offset1)
        # # add offset
        for nid in self.gnodesplotpos.keys():
            if nid.startswith('assrgt'):
                self.gnodesplotpos[nid] = map(add, self.gnodesplotpos[nid], [offset0[0], 0])
            if nid.startswith('asslft'):
                self.gnodesplotpos[nid] = map(add, self.gnodesplotpos[nid], [offset1[0], 0])

        # make composed gnodesplotpos
        self.composedgnodesplotpos = {}
        for key in self.gnodesplotpos.keys():
            self.composedgnodesplotpos[key] = self.gnodesplotpos[key]
        for key in self.regghalf[0].gnodesplotpos.keys():
            self.composedgnodesplotpos[key] = self.regghalf[0].gnodesplotpos[key]
        for key in self.regghalf[0].graphtpp.gnodesplotpos.keys():
            self.composedgnodesplotpos[key] = self.regghalf[0].graphtpp.gnodesplotpos[key]
        for key in self.regghalf[1].gnodesplotpos.keys():
            self.composedgnodesplotpos[key] = self.regghalf[1].gnodesplotpos[key]
        for key in self.regghalf[1].graphtpp.gnodesplotpos.keys():
            self.composedgnodesplotpos[key] = self.regghalf[1].graphtpp.gnodesplotpos[key]

        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            if reggedgedata['edgetype'] is 'asstransit':
                transitedges.append([self.composedgnodesplotpos[nid0][:2], self.composedgnodesplotpos[nid1][:2]])
            if reggedgedata['edgetype'] is 'asstransfer':
                transferedges.append([self.composedgnodesplotpos[nid0][:2], self.composedgnodesplotpos[nid1][:2]])
        transitec = mc.LineCollection(transitedges, colors=[1,0,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[.5,.5,0,.03], linewidths=1)
        pltax.add_collection(transferec)
        pltax.add_collection(transitec)

    def plotshortestpath(self, pltax, id=0):
        """
        plot the shortest path

        :param id:
        :return:
        """

        for i,path in enumerate(self.directshortestpaths):
            if i is id:
                pathedges = []
                pathlength = len(path)
                for pnidx in range(pathlength-1):
                    nid0 = path[pnidx]
                    nid1 = path[pnidx+1]
                    pathedges.append([self.composedgnodesplotpos[nid0][:2], self.composedgnodesplotpos[nid1][:2]])
                pathedgesec = mc.LineCollection(pathedges, colors=[0, 1, 0, 1], linewidths=5)

                pltax.add_collection(pathedgesec)


    def __addEnds(self, obj0SRotmat4, obj1SRotmat4):
        """
        add the two ends to the graph

        :param obj0SRotmat4:
        :param obj1SRotmat4:
        :return:
        """

        self.regghalf[0].deleteEnd()
        self.regghalf[1].deleteEnd()
        self.regghalf[0].addEnd(obj0SRotmat4)
        self.regghalf[1].addEnd(obj1SRotmat4)
        self.regg = nx.compose(self.regghalf[0].graphtpp.regg, self.regghalf[1].graphtpp.regg)
        self.__addAssNodes(armname = 'rgt')
        self.__addAssNodes(armname = 'lft')
        self.__bridgeGraph()

    def __addAssNodes(self, armname = 'rgt'):
        iele = 0
        if armname == 'lft':
            iele = 1
        freeairidontpp = {}
        # for plot
        radiusgrip = self.regghalf[iele].radiusgrip
        for nid in self.regghalf[iele].graphtpp.regg.nodes():
            dictind = str(self.regghalf[iele].graphtpp.regg.node[nid]['globalgripid'])
            if dictind in freeairidontpp:
                freeairidontpp[dictind].append(nid)
            else:
                freeairidontpp[dictind] = []
        # add floatingposes
        freeairidonass = {}
        for asspind, assprotmat4 in enumerate(self.toass.gridsfloatingposemat4s):
            retass = assprotmat4.xformVec(self.retass[iele])
            for pairind, hndrotmat4pair in enumerate(self.toass.icoassgrippairshndmat4s[asspind]):
                assgid = self.toass.icoassgrippairsids[asspind][pairind][iele]
                assgidfreeair = self.toass.icoassgrippairsidfreeairs[asspind][pairind][iele]
                ccts = self.toass.icoassgrippairscontacts[asspind][pairind][iele]
                hndrotmat4 = hndrotmat4pair[iele]
                asspgfgrcenter = (Vec3(ccts[0][0], ccts[0][1], ccts[0][2]) + Vec3(ccts[1][0], ccts[1][1], ccts[1][2])) / 2
                asspgfgrcenter_retass = asspgfgrcenter + retass
                asspgfgrcenternp = pg.v3ToNp(asspgfgrcenter)
                asspgfgrcenter_retassnp = pg.v3ToNp(asspgfgrcenter_retass)
                jawwidth = self.toass.icoassgrippairsjawwidths[asspind][pairind][iele]
                hndrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                objrotmat4 = self.objrotmat4s[iele]*assprotmat4
                objrotmat4retass = Mat4(objrotmat4)
                objrotmat4retass.setRow(3, objrotmat4retass.getRow3(3)+retass)
                self.regg.add_node('ass' + armname + str(assgid), fgrcenter=asspgfgrcenternp,
                                   fgrcenterretass=asspgfgrcenter_retassnp, jawwidth=jawwidth,
                                   hndrotmat3np=hndrotmat3np, assposerotmat4=objrotmat4,
                                   assposerotmat4retass=objrotmat4retass, assposeind=asspind,
                                   icoassgrippairsid=pairind, globalgripid=assgidfreeair)
                if str(assgidfreeair) in freeairidonass:
                    freeairidonass[str(assgidfreeair)].append('ass' + armname + str(assgid))
                else:
                    freeairidonass[str(assgidfreeair)] = []
        for freeairidontppkey in freeairidontpp.keys():
            try:
                for edge in list(itertools.product(freeairidontpp[freeairidontppkey], freeairidonass[freeairidontppkey])):
                    self.regg.add_edge(*edge, weight=1, edgetype='asstransfer')
            except:
                pass
        # plot pos
        nfp = len(self.toass.gridsfloatingposemat4s)
        xdist = 10
        x = range(300,501,xdist)
        y = range(-50,50,100*xdist/nfp)
        for nid in self.regg.nodes():
            if nid.startswith('ass'):
                asspind = self.regg.node[nid]['assposeind']
                assgind = self.regg.node[nid]['icoassgrippairsid']
                nassg = len(self.toass.icoassgrippairshndmat4s[asspind])
                xpos = x[asspind % len(x)]
                ypos = y[asspind/len(x)]
                xyzpos = [radiusgrip * math.cos(2 * math.pi / nassg * assgind)+xpos,
                         radiusgrip * math.sin(2 * math.pi / nassg * assgind)+ypos, 0]
                self.gnodesplotpos[nid] = xyzpos[:2]
                if nid.startswith('assrgt'):
                    self.gnodesplotpos[nid][1] = self.gnodesplotpos[nid][1] - 100
                elif nid.startswith('asslft'):
                    self.gnodesplotpos[nid][1] = self.gnodesplotpos[nid][1] + 100

    def __bridgeGraph(self):
        for fpind, objrotmat4 in enumerate(self.toass.gridsfloatingposemat4s):
            for pairind, hndrotmat4pair in enumerate(self.toass.icoassgrippairshndmat4s[fpind]):
                fpgid0 = self.toass.icoassgrippairsids[fpind][pairind][0]
                fpgid1 = self.toass.icoassgrippairsids[fpind][pairind][1]
                self.regg.add_edge('assrgt'+str(fpgid0), 'asslft'+str(fpgid1), weight = 1, edgetype = 'asstransit')

if __name__=='__main__':
    gdb = db.GraspDB()
    nxtrobot = nextage.NxtRobot()
    handpkg = rtq85nm

    base = pandactrl.World(camp=[700,300,600], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    obj0path = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planefrontstay.stl")
    obj0Mat4 = Mat4.identMat()
    obj1path = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planewheel.stl")
    obj1Mat4 = Mat4(obj0Mat4)
    obj1Mat4.setCell(3,1,32)
    obj1Mat4.setCell(3,2,10)

    assDirect1to0 = Vec3(0, -70, 0)

    obj0trimesh = trimesh.load_mesh(obj0path)
    obj0np = pg.packpandanp(obj0trimesh.vertices, obj0trimesh.face_normals, obj0trimesh.faces)
    obj0np.setMat(obj0Mat4)
    obj0np.setColor(.7,.3,0)
    obj1trimesh = trimesh.load_mesh(obj1path)
    obj1np = pg.packpandanp(obj1trimesh.vertices, obj1trimesh.face_normals, obj1trimesh.faces)
    obj1np.setMat(obj1Mat4)
    obj1np.setColor(0,.3,0.7)
    sprotmat4 = Mat4(1.0,0.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     0.0,-1.0,0.0,0.0,\
                     350,-400,15.0,1.0)
    whrotmat4 = Mat4(1.0,0.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     0.0,-1.0,0.0,0.0,\
                     350,400,0.0,1.0)

    regass = RegripTppAss(base, obj0path, obj0Mat4, obj1path, obj1Mat4, assDirect1to0, gdb, nxtrobot, handpkg)
    regass.findshortestpath(obj0SRotmat4 = sprotmat4, obj1SRotmat4 = whrotmat4)
    pltfig = plt.figure()
    ax = pltfig.add_subplot(111)
    regass.plotGraph(ax, offset0 = [600,-800], offset1 = [600, 800])
    print(regass.directshortestpaths)
    regass.plotshortestpath(ax)

    plt.axis("equal")
    plt.show()

    # obj1np.setMat(whrotmat4)
    # obj1np.reparentTo(base.render)
    # pg.plotAxis(base.render)

    base.run()