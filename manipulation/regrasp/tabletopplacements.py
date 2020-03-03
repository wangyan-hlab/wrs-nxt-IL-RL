#!/usr/bin/python

import os
import time

import numpy as np
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import trimesh
from database import dbaccess as db, dbcvt as dc
from manipulation.grip.robotiq85 import rtq85nm
from utiltools import robotmath as rm

class TabletopPlacements(object):
    """
    manipulation.freetabletopplacement doesn't take into account
    the position and orientation of the object
    it is "free" in position and rotation around z axis
    in contrast, each item in regrasp.tabletopplacements
    has different position and orientation
    it is at a specific pose in the workspace
    To clearly indicate the difference, "free" is attached
    to the front of "freetabletopplacement"
    "s" is attached to the end of "tabletopplacements"
    """

    def __init__(self, objpath, handpkg):
        """
        initialization

        :param objpath: path of the object

        author: weiwei
        date: 20161215, osaka
        """

        self.objtrimesh = trimesh.load_mesh(objpath)
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()

    def saveToDB(self, positionlist, gdb, discretesize=8):
        """

        :param positionlist: a list of positions to place the object one the table
        :param discretesize: the discretization of rotation angles around z axis
        :return:

        author: weiwei
        date: 20161215, osaka
        """

        # save discretiezed angle
        sql = "SELECT * FROM angle"
        result = gdb.execute(sql)
        if len(result) == 0:
            sql = "INSERT INTO angle(value) VALUES "
            for i in range(discretesize):
                sql += "("+str(360*i*1.0/discretesize)+"), "
            sql = sql[:-2]+";"
            gdb.execute(sql)
        else:
            print("Angles already set!")

        # 1) select the tabletopplacements
        sql = "SELECT freetabletopplacement.idfreetabletopplacement, freetabletopplacement.rotmat \
                    FROM freetabletopplacement,object WHERE freetabletopplacement.idobject = object.idobject \
                    AND object.name LIKE '%s'" % self.dbobjname
        result = gdb.execute(sql)
        if len(result) == 0:
            raise Exception("Plan the freetabletopplacement first!")
            return
        result = np.asarray(result)
        idfreettplist = [int(x) for x in result[:, 0]]
        rotmatfreettplist = [dc.strToMat4(x) for x in result[:, 1]]
        # 2) select the angle
        sql = "SELECT angle.idangle,angle.value FROM angle"
        result = np.asarray(gdb.execute(sql))
        idanglelist = [int(x) for x in result[:, 0]]
        anglevaluelist = [float(x) for x in result[:, 1]]
        # 3) save tabletopplacements
        sql = "SELECT idtabletopplacements FROM tabletopplacements,freetabletopplacement,object WHERE \
                tabletopplacements.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                 freetabletopplacement.idobject=object.idobject AND object.name LIKE '%s'" % self.dbobjname
        result = gdb.execute(sql)
        # 3.1) check if already exist
        if len(result) != 0:
            print("Tabletopplacements already exist! ")
            isredo = input("Do you want to overwrite the database? (Y/N)")
            if isredo != "Y" and isredo != "y":
                print("Placement planning aborted.")
                return
            else:
                for idfreettp in idfreettplist:
                    sql = "DELETE FROM tabletopplacements WHERE \
                           tabletopplacements.idfreetabletopplacement LIKE '%s'" % idfreettp
                    gdb.execute(sql)
                    print("Old tabletopplacements have been deleted!")
        # save to database
        sql = "INSERT INTO tabletopplacements(rotmat, tabletopposition, idangle, idfreetabletopplacement) VALUES "
        for ttoppos in positionlist:
            ttoppos = Point3(ttoppos[0], ttoppos[1], ttoppos[2])
            for idfree, rotmatfree in zip(idfreettplist, rotmatfreettplist):
                for idangle, anglevalue in zip(idanglelist, anglevaluelist):
                    rotangle = anglevalue
                    rotmat = rm.rodrigues([0, 0, 1], rotangle)
                    rotmat4 = pg.npToMat4(rotmat, ttoppos)
                    varrotmat = rotmatfree * rotmat4
                    sql += "('%s', '%s', %d, %d), " % \
                          (dc.mat4ToStr(varrotmat), dc.v3ToStr(ttoppos), idangle, idfree)
        sql = sql[:-2]+";"
        gdb.execute(sql)
        print("Tabletop placements saved!")

        # save tabletopgrips
        isttg, ttgresult = self.__getTtg(gdb)
        if not isttg:
            idhand = gdb.loadIdHand(self.handname)
            for idfree in idfreettplist:
                sql = "SELECT tabletopplacements.idtabletopplacements, \
                        tabletopplacements.tabletopposition, angle.value,\
                        freetabletopgrip.contactpoint0, freetabletopgrip.contactpoint1, \
                        freetabletopgrip.contactnormal0, freetabletopgrip.contactnormal1, \
                        freetabletopgrip.rotmat, freetabletopgrip.jawwidth, freetabletopgrip.idfreeairgrip \
                        FROM tabletopplacements,freetabletopplacement,freetabletopgrip,freeairgrip,angle WHERE \
                        tabletopplacements.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement AND \
                        tabletopplacements.idangle = angle.idangle AND \
                        freetabletopgrip.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement AND \
                        freetabletopgrip.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                        freeairgrip.idhand = %d AND \
                        freetabletopplacement.idfreetabletopplacement = %d" % (idhand, idfree)
                result1 = gdb.execute(sql)
                if len(result1) == 0:
                    # no availalbe grasps
                    continue
                if len(result1) > 20000:
                    result1 = result1[0::int(len(result1)/20000.0)]
                result1 = np.asarray(result1)
                idtabletopplacementslist = [int(x) for x in result1[:,0]]
                tabletoppositionlist = [dc.strToV3(x) for x in result1[:,1]]
                rotanglelist = [float(x) for x in result1[:,2]]
                freegripcontactpoint0list = [dc.strToV3(x) for x in result1[:,3]]
                freegripcontactpoint1list = [dc.strToV3(x) for x in result1[:,4]]
                freegripcontactnormal0list = [dc.strToV3(x) for x in result1[:,5]]
                freegripcontactnormal1list = [dc.strToV3(x) for x in result1[:,6]]
                freegriprotmatlist = [dc.strToMat4(x) for x in result1[:,7]]
                freegripjawwidthlist = [float(x) for x in result1[:,8]]
                freegripidlist = [int(x) for x in result1[:,9]]
                for idtabletopplacements, ttoppos, rotangle, cct0, cct1, cctn0, cctn1, \
                    freegriprotmat, jawwidth, idfreegrip in zip(idtabletopplacementslist, \
                    tabletoppositionlist, rotanglelist, freegripcontactpoint0list, freegripcontactpoint1list, \
                    freegripcontactnormal0list, freegripcontactnormal1list, freegriprotmatlist, freegripjawwidthlist, \
                    freegripidlist):
                    rotmat = rm.rodrigues([0, 0, 1], rotangle)
                    rotmat4 = pg.npToMat4(rotmat, ttoppos)
                    ttpcct0 = rotmat4.xformPoint(cct0)
                    ttpcct1 = rotmat4.xformPoint(cct1)
                    ttpcctn0 = rotmat4.xformVec(cctn0)
                    ttpcctn1 = rotmat4.xformVec(cctn1)
                    ttpgriprotmat = freegriprotmat*rotmat4
                    sql = "INSERT INTO tabletopgrips(contactpnt0, contactpnt1, contactnormal0, contactnormal1, \
                            rotmat, jawwidth, idfreeairgrip, idtabletopplacements) VALUES \
                            ('%s', '%s', '%s', '%s', '%s', '%s', %d, %d) " % \
                           (dc.v3ToStr(ttpcct0), dc.v3ToStr(ttpcct1), dc.v3ToStr(ttpcctn0), dc.v3ToStr(ttpcctn1), \
                            dc.mat4ToStr(ttpgriprotmat), str(jawwidth), idfreegrip, idtabletopplacements)
                    gdb.execute(sql)
        else:
            # TODO delete this one, placements and grasps always exist together
            print("Tabletopgrips already exist!")
            raise Exception("Warning! Grasps are NOT saved!")
            return
        print("Grasps saved!")

    def __getTtg(self, gdb):
        """
        # check if tabletopgrips exist

        :return:
        author: weiwei
        date: 20190322, toyonaka
        """

        sql = "SELECT tabletopgrips.idtabletopgrips, tabletopgrips.contactpnt0, tabletopgrips.contactpnt1, \
                tabletopgrips.rotmat FROM tabletopgrips,freeairgrip,object,hand WHERE \
                tabletopgrips.idfreeairgrip=freeairgrip.idfreeairgrip AND \
                 freeairgrip.idobject=object.idobject AND freeairgrip.idhand=hand.idhand AND object.name LIKE '%s' AND \
                  hand.name LIKE '%s'" % (self.dbobjname, self.handname)
        result = gdb.execute(sql)
        if len(result) == 0:
            return False, result
        else:
            return True, result

    def updateDBwithIK(self, gdb, robot, armname = 'rgt'):
        """

        :param gdb:
        :param robot:
        :param armname: the name of the arm used rgt or lft
        :return:

        author: weiwei
        date: 20170111
        """
        # load idarm
        idarm = gdb.loadIdArm(armname)

        # load retraction distances
        rethanda, retworlda, worlda = gdb.loadIKRet()

        # select idrobot
        idrobot = gdb.loadIdRobot(robot)

        feasibility = {}
        feasibility_handa = {}
        feasibility_worlda = {}
        jnts = {}
        jnts_handa = {}
        jnts_worlda = {}

        isttg, ttgresult = self.__getTtg(gdb)
        if not isttg:
            raise ValueError("Plan the tabletopgrips first!")
        else:
            for resultrow in ttgresult:
                ttgsid = int(resultrow[0])
                sql = "SELECT * FROM iktabletopgrips WHERE iktabletopgrips.idrobot = %d AND idarm = %d \
                       AND idtabletopgrips = %d" % (idrobot, idarm, ttgsid)
                result = gdb.execute(sql)
                if len(result) != 0:
                    print("IK for the "+armname+" arm is already updated!")
                    isredo = input("Do you want to overwrite the database? (Y/N)")
                    if isredo != "Y" and isredo != "y":
                        print("Updating IK is aborted.")
                        return
                    else:
                        for resultrow in ttgresult:
                            ttgsid = int(resultrow[0])
                            sql = "DELETE FROM iktabletopgrips WHERE iktabletopgrips.idrobot = %d AND idarm = %d \
                                   AND idtabletopgrips = %d" % (idrobot, idarm, ttgsid)
                            gdb.execute(sql)
                            print("Old tabletopplacements are being deleted!")
                        print("Old tabletopplacements have been deleted!")
                        break

        idcounter = 0
        tic = time.time()
        for resultrow in ttgresult:
            toc = time.time()
            print("Arm: ", armname, "    Finished: ",  "%.2f" % (idcounter*1.0/len(ttgresult)*100.0)+"%",
                  "    Time past: ", "%.2f" % (toc-tic), "s",
                  "    Expected remaining time: ", "%.2f" % ((toc-tic)/(idcounter*1.0/len(ttgresult)+1e-4)-(toc-tic)), "s")
            idcounter += 1
            ttgsid = int(resultrow[0])
            ttgscct0 = dc.strToV3(resultrow[1])
            ttgscct1 = dc.strToV3(resultrow[2])
            ttgsrotmat = dc.strToMat4(resultrow[3])
            ttgsfgrcenter = (ttgscct0 + ttgscct1) / 2.0
            handa = -ttgsrotmat.getRow3(2)
            ttgsfgrcenterhanda = ttgsfgrcenter + handa*rethanda
            ttgsfgrcenterworlda = ttgsfgrcenter + worlda*retworlda

            ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
            ttgsfgrcenternp_handa = pg.v3ToNp(ttgsfgrcenterhanda)
            ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
            ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())

            msc = robot.numik(ttgsfgrcenternp, ttgsrotmat3np, armname)
            if msc is not None:
                feasibility[ttgsid] = 'True'
                jnts[ttgsid] = dc.listToStr(msc)
                msc_handa = robot.numikmsc(ttgsfgrcenternp_handa, ttgsrotmat3np, msc, armname)
                if msc_handa is not None:
                    feasibility_handa[ttgsid] = 'True'
                    jnts_handa[ttgsid] = dc.listToStr(msc_handa)
                else:
                    feasibility_handa[ttgsid] = 'False'
                    jnts_handa[ttgsid] = dc.listToStr([])
                msc_worlda = robot.numikmsc(ttgsfgrcenternp_worlda, ttgsrotmat3np, msc, armname)
                if msc_worlda is not None:
                    feasibility_worlda[ttgsid] = 'True'
                    jnts_worlda[ttgsid] = dc.listToStr(msc_worlda)
                else:
                    feasibility_worlda[ttgsid] = 'False'
                    jnts_worlda[ttgsid] = dc.listToStr([])
            else:
                feasibility[ttgsid] = 'False'
                feasibility_handa[ttgsid] = 'False'
                feasibility_worlda[ttgsid] = 'False'
                jnts[ttgsid] = dc.listToStr([])
                jnts_handa[ttgsid] = dc.listToStr([])
                jnts_worlda[ttgsid] = dc.listToStr([])
            # insert ik table
            sql = "INSERT INTO iktabletopgrips(idrobot, idarm, idtabletopgrips, feasibility, feasibility_handa, \
                    feasibility_worlda, jnts, jnts_handa, jnts_worlda) VALUES (%d, %d, %d, '%s', '%s', '%s', '%s', '%s', '%s')" % \
                    (idrobot, idarm, ttgsid, feasibility[ttgsid], feasibility_handa[ttgsid], feasibility_worlda[ttgsid], \
                     jnts[ttgsid], jnts_handa[ttgsid], jnts_worlda[ttgsid])
            gdb.execute(sql)
        print("IK updated!")

    def grpshow(self, base, gdb):
        sql = "SELECT tabletopplacements.idtabletopplacements, tabletopplacements.rotmat \
                FROM tabletopplacements,freetabletopplacement,object,angle WHERE \
                tabletopplacements.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement AND \
                freetabletopplacement.idobject = object.idobject AND object.name LIKE '%s' AND \
                tabletopplacements.idangle = angle.idangle AND \
                freetabletopplacement.idfreetabletopplacement = %d AND angle.value = %d" % (self.dbobjname, 1, 45)
        result = gdb.execute(sql)
        if len(result) != 0:
            for resultrow in result:
                idtabletopplacements = int(resultrow[0])
                objrotmat  = dc.strToMat4(resultrow[1])
                objpos = objrotmat.getRow3(3)
                base.changeLookAt(lookatp=[objpos[0],objpos[1],objpos[2]])
                # show object
                geom = pg.packpandageom_fn(self.objtrimesh.vertices,
                                        self.objtrimesh.face_normals,
                                        self.objtrimesh.faces)
                node = GeomNode('obj')
                node.addGeom(geom)
                star = NodePath('obj')
                star.attachNewNode(node)
                star.setColor(Vec4(.77,.67,0,1))
                star.setTransparency(TransparencyAttrib.MAlpha)
                star.setMat(objrotmat)
                star.reparentTo(base.render)
                sql = "SELECT tabletopgrips.rotmat, tabletopgrips.jawwidth FROM tabletopgrips WHERE \
                        tabletopgrips.idtabletopplacements=%d" % idtabletopplacements
                result = gdb.execute(sql)
                for resultrow in result:
                    hndrotmat = dc.strToMat4(resultrow[0])
                    hndjawwidth = float(resultrow[1])
                    # show grasps
                    tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .1])
                    tmphnd.setMat(pandanpmat4 = hndrotmat)
                    # tmphnd.setMat(npmat4=hndrotmat)
                    tmphnd.setJawwidth(hndjawwidth)
                    tmphnd.reparentTo(base.render)

if __name__ == '__main__':
    # nxtrobot = nxt.NxtRobot()
    # hrp5robot = hrp5.Hrp5Robot()
    # hrp5n = hrp5n.Hrp5NRobot()
    # hrp2k = hrp2k.Hrp2KRobot()

    base = pandactrl.World(camp=[1000,400,1000], lookatp=[400,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "sandpart.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "ttube.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "tool.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "tool2.stl")
    # done 20170307
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planewheel.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planelowerbody.stl")
    # done 20170313
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planefrontstay.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planerearstay.stl")
    objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "housing.stl")

    # from manipulation.grip.hrp5three import hrp5threenm
    # handpkg = hrp5threenm
    handpkg = rtq85nm
    print(objpath)
    ttps = TabletopPlacements(objpath, handpkg)

    # plot obj and its convexhull
    # geom = pandageom.packpandageom(tps.objtrimesh-.vertices,
    #                                tps.objtrimesh.face_normals,
    #                                tps.objtrimesh.faces)
    # node = GeomNode('obj')
    # node.addGeom(geom)
    # star = NodePath('obj')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,0,0,1))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    #
    # geom = pandageom.packpandageom(tps.objtrimeshconv.vertices,
    #                                tps.objtrimeshconv.face_normals,
    #                                tps.objtrimeshconv.faces)
    # node = GeomNode('objconv')
    # node.addGeom(geom)
    # star = NodePath('objconv')
    # star.attachNewNode(node)
    # star.setColor(Vec4(0, 1, 0, .3))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)

    # def updateshow(task):
    #     # tps.ocfacetshow(base)
    #     tps.removebadfacetsshow(base, doverh=.033)
    #     return task.again
    #
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")


    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    # build grid space
    grids = []
    for x in range(400,551,100):
        for y in range(-300,301,600):
            grids.append([x,y,-55])
    # grids = []
    # for x in range(400,401,200):
    #     for y in range(-250,251,600):
    #         grids.append([x,y,150])
    gdb = db.GraspDB()
    ttps.saveToDB(grids, gdb)
    # # # tps.grpshow(base, gdb)
    # tps.updateDBwithIK(gdb, hrp5n, armname = "rgt")
    # tps.updateDBwithIK(gdb, hrp5n, armname = "lft")

    from robotsim.ur3dual import ur3dual

    ttps.updateDBwithIK(gdb, ur3dual, armname = "rgt")
    ttps.updateDBwithIK(gdb, ur3dual, armname = "lft")
    # ttps.updateDBwithIK(gdb, hrp2k, armname = "rgt")
    # ttps.updateDBwithIK(gdb, hrp2k, armname = "lft")

    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    #
    # tps.bulletworldtable.setDebugNode(debugNP.node())
    #
    # taskMgr.add(updateworld, "updateworld", extraArgs=[tps.bulletworldtable], appendTask=True)
    ttps.grpshow(base, gdb)

    base.run()