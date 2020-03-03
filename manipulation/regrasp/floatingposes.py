#!/usr/bin/python

import itertools
import os
import json
import time

from panda3d.bullet import BulletWorld
from panda3d.core import *

import trimesh
from database import dbcvt as dc
from utiltools import collisiondetection as cd

class FloatingPoses(object):
    """
    like freetabletopplacement and tableplacements
    manipulation.floatingposes corresponds to freegrip
    floatingposes doesn't take into account
    the position and orientation of the object
    it is "free" in position and rotation. No obstacles were considered.
    In contrast, each item in regrasp.floatingposes
    has different position and orientation
    it is at a specific pose in the workspace
    To clearly indicate the difference, "free" is attached
    to the front of "freegrip"
    "s" is attached to the end of "floatingposes"

    meanings of abbreviations: FP = floating pose, G = grasps, GP = grasping pairs

    author: weiwei
    date: 2016, 2019
    """

    def __init__(self, objpath, gdb, handpkg, base):
        """
        initialization

        :param objpath: path of the object
        :param base: for the loadFreeAirGrip --> genHandPairs functions (for collision detection)

        author: weiwei
        date: 20161215, tsukuba
        """

        self.__base = base
        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand0 = handpkg.newHandNM(hndcolor=[1,0,0,.1])
        self.hand1 = handpkg.newHandNM(hndcolor=[0,0,1,.1])

        self.objtrimesh = trimesh.load_mesh(objpath)
        self.objnp = self.__base.pg.packpandanp_fn(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]
        # mat4list is nested list, floatingposemat4 is list (the flat version of mat4lsit), they are essentially the same
        self.mat4list = []
        self.floatingposemat4 = []
        # gridsfloatingposemat4s is self.floatingposemat4 translated to different grids
        self.gridsfloatingposemat4s = []
        self.icos = None

        self.floatinggripids = []
        self.floatinggripmat4s = []
        self.floatinggripcontacts = []
        self.floatinggripnormals = []
        self.floatinggripjawwidth = []
        self.floatinggripidfreeair = []

        self.bulletworld = BulletWorld()
        self.handpairList = []

        self.gdb = gdb

        self.__loadFreeAirGrip(base)

        # for IK
        self.feasibility_rgt = []
        self.feasibility_handa_rgt = []
        self.feasibility_lft = []
        self.feasibility_handa_lft = []
        self.jnts_rgt = []
        self.jnts_handa_rgt = []
        self.jnts_lft = []
        self.jnts_handa_lft = []

        # for ik-feasible pairs
        self.floatinggrippairsids = []
        self.floatinggrippairshndmat4s = []
        self.floatinggrippairscontacts = []
        self.floatinggrippairsnormals = []
        self.floatinggrippairsjawwidths = []
        self.floatinggrippairsidfreeairs = []
        self.floatinggrippairsjnts = []
        self.floatinggrippairsjnts_handa = []

    def __loadFreeAirGrip(self, base):
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

    def __genPandaRotmat4(self, icolevel=1, angles=[0,45,90,135,180,225,270,315]):
        """
        generate panda3d rotmat4 using icospheres and rotationaangle each origin-vertex vector of the icosphere

        :param icolevel, the default value 1 = 42vertices
        :param angles, 8 directions by default
        :return: a list of [pandarotmat4, ...] size of the inner list is size of the angles

        author: weiwei
        date: 20170221, tsukuba
        """

        self.mat4list = []
        self.icos = trimesh.creation.icosphere(icolevel)
        initmat4 = self.objnp.getMat()
        for vert in self.icos.vertices:
            self.mat4list.append([])
            self.objnp.lookAt(vert[0], vert[1], vert[2])
            ytozmat4 = Mat4.rotateMat(-90, self.objnp.getMat().getRow3(0))
            newobjmat4 = self.objnp.getMat()*ytozmat4
            for angle in angles:
                tmppandamat4 = Mat4.rotateMat(angle, newobjmat4.getRow3(2))
                tmppandamat4 = newobjmat4*tmppandamat4
                self.mat4list[-1].append(tmppandamat4)
            self.objnp.setMat(initmat4)
        self.floatingposemat4 = [e for l in self.mat4list for e in l]
        print(len(self.floatingposemat4))

    def __checkFPexistance(self):
        """
        check if the floating pose, grasps, and the handover pairs already exist
        if exist, return True
        else return False

        :return: boolean value indicating the existance of floating poses, grasps, and the handover pairs

        author: weiwei
        date: 20190318
        """

        sql = "SELECT * FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                        AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) > 0:
            print("Floating poses already saved!")
            isredo = input("Do you want to overwrite the database? (Y/N)")
            if isredo != "Y" and isredo != "y":
                print("Floating pose planning aborted.")
                return True
            else:
                print("Deleting existing floating  poses...")
                sql = "DELETE FROM floatingposes USING floatingposes, object WHERE floatingposes.idobject = object.idobject AND \
                        object.name LIKE '%s'" % self.dbobjname
                self.gdb.execute(sql)
                return False

    def __saveFPGToDB(self):
        """
        save the floatingposes and their grasps to the database

        :return:

        author: weiwei
        date: 20170221
        """

        print("Saving floating poses and grasps into the database...")

        sql = "SELECT * FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                        AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) == 0:
            # the gridsfloatingposes for the self.dbobjname is not saved
            sql = "INSERT INTO floatingposes(rotmat, idobject) VALUES "
            for i in range(len(self.gridsfloatingposemat4s)):
                # print i, "/", len(self.gridsfloatingposemat4s)
                sql += "('%s', (SELECT idobject FROM object WHERE name LIKE '%s')), " % \
                       (dc.mat4ToStr(self.gridsfloatingposemat4s[i]), self.dbobjname)
            sql = sql[:-2] + ";"
            self.gdb.execute(sql)

        sql = "SELECT * FROM floatinggrips,floatingposes,object,freeairgrip,hand WHERE \
                        floatinggrips.idfloatingposes = floatingposes.idfloatingposes AND \
                        floatingposes.idobject = object.idobject AND \
                        object.name LIKE '%s' AND floatinggrips.idfreeairgrip=freeairgrip.idfreeairgrip AND \
                        freeairgrip.idhand = hand.idhand AND hand.name LIKE '%s'" % (self.dbobjname, self.handpkg.getHandName())
        result = self.gdb.execute(sql)
        if len(result) == 0:
            tic = time.time()
            for i in range(len(self.gridsfloatingposemat4s)):
                toc = time.time()
                print("Saving FP and Gs to DB", "    Finished: ", i, "/", len(self.gridsfloatingposemat4s),
                      "    Time past: ", "%.2f" % (toc-tic), "s",
                      "    Expected remaining time: ", "%.2f" % ((toc-tic)*len(self.gridsfloatingposemat4s)/(i+1e-4)-(toc-tic)), "s")
                sql = "SELECT floatingposes.idfloatingposes FROM floatingposes,object WHERE \
                        floatingposes.idobject = object.idobject AND \
                        floatingposes.rotmat LIKE '%s' AND \
                        object.name LIKE '%s'" % (dc.mat4ToStr(self.gridsfloatingposemat4s[i]), self.dbobjname)
                result = self.gdb.execute(sql)[0]
                if len(result) != 0:
                    idfloatingposes = result[0]
                    sql = "SELECT * FROM floatinggrips WHERE idfloatingposes = %d" % idfloatingposes
                    result = self.gdb.execute(sql)
                    if len(result) == 0:
                        if len(self.floatinggripmat4s[i]) != 0:
                            sql = "INSERT INTO floatinggrips(contactpoint0, contactpoint1, contactnormal0, contactnormal1, \
                                    rotmat, jawwidth, idfloatingposes, idfreeairgrip) VALUES "
                            for j in range(len(self.floatinggripmat4s[i])):
                                # print "gripposes", i, "/", len(self.gridsfloatingposemat4s)
                                # print  "grips", j, "/", len(self.floatinggripmat4s[i])
                                cct0 = self.floatinggripcontacts[i][j][0]
                                cct1 = self.floatinggripcontacts[i][j][1]
                                cctn0 = self.floatinggripnormals[i][j][0]
                                cctn1 = self.floatinggripnormals[i][j][1]
                                sql += "('%s', '%s', '%s', '%s', '%s', '%s', %d, %d), " % \
                                       (dc.v3ToStr(cct0), dc.v3ToStr(cct1), dc.v3ToStr(cctn0), dc.v3ToStr(cctn1), \
                                        dc.mat4ToStr(self.floatinggripmat4s[i][j]), str(self.floatinggripjawwidth[i][j]), \
                                        idfloatingposes, self.floatinggripidfreeair[i][j])
                            sql = sql[:-2] + ";"
                            self.gdb.execute(sql)

    def __genHandPairs(self, base, loadser=False):
        self.handpairList = []
        if loadser is False:
            # hand0 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
            # hand1 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
            pairidlist = list(itertools.combinations(range(len(self.freegripids)), 2))
            total = len(pairidlist)/(len(pairidlist)/5000)+1
            tic = time.time()
            for i in range(100,len(pairidlist),int(len(pairidlist)/5000.0+1)):
                toc = time.time()
                print("Generating grasp pairs", "    Finished: ", "%.2f" % (i/int(len(pairidlist)/5000.0+1)/total*100), "%",
                      "    Time past: ", "%.2f" % (toc-tic), "s",
                      "    Expected remaining time: ", "%.2f" % ((toc-tic)*int(len(pairidlist)/5000.0+1)*total/(i+1e-4)-(toc-tic)), "s")
                i0, i1 = pairidlist[i]
                self.hand0.setMat(pandanpmat4 = self.freegriprotmats[i0])
                # self.hand0.setMat(npmat4=self.freegriprotmats[i0])
                self.hand0.setJawwidth(self.freegripjawwidth[i0])
                self.hand1.setMat(pandanpmat4 = self.freegriprotmats[i1])
                # self.hand1.setMat(npmat4=self.freegriprotmats[i1])
                self.hand1.setJawwidth(self.freegripjawwidth[i1])
                hndbullnodei0 = cd.genBulletCDMeshMultiNp(self.hand0.handnp, base.render)
                hndbullnodei1 = cd.genBulletCDMeshMultiNp(self.hand1.handnp, base.render)
                result = self.bulletworld.contactTestPair(hndbullnodei0, hndbullnodei1)
                if not result.getNumContacts():
                    # check the open state
                    self.hand0.setJawwidth(self.hand0.jawwidthopen)
                    self.hand1.setJawwidth(self.hand1.jawwidthopen)
                    hndbullnodei0 = cd.genBulletCDMeshMultiNp(self.hand0.handnp, base.render)
                    hndbullnodei1 = cd.genBulletCDMeshMultiNp(self.hand1.handnp, base.render)
                    result = self.bulletworld.contactTestPair(hndbullnodei0, hndbullnodei1)
                    if not result.getNumContacts():
                        self.handpairList.append([self.freegripids[i0], self.freegripids[i1]])
            json.dump(self.handpairList, open("tmp.json", mode="w"))
        else:
            self.handpairList = json.load(open("tmp.json", mode="r"))

    def __transformGrips(self):
        """
        transform the freeair grips to all rotmat4 in self.gridsfloatingposemat4s

        :return:

        author: weiwei
        date: 20170221, tsukuba
        """

        self.floatinggripmat4s = []
        self.floatinggripcontacts = []
        self.floatinggripnormals = []
        self.floatinggripjawwidth = []
        self.floatinggripidfreeair = []
        for icomat4 in self.gridsfloatingposemat4s:
            floatinggrips = self.transformGripsOnePose(icomat4)
            self.floatinggripmat4s.append(floatinggrips[0])
            self.floatinggripcontacts.append(floatinggrips[1])
            self.floatinggripnormals.append(floatinggrips[2])
            self.floatinggripjawwidth.append(floatinggrips[3])
            self.floatinggripidfreeair.append(floatinggrips[4])

    def __updateDBwithGP(self, loadser = False):
        """
        compute the floatinggrippairs using freegrippairs and
        save the floatinggripspairs into Database

        :param loadser whether use serialized data for handpairlist
        :return:

        author: weiwei
        date: 20170301
        """

        print("Start updating the floatingpose/grasps DB with floating grasp pairs!")

        if len(self.handpairList) == 0:
            self.__genHandPairs(base, loadser)
        tic = time.time()
        for fpid in range(len(self.gridsfloatingposemat4s)):
            toc = time.time()
            print("Saving grasp pairs", "    Finished: ", fpid, "/", len(self.gridsfloatingposemat4s),
                  "    Time past: ", "%.2f" % (toc-tic), "s",
                  "    Expected remaining time: ", "%.2f" % ((toc-tic)*len(self.gridsfloatingposemat4s)/(fpid+1e-4)-(toc-tic)), "s")
            # gen correspondence between freeairgripid and index
            # indfgoffa means index of floatinggrips whose freeairgripid are xxx
            indfgoffa = {}
            for i in range(len(self.floatinggripidfreeair[fpid])):
                indfgoffa[self.floatinggripidfreeair[fpid][i]] = i
            # handidpair_indfg is the pairs using index of floatinggrips
            handidpair_indfg = []
            sql = "INSERT INTO floatinggripspairs VALUES "
            for handidpair in self.handpairList:
                handidpair_indfg.append([indfgoffa[handidpair[0]], indfgoffa[handidpair[1]]])
                # if handidpair_indfg[0] is right, 1 is left
                sql = sql + "(%d, %d)" % \
                    (self.floatinggripids[fpid][handidpair_indfg[-1][0]], self.floatinggripids[fpid][handidpair_indfg[-1][1]])
                # if 1 is right, 0 is left
                sql = sql + ", (%d, %d), " % \
                      (self.floatinggripids[fpid][handidpair_indfg[-1][1]], self.floatinggripids[fpid][handidpair_indfg[-1][0]])
                # self.gdb.execute(sql)
                # # if 1 is right, 0 is left
                # sql = "INSERT INTO floatinggripspairs VALUES (%d, %d)" % \
                #         (self.floatinggripids[fpid][handidpair_indfg[-1][1]], self.floatinggripids[fpid][handidpair_indfg[-1][0]])
            sql = sql[:-2]
            self.gdb.execute(sql)

    def __checkIKexistance(self, idrobot):
        """
        check if the ik has been computed
        the existance is confirmed if the ik of the first idfloatinggrips has been updated

        :param idrobot:
        :return: boolean value indicating the existence of ik, True for existence, False for non-existence

        author: weiwei
        date: 20190318
        """

        # bexist = False
        # for fpid in range(len(self.gridsfloatingposemat4s)):
        #     print(fpid, len(self.gridsfloatingposemat4s))
        #     # right arm
        #     idarm = self.gdb.loadIdArm('rgt')
        #     for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
        #         sql = "SELECT * FROM ikfloatinggrips WHERE idrobot LIKE '%s' AND idarm LIKE '%s' \
        #                 AND idfloatinggrips LIKE '%s'" % (idrobot, idarm, idfloatinggrips)
        #         result = self.gdb.execute(sql)
        #         if(len(result)>0):
        #             bexist = True
        #             break
        #     if bexist:
        #         break
        #     # left arm
        #     idarm = self.gdb.loadIdArm('lft')
        #     for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
        #         sql = "SELECT * FROM ikfloatinggrips WHERE idrobot LIKE '%s' AND idarm LIKE '%s' \
        #                 AND idfloatinggrips LIKE '%s'"  % (idrobot, idarm, idfloatinggrips)
        #         result = self.gdb.execute(sql)
        #         if(len(result)>0):
        #             bexist = True
        #             break
        #     if bexist:
        #         break
        #
        # if bexist:
        #     print("IK is already updated!")
        #     isredo = input("Do you want to overwrite the database? (Y/N)")
        #     if isredo != "Y" and isredo != "y":
        #         print("Updating IK aborted.")
        #         return True
        #     else:
        #         for fpid in range(len(self.gridsfloatingposemat4s)):
        #             # right arm
        #             idarm = self.gdb.loadIdArm('rgt')
        #             for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
        #                 sql = "DELETE FROM ikfloatinggrips WHERE idrobot LIKE '%s' AND idarm LIKE '%s' \
        #                         AND idfloatinggrips LIKE '%s'" % (idrobot, idarm, idfloatinggrips)
        #                 self.gdb.execute(sql)
        #             # left arm
        #             idarm = self.gdb.loadIdArm('lft')
        #             for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
        #                 sql = "DELETE FROM ikfloatinggrips WHERE idrobot LIKE '%s' AND idarm LIKE '%s' \
        #                         AND idfloatinggrips LIKE '%s'" % (idrobot, idarm, idfloatinggrips)
        #                 self.gdb.execute(sql)
        #         return False
        # else:
        #     return False

        sql = "SELECT * FROM ikfloatinggrips WHERE idrobot LIKE '%s'" % (idrobot)
        result = self.gdb.execute(sql)
        if(len(result)>0):
            print("IK is already updated!")
            isredo = input("Do you want to overwrite the database? (Y/N)")
            if isredo != "Y" and isredo != "y":
                print("Updating IK aborted.")
                return True
            else:
                sql = "DELETE FROM ikfloatinggrips WHERE idrobot LIKE '%s'" % (idrobot)
                self.gdb.execute(sql)
                return False
        else:
            return False

    def __saveIKtoDB(self, idrobot):
        """
        saveupdated IK to DB
        this function is separated from updateDBwithIK for illustration

        :return:

        author: weiwei
        date: 20190316, toyonaka
        """

        for fpid in range(len(self.gridsfloatingposemat4s)):
            # right arm
            idarm = self.gdb.loadIdArm('rgt')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                sql = "INSERT INTO ikfloatinggrips(idrobot, idarm, idfloatinggrips, feasibility, \
                      feasibility_handa, jnts, jnts_handa) VALUES (%d, %d, %d, '%s', '%s', '%s', '%s')" \
                      % (idrobot, idarm, idfloatinggrips, self.feasibility_rgt[fpid][i],
                         self.feasibility_handa_rgt[fpid][i], self.jnts_rgt[fpid][i], self.jnts_handa_rgt[fpid][i])
                self.gdb.execute(sql)
            # left arm
            idarm = self.gdb.loadIdArm('lft')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                sql = "INSERT INTO ikfloatinggrips(idrobot, idarm, idfloatinggrips, feasibility, \
                      feasibility_handa, jnts, jnts_handa) VALUES (%d, %d, %d, '%s', '%s', '%s', '%s')" \
                      % (idrobot, idarm, idfloatinggrips, self.feasibility_lft[fpid][i],
                         self.feasibility_handa_lft[fpid][i], self.jnts_lft[fpid][i], self.jnts_handa_lft[fpid][i])
                self.gdb.execute(sql)

    def genFPGPandSaveToDB(self, grids, icolevel=1, angles=[0,45,90,135,180,225,270,315]):
        """
        genterate floating poses and their grasps and the floatinggrippairs,
        this function leverages genPandaRotmat4 and transformGrips

        :param icolevel
        :param angles see genPandaRotmat4
        :return:

        author: weiwei
        date: 20170221
        """

        if self.__checkFPexistance():
            pass
        else:
            self.gridsfloatingposemat4s = []
            self.__genPandaRotmat4(icolevel, angles)
            for gridposition in grids:
                for posemat4 in self.floatingposemat4:
                    tmpposemat4 = Mat4(posemat4)
                    tmpposemat4.setRow(3, Vec3(gridposition[0], gridposition[1], gridposition[2]))
                    self.gridsfloatingposemat4s.append(tmpposemat4)
            self.__transformGrips()
            print("Saving the floatingposes and grasps to database")
            self.__saveFPGToDB()
            print("Updating the databse with floating grasppairs")
            self.loadFPGfromDB() # update floatinggripids
            self.__updateDBwithGP()

    def updateDBwithIK(self, robot):
        """
        compute the IK feasible grasps of each pose

        :return:

        author: weiwei
        date: 20190316
        """

        # load retraction distances
        rethanda, retworlda, worldz = self.gdb.loadIKRet()
        # load idrobot
        idrobot = self.gdb.loadIdRobot(robot)

        if self.__checkIKexistance(idrobot):
            pass
        else:
            self.feasibility_rgt = []
            self.feasibility_handa_rgt = []
            self.feasibility_lft = []
            self.feasibility_handa_lft = []
            self.jnts_rgt = []
            self.jnts_handa_rgt = []
            self.jnts_lft = []
            self.jnts_handa_lft = []
            tic = time.time()
            for fpid in range(len(self.gridsfloatingposemat4s)):
                toc = time.time()
                print("Updating IK", "    Finished: ", fpid, "/", len(self.gridsfloatingposemat4s),
                      "    Time past: ", "%.2f" % (toc-tic), "s",
                      "    Expected remaining time: ", "%.2f" % ((toc-tic)*len(self.gridsfloatingposemat4s)/(fpid+1e-4)-(toc-tic)), "s")
                ### right hand
                armid = 'rgt'
                feasibility = []
                feasibility_handa = []
                jnts = []
                jnts_handa = []
                for i, hndrotmat4 in enumerate(self.floatinggripmat4s[fpid]):
                    feasibility.append('False')
                    feasibility_handa.append('False')
                    jnts.append([])
                    jnts_handa.append([])
                    fpgsfgrcenter = (self.floatinggripcontacts[fpid][i][0]+self.floatinggripcontacts[fpid][i][1])/2
                    fpgsfgrcenternp = self.__base.pg.v3ToNp(fpgsfgrcenter)
                    fpgsrotmat3np = self.__base.pg.mat3ToNp(hndrotmat4.getUpper3())
                    handa =  -hndrotmat4.getRow3(2)
                    # check the angle between handa and minus y
                    minusworldy = Vec3(0,-1,0)
                    if Vec3(handa).angleDeg(minusworldy) < 60:
                        msc = robot.numik(fpgsfgrcenternp, fpgsrotmat3np, armid)
                        if msc is not None:
                            feasibility[-1] = 'True'
                            jnts[-1] = dc.listToStr(msc)
                            fpgsfgrcenternp_handa = self.__base.pg.v3ToNp(fpgsfgrcenter+handa*rethanda)
                            msc_handa = robot.numikmsc(fpgsfgrcenternp_handa, fpgsrotmat3np, msc, armid)
                            if msc_handa is not None:
                                feasibility_handa[-1] = 'True'
                                jnts_handa[-1] = dc.listToStr(msc_handa)
                self.feasibility_rgt.append(feasibility)
                self.feasibility_handa_rgt.append(feasibility_handa)
                self.jnts_rgt.append(jnts)
                self.jnts_handa_rgt.append(jnts_handa)
                ### left hand
                armid = 'lft'
                feasibility = []
                feasibility_handa = []
                jnts = []
                jnts_handa = []
                for i, hndrotmat4 in enumerate(self.floatinggripmat4s[fpid]):
                    feasibility.append('False')
                    feasibility_handa.append('False')
                    jnts.append([])
                    jnts_handa.append([])
                    fpgsfgrcenter = (self.floatinggripcontacts[fpid][i][0]+self.floatinggripcontacts[fpid][i][1])/2
                    fpgsfgrcenternp = self.__base.pg.v3ToNp(fpgsfgrcenter)
                    fpgsrotmat3np = self.__base.pg.mat3ToNp(hndrotmat4.getUpper3())
                    handa =  -hndrotmat4.getRow3(2)
                    # check the angle between handa and minus y
                    plusworldy = Vec3(0,1,0)
                    if Vec3(handa).angleDeg(plusworldy) < 60:
                        msc = robot.numik(fpgsfgrcenternp, fpgsrotmat3np, armid)
                        if msc is not None:
                            feasibility[-1] = 'True'
                            jnts[-1] = dc.listToStr(msc)
                            fpgsfgrcenternp_handa = self.__base.pg.v3ToNp(fpgsfgrcenter+handa*rethanda)
                            msc_handa = robot.numikmsc(fpgsfgrcenternp_handa, fpgsrotmat3np, msc, armid)
                            if msc_handa is not None:
                                feasibility_handa[-1] = 'True'
                                jnts_handa[-1] = dc.listToStr(msc_handa)
                self.feasibility_lft.append(feasibility)
                self.feasibility_handa_lft.append(feasibility_handa)
                self.jnts_lft.append(jnts)
                self.jnts_handa_lft.append(jnts_handa)

            self.__saveIKtoDB(idrobot)
            print("IK is updated!")

    def loadFPGfromDB(self):
        """
        for debug purpose
        load the floatingposes and their grasps from the database

        :return:

        author: weiwei
        date: 20170227
        """

        print("Loading FP and Gs to update floatinggripids...")

        self.gridsfloatingposemat4s = []
        self.floatinggripids = []
        self.floatinggripmat4s = []
        self.floatinggripcontacts = []
        self.floatinggripnormals = []
        self.floatinggripjawwidth = []
        self.floatinggripidfreeair = []
        sql = "SELECT * FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                        AND object.name LIKE '%s'" % self.dbobjname
        fposesresult = self.gdb.execute(sql)
        if len(fposesresult) != 0:
            tic = time.time()
            for i, resultrow in enumerate(fposesresult):
                toc = time.time()
                print("Loading floatinggripids", "    Finished: ", i, "/", len(fposesresult),
                      "    Time past: ", "%.2f" % (toc-tic), "s",
                      "    Expected remaining time: ", "%.2f" % ((toc-tic)*len(fposesresult)/(i+1e-4)-(toc-tic)), "s")
                self.gridsfloatingposemat4s.append(dc.strToMat4(resultrow[1]))
                idfloatingposes = resultrow[0]
                sql = "SELECT floatinggrips.idfloatinggrips, floatinggrips.contactpoint0, \
                        floatinggrips.contactpoint1, floatinggrips.contactnormal0, \
                        floatinggrips.contactnormal1, floatinggrips.rotmat, floatinggrips.jawwidth, \
                        floatinggrips.idfreeairgrip FROM floatinggrips,freeairgrip,hand WHERE \
                        floatinggrips.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                        freeairgrip.idhand = hand.idhand AND floatinggrips.idfloatingposes = %d AND \
                        hand.name = '%s'" % (idfloatingposes, self.handpkg.getHandName())
                fgresult = self.gdb.execute(sql)
                if len(fgresult) != 0:
                    floatinggripids = []
                    floatinggripmat4s = []
                    floatinggripcontacts = []
                    floatinggripnormals = []
                    floatinggripjawwidths = []
                    floatinggripidfreeairs = []
                    for fgresultrow in fgresult:
                        cct0 = dc.strToV3(fgresultrow[1])
                        cct1 = dc.strToV3(fgresultrow[2])
                        cctn0 = dc.strToV3(fgresultrow[3])
                        cctn1 = dc.strToV3(fgresultrow[4])
                        floatinggripids.append(int(fgresultrow[0]))
                        floatinggripmat4s.append(dc.strToMat4(fgresultrow[5]))
                        floatinggripcontacts.append([cct0, cct1])
                        floatinggripnormals.append([cctn0, cctn1])
                        floatinggripjawwidths.append(float(fgresultrow[6]))
                        floatinggripidfreeairs.append(int(fgresultrow[7]))
                    self.floatinggripids.append(floatinggripids)
                    self.floatinggripmat4s.append(floatinggripmat4s)
                    self.floatinggripcontacts.append(floatinggripcontacts)
                    self.floatinggripnormals.append(floatinggripnormals)
                    self.floatinggripjawwidth.append(floatinggripjawwidths)
                    self.floatinggripidfreeair.append(floatinggripidfreeairs)
                else:
                    print('Plan floating grips first!')
                    assert(False)
        else:
            assert('No object found!')

    def transformGripsOnePose(self, rotmat4):
        """
        transform the freeair grips to one specific rotmat4

        :param rotmat4:
        :return: [floatinggripmat4s, floatinggripcontacts, floatinggripnormals, floatinggripjawwidths, floatinggripidfreeairs]
        each element in the list is also a list
        """

        floatinggripmat4s = []
        floatinggripcontacts = []
        floatinggripnormals = []
        floatinggripjawwidths = []
        floatinggripidfreeairs = []
        for i, gripmat4 in enumerate(self.freegriprotmats):
            floatinggripmat4 = gripmat4 * rotmat4
            cct0 = rotmat4.xformPoint(self.freegripcontacts[i][0])
            cct1 = rotmat4.xformPoint(self.freegripcontacts[i][1])
            cctn0 = rotmat4.xformPoint(self.freegripnormals[i][0])
            cctn1 = rotmat4.xformPoint(self.freegripnormals[i][1])
            floatinggripjawwidth = self.freegripjawwidth[i]
            floatinggripidfreeair = self.freegripids[i]
            floatinggripmat4s.append(floatinggripmat4)
            floatinggripcontacts.append([cct0, cct1])
            floatinggripnormals.append([cctn0, cctn1])
            floatinggripjawwidths.append(floatinggripjawwidth)
            floatinggripidfreeairs.append(floatinggripidfreeair)
        return [floatinggripmat4s, floatinggripcontacts, floatinggripnormals,
                floatinggripjawwidths, floatinggripidfreeairs]

    def showIcomat4s(self, nodepath):
        """
        show the pandamat4s generated by genPandaRotmat4

        :param nodepath, where np to repreantTo, usually base.render
        :return:

        author: weiwei
        date: 20170221, tsukuba
        """

        for i, icomat4list in enumerate(self.mat4list):
            vert = self.icos.vertices*100
            spos = Vec3(vert[i][0], vert[i][1], vert[i][2])
            for icomat4 in icomat4list:
                self.__base.pg.plotAxis(nodepath, spos, icomat4, length=100, thickness=3)

    def loadIKfeasibleGPfromDB(self, robot):
        """
        load the IK Feasible handover pairs
        :return:

        author: weiwei
        date: 20170301
        """

        self.loadFPGfromDB()
        self.loadIKFromDB(robot)

        idrobot = self.gdb.loadIdRobot(robot)
        idarmrgt = self.gdb.loadIdArm('rgt')
        idarmlft = self.gdb.loadIdArm('lft')

        self.floatinggrippairsids = []
        self.floatinggrippairshndmat4s = []
        self.floatinggrippairscontacts = []
        self.floatinggrippairsnormals = []
        self.floatinggrippairsjawwidths = []
        self.floatinggrippairsidfreeairs = []
        self.floatinggrippairsjnts = []
        self.floatinggrippairsjnts_handa = []
        for fpid in range(len(self.gridsfloatingposemat4s)):
            sql = "SELECT floatingposes.idfloatingposes FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                            AND object.name LIKE '%s' AND floatingposes.rotmat LIKE '%s'" % \
                  (self.dbobjname, dc.mat4ToStr(self.gridsfloatingposemat4s[fpid]))
            result = self.gdb.execute(sql)
            floatinggrippairsids = []
            floatinggrippairshndmat4s = []
            floatinggrippairscontacts = []
            floatinggrippairsnormals = []
            floatinggrippairsjawwidths = []
            floatinggrippairsidfreeairs = []
            floatinggrippairsjnts = []
            floatinggrippairsjnts_handa = []
            if len(result) != 0:
                idfloatingposes = result[0][0]
                sql = "SELECT floatinggripspairs.idfloatinggrips0, floatinggripspairs.idfloatinggrips1, \
                        fg0.contactpoint0, fg0.contactpoint1, fg0.contactnormal0, fg0.contactnormal1, fg0.rotmat, \
                        fg0.jawwidth, fg0.idfreeairgrip, \
                        fg1.contactpoint0, fg1.contactpoint1, fg1.contactnormal0, fg1.contactnormal1, fg1.rotmat, \
                        fg1.jawwidth, fg1.idfreeairgrip, ikfg0.jnts, ikfg0.jnts_handa, \
                        ikfg1.jnts, ikfg1.jnts_handa FROM floatinggripspairs, floatinggrips fg0, floatinggrips fg1, \
                        ikfloatinggrips ikfg0, ikfloatinggrips ikfg1  WHERE \
                        floatinggripspairs.idfloatinggrips0 = fg0.idfloatinggrips AND \
                        floatinggripspairs.idfloatinggrips1 = fg1.idfloatinggrips AND \
                        fg0.idfloatingposes = %d AND fg1.idfloatingposes = %d AND \
                        fg0.idfloatinggrips = ikfg0.idfloatinggrips AND \
                        ikfg0.feasibility like 'True' AND ikfg0.feasibility_handa like 'True' AND \
                        ikfg0.idrobot = %d AND ikfg0.idarm = %d AND \
                        fg1.idfloatinggrips = ikfg1.idfloatinggrips AND ikfg1.feasibility like 'True' \
                        AND ikfg1.feasibility_handa like 'True' AND \
                        ikfg1.idrobot = %d AND ikfg1.idarm = %d" % \
                      (idfloatingposes, idfloatingposes, idrobot, idarmrgt, idrobot, idarmlft)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    for resultrow in result:
                        floatinggrippairsids.append([resultrow[0], resultrow[1]])
                        floatinggrippairshndmat4s.append([dc.strToMat4(resultrow[6]), dc.strToMat4(resultrow[13])])
                        rgtcct0 = dc.strToV3(resultrow[2])
                        rgtcct1 = dc.strToV3(resultrow[3])
                        lftcct0 = dc.strToV3(resultrow[9])
                        lftcct1 = dc.strToV3(resultrow[10])
                        floatinggrippairscontacts.append([[rgtcct0, rgtcct1], [lftcct0, lftcct1]])
                        rgtcctn0 = dc.strToV3(resultrow[4])
                        rgtcctn1 = dc.strToV3(resultrow[5])
                        lftcctn0 = dc.strToV3(resultrow[11])
                        lftcctn1 = dc.strToV3(resultrow[12])
                        floatinggrippairsnormals.append([[rgtcctn0, rgtcctn1], [lftcctn0, lftcctn1]])
                        floatinggrippairsjawwidths.append([float(resultrow[7]), float(resultrow[14])])
                        floatinggrippairsidfreeairs.append([int(resultrow[8]), int(resultrow[15])])
                        floatinggrippairsjnts.append([dc.strToList(resultrow[16]), dc.strToList(resultrow[18])])
                        floatinggrippairsjnts_handa.append([dc.strToList(resultrow[17]), dc.strToList(resultrow[19])])
            self.floatinggrippairsids.append(floatinggrippairsids)
            self.floatinggrippairshndmat4s.append(floatinggrippairshndmat4s)
            self.floatinggrippairscontacts.append(floatinggrippairscontacts)
            self.floatinggrippairsnormals.append(floatinggrippairsnormals)
            self.floatinggrippairsjawwidths.append(floatinggrippairsjawwidths)
            self.floatinggrippairsidfreeairs.append(floatinggrippairsidfreeairs)
            self.floatinggrippairsjnts.append(floatinggrippairsjnts)
            self.floatinggrippairsjnts_handa.append(floatinggrippairsjnts_handa)

    def loadIKFromDB(self, robot):
        """
        load the feasibility of IK from db

        :param robot:
        :return:

        author: weiwei
        date: 20170301
        """

        idrobot = self.gdb.loadIdRobot(robot)

        self.feasibility_rgt = []
        self.feasibility_handa_rgt = []
        self.feasibility_lft = []
        self.feasibility_handa_lft = []
        for fpid in range(len(self.gridsfloatingposemat4s)):
            # right arm
            feasibility = []
            feasibility_handa = []
            idarm = self.gdb.loadIdArm('rgt')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                feasibility.append('False')
                feasibility_handa.append('False')
                sql = "SELECT feasibility, feasibility_handa FROM ikfloatinggrips WHERE \
                        idrobot = %d AND idarm = %d and idfloatinggrips = %d" % (idrobot, idarm, idfloatinggrips)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    feasibility[i] = result[0][0]
                    feasibility_handa[i] = result[0][1]
            self.feasibility_rgt.append(feasibility)
            self.feasibility_handa_rgt.append(feasibility_handa)
            # left arm
            feasibility = []
            feasibility_handa = []
            idarm = self.gdb.loadIdArm('lft')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                feasibility.append('False')
                feasibility_handa.append('False')
                sql = "SELECT feasibility, feasibility_handa FROM ikfloatinggrips WHERE \
                        idrobot = %d AND idarm = %d and idfloatinggrips = %d" % (idrobot, idarm, idfloatinggrips)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    feasibility[i] = result[0][0]
                    feasibility_handa[i] = result[0][1]
            self.feasibility_lft.append(feasibility)
            self.feasibility_handa_lft.append(feasibility_handa)

    def plotOneFPandG(self, parentnp, fpid=0):
        """
        plot the objpose and grasps at a specific rotmat4

        :param fpid: the index of self.floatingposemat4
        :return:

        author: weiwei
        date: 20170221, tsukuba
        """

        objnp = self.__base.pg.packpandanp_fn(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        objnp.setMat(self.gridsfloatingposemat4s[fpid])
        objnp.reparentTo(parentnp)
        for i, hndrotmat4 in enumerate(self.gridsfloatingposemat4s[fpid]):
            if i == 7:
                # show grasps
                hand0 = self.handpkg.newHandNM(hndcolor=[0, 1, 0, 1])
                hand0.setMat(pandanpmat4 = hndrotmat4)
                hand0.setJawwidth(self.floatinggripjawwidth[fpid][i])
                hand0.reparentTo(parentnp)
                print(self.handpairList)
                for handidpair in self.handpairList:
                    if handidpair[0] == self.floatinggripidfreeair[fpid][i]:
                        pairedilist = [i1 for i1 in range(len(self.floatinggripidfreeair[fpid]))
                                       if self.floatinggripidfreeair[fpid][i1]==handidpair[1]]
                        print(pairedilist)
                        i1 = pairedilist[0]
                        # if self.feasibility_lft[fpid][i1] == 'True':
                        hand1 = self.handpkg.newHandNM(hndcolor=[0, 1, 1, 1])
                        hndrotmat4 = self.floatinggripmat4s[fpid][i1]
                        hand1.setMat(pandanpmat4 = hndrotmat4)
                        hand1.setJawwidth(self.floatinggripjawwidth[fpid][i1])
                        hand1.reparentTo(parentnp)
                    if handidpair[1] == self.floatinggripidfreeair[fpid][i]:
                        pairedilist = [i1 for i1 in range(len(self.floatinggripidfreeair[fpid]))
                                       if self.floatinggripidfreeair[fpid][i1]==handidpair[0]]
                        print(pairedilist)
                        i1 = pairedilist[0]
                        # if self.feasibility_lft[fpid][i1] == 'True':
                        hand1 = self.handpkg.newHandNM(hndcolor=[0, 1, 1, 1])
                        hndrotmat4 = self.floatinggripmat4s[fpid][i1]
                        hand1.setMat(pandanpmat4 = hndrotmat4)
                        hand1.setJawwidth(self.floatinggripjawwidth[fpid][i1])
                        hand1.reparentTo(parentnp)

    def plotOneFPandGPairs(self, parentnp, fpid=0):
        """
        plot the gpairss at a specific rotmat4

        :param fpid: the index of self.floatingposemat4
        :return:

        author: weiwei
        date: 20170301, tsukuba
        """

        objnp = self.__base.pg.packpandanp_fn(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        objnp.setMat(self.gridsfloatingposemat4s[fpid])
        objnp.setColor(Vec4(.7,0.3,0,1))
        objnp.reparentTo(parentnp)
        print(self.floatinggrippairshndmat4s[fpid])
        for i, hndrotmat4pair in enumerate(self.floatinggrippairshndmat4s[fpid]):
            # if i == 9:
            # show grasps
            hand0 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .5])
            hand0mat4 = Mat4(hndrotmat4pair[0])
            # h0row3 = hand0mat4.getRow3(3)
            # h0row3[2] = h0row3[2]+i*20.0
            # hand0mat4.setRow(3, h0row3[2])
            hand0.setMat(pandanpmat4 = hand0mat4)
            hand0.setJawwidth(self.floatinggrippairsjawwidths[fpid][i][0])
            hand0.reparentTo(parentnp)
            hand1 = self.handpkg.newHandNM(hndcolor=[0, .0, 1, .5])
            hand1mat4 = Mat4(hndrotmat4pair[1])
            # h1row3 = hand1mat4.getRow3(3)
            # h1row3[2] = h1row3[2]+i*20.0
            # hand1mat4.setRow(3, h1row3[2])
            hand1.setMat(pandanpmat4 = hand1mat4)
            hand1.setJawwidth(self.floatinggrippairsjawwidths[fpid][i][1])
            hand1.reparentTo(parentnp)

if __name__ == '__main__':
    icos = trimesh.creation.icosphere(1)
    icos.vertices=icos.vertices*500
    import pandaplotutils.pandactrl as pandactrl
    base = pandactrl.World()
    base.pg.trimesh2Panda(icos).reparentTo(base.render)
    base.run()
