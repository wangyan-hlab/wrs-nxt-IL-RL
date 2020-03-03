import os

import numpy as np
from manipulation.grip.hrp5pf3 import hrp5pf3
from panda3d.core import *
import copy
import math
import trimesh

import pandaplotutils.pandageom as pg

class Hrp5PBall(object):
    """
    generate hrp5pballs for quick collision detection
    """

    def __init__(self):
        """
        load models

        author: weiwei
        date: 20170612
        """

        self.__shownp = None

    def __genbcn_direct(self, linklist, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link
        the balls are generated along the direction of the links

        :param linkid:
        :param armid:
        :return:
        """

        radius = float(radius)
        balldist = radius

        tmpcolnode = CollisionNode(name)
        for link in linklist:
            spos = link['linkpos']
            epos = link['linkend']
            linklength = np.linalg.norm(epos - spos)
            linkvec = (epos - spos)/linklength
            nball = int(math.ceil(linklength / balldist))
            for i in range(1, nball):
                pos = spos+linkvec*i*(balldist)
                # if i == nball-1:
                #     pos = spos+linkvec*(i-.4)*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def __genbcn_manhattan(self, linklist, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link
        the balls are generated along the manhattan directions between joints

        :param linkid:
        :param armid:
        :return:
        """

        radius = float(radius)
        balldist = radius

        tmpcolnode = CollisionNode(name)
        for link, divdir in linklist:
            spos = link['linkpos']
            epos = link['linkend']
            # linkvec direction
            linkdir = np.dot((epos-spos), divdir)*divdir
            linkdirlength = np.linalg.norm(linkdir)
            linkdirnormalized = linkdir/linkdirlength
            nball = int(math.ceil(linkdirlength / balldist))
            for i in range(0, nball):
                pos = spos+linkdirnormalized*i*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
            # linkvec orthogonal direction
            linkorth = epos-spos-linkdir
            linkorthlength = np.linalg.norm(linkorth)
            linkorthnormalized = linkorth/linkorthlength
            nball = int(math.ceil(linkorthlength / balldist))
            for i in range(0, nball):
                pos = spos+linkdir+linkorthnormalized*i*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def __genbcn_ext(self, tmpcolnode, spos, epos, radius=70.0, name = "autogen"):
        """
        generate extra ball collision node for non-link decorations
        the balls follow a line connecting spos and epos

        :param tmpcolnode: the generated cd balls will be added to the tmpcolnode
        :param spos:
        :param epos:
        :param radius:
        :param name:
        :return:

        author: weiwei
        date: 20180420
        """

        balldist = radius

        linklength = np.linalg.norm(epos - spos)
        linkvec = (epos - spos)/linklength
        nball = int(math.ceil(linklength / balldist))
        for i in range(0, nball):
            pos = spos+linkvec*i*(balldist)
            colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
            tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def genbcndict(self, robot, jawwidthrgt = 120.0, jawwidthlft = 120.0, objhold = None):
        """
        generate the ball collision nodes of a robot

        :param robot: the ur5sgl object, see ur5sgl.py
        :param objhold: [length, radius, armid]
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        # link0r = robot.rgtarm[0]
        # divdir0r = robot.rgtarm[0]['rotmat'][:,2]
        # link0l = robot.lftarm[0]
        # divdir0l = robot.lftarm[0]['rotmat'][:,2]
        # leglink3r = robot.rgtleg[3]
        # legdivdir3r = robot.rgtleg[3]['rotmat'][:,2]
        # leglink3l = robot.lftleg[3]
        # legdivdir3l = robot.lftleg[3]['rotmat'][:,2]
        # bodybcn = self.__genbcn_manhattan([[link0r, divdir0r], [leglink3r, legdivdir3r],
        #                                    [leglink3l, legdivdir3l]], radius = 180)
        # returnlist['body'] = bodybcn

        # body
        radius = 180.0
        tmpcolnode = CollisionNode("cdbody")
        # body
        bstart = robot.rgtarm[0]['linkpos']
        bend= robot.rgtarm[0]['linkpos']+robot.rgtarm[0]['rotmat'][:,2]*1000.0
        self.__genbcn_ext(tmpcolnode, bstart, bend, radius=radius)
        # head
        bstart = robot.rgtarm[0]['linkpos']+robot.rgtarm[0]['rotmat'][:,2]*800.0
        bend = robot.rgtarm[0]['linkpos']+robot.rgtarm[0]['rotmat'][:,2]*800.0+robot.rgtarm[0]['rotmat'][:,0]*300.0
        self.__genbcn_ext(tmpcolnode, bstart, bend, radius=radius)
        # shoulders
        rshstart = robot.rgtarm[2]['linkpos']
        rshend= robot.rgtarm[2]['linkpos']+robot.rgtarm[2]['rotmat'][:,2]*10.0
        self.__genbcn_ext(tmpcolnode, rshstart, rshend, radius=radius*5.0/9.0)
        lshstart = robot.lftarm[2]['linkpos']
        lshend= robot.lftarm[2]['linkpos']+robot.lftarm[2]['rotmat'][:,2]*10.0
        bodybcn = self.__genbcn_ext(tmpcolnode, lshstart, lshend, radius=radius*5.0/9.0)
        returnlist['body'] = bodybcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:,2]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,0]
        link8r = robot.rgtarm[8]
        divdir8r = robot.rgtarm[8]['rotmat'][:,2]
        link9r = robot.rgtarm[9]
        divdir9r = robot.rgtarm[9]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r], [link9r, divdir9r]], radius = 80)
        offset = 70.0
        radius = 30.0
        stip = robot.rgtarm[-1]['linkend']+robot.rgtarm[-1]['rotmat'][:,1]*offset+robot.rgtarm[-1]['rotmat'][:,2]*10
        etip = robot.rgtarm[-1]['linkend']+robot.rgtarm[-1]['rotmat'][:,1]*offset-robot.rgtarm[-1]['rotmat'][:,2]*10
        rgthandbcn = self.__genbcn_ext(rgthandbcn, stip, etip, radius=radius)
        stip = robot.rgtarm[-1]['linkend']-robot.rgtarm[-1]['rotmat'][:,1]*offset+robot.rgtarm[-1]['rotmat'][:,2]*40
        etip = robot.rgtarm[-1]['linkend']-robot.rgtarm[-1]['rotmat'][:,1]*offset-robot.rgtarm[-1]['rotmat'][:,2]*40
        rgthandbcn = self.__genbcn_ext(rgthandbcn, stip, etip, radius=radius)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:,2]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,0]
        link8l = robot.lftarm[8]
        divdir8l = robot.lftarm[8]['rotmat'][:,2]
        link9l = robot.lftarm[9]
        divdir9l = robot.lftarm[9]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l], [link9l, divdir9l]], radius = 80)
        offset = 70.0
        radius = 30.0
        stip = robot.lftarm[-1]['linkend']+robot.lftarm[-1]['rotmat'][:,1]*offset+robot.lftarm[-1]['rotmat'][:,2]*40
        etip = robot.lftarm[-1]['linkend']+robot.lftarm[-1]['rotmat'][:,1]*offset-robot.lftarm[-1]['rotmat'][:,2]*40
        lfthandbcn = self.__genbcn_ext(lfthandbcn, stip, etip, radius=radius)
        stip = robot.lftarm[-1]['linkend']-robot.lftarm[-1]['rotmat'][:,1]*offset+robot.lftarm[-1]['rotmat'][:,2]*10
        etip = robot.lftarm[-1]['linkend']-robot.lftarm[-1]['rotmat'][:,1]*offset-robot.lftarm[-1]['rotmat'][:,2]*10
        lfthandbcn = self.__genbcn_ext(lfthandbcn, stip, etip, radius=radius)
        returnlist['lfthnd'] = lfthandbcn

        # objhold
        if objhold is not None:
            tmpcolnode = CollisionNode("cdbody")
            hand = robot.rgtarm[-1]
            if objhold[2] == 'lft':
                hand = robot.lftarm[-1]
            objstart = hand['linkend']-hand['rotmat'][:, 2]*objhold[0]
            objend = hand['linkend']+hand['rotmat'][:, 2]*objhold[0]
            returnlist['object'] = self.__genbcn_ext(tmpcolnode, objstart, objend, radius=objhold[1])

        return returnlist

    def genbcndicthold(self, robot, objrelrot = np.eye(3), objrelpos = np.array([0,0,0]), objtrimesh = None, armid='rgt'):
        """
        # NOTE this example cd is not well implemented 20180702, tsukuba
        generate the ball collision nodes of a robot, with an object held in hand

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        # link0r = robot.rgtarm[0]
        # divdir0r = robot.rgtarm[0]['rotmat'][:,2]
        # link0l = robot.lftarm[0]
        # divdir0l = robot.lftarm[0]['rotmat'][:,2]
        # leglink3r = robot.rgtleg[3]
        # legdivdir3r = robot.rgtleg[3]['rotmat'][:,2]
        # leglink3l = robot.lftleg[3]
        # legdivdir3l = robot.lftleg[3]['rotmat'][:,2]
        # bodybcn = self.__genbcn_manhattan([[link0r, divdir0r], [leglink3r, legdivdir3r],
        #                                    [leglink3l, legdivdir3l]], radius = 180)
        # returnlist['body'] = bodybcn

        # body
        radius = 180.0
        tmpcolnode = CollisionNode("cdbody")
        # body
        bstart = robot.rgtarm[0]['linkpos']
        bend= robot.rgtarm[0]['linkpos']+robot.rgtarm[0]['rotmat'][:,2]*1000.0
        self.__genbcn_ext(tmpcolnode, bstart, bend, radius=radius)
        # head
        bstart = robot.rgtarm[0]['linkpos']+robot.rgtarm[0]['rotmat'][:,2]*800.0
        bend = robot.rgtarm[0]['linkpos']+robot.rgtarm[0]['rotmat'][:,2]*800.0+robot.rgtarm[0]['rotmat'][:,0]*300.0
        self.__genbcn_ext(tmpcolnode, bstart, bend, radius=radius)
        # shoulders
        rshstart = robot.rgtarm[2]['linkpos']
        rshend= robot.rgtarm[2]['linkpos']+robot.rgtarm[2]['rotmat'][:,2]*10.0
        self.__genbcn_ext(tmpcolnode, rshstart, rshend, radius=radius*5.0/9.0)
        lshstart = robot.lftarm[2]['linkpos']
        lshend= robot.lftarm[2]['linkpos']+robot.lftarm[2]['rotmat'][:,2]*10.0
        bodybcn = self.__genbcn_ext(tmpcolnode, lshstart, lshend, radius=radius*5.0/9.0)
        returnlist['body'] = bodybcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:,2]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,0]
        link8r = robot.rgtarm[8]
        divdir8r = robot.rgtarm[8]['rotmat'][:,2]
        link9r = robot.rgtarm[9]
        divdir9r = robot.rgtarm[9]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r], [link9r, divdir9r]], radius = 100)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:,2]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,0]
        link8l = robot.lftarm[8]
        divdir8l = robot.lftarm[8]['rotmat'][:,2]
        link9l = robot.lftarm[9]
        divdir9l = robot.lftarm[9]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l], [link9l, divdir9l]], radius = 100)
        returnlist['lfthnd'] = lfthandbcn

        # obj
        objcn = CollisionNode("object")
        baserot = robot.rgtarm[-1]['rotmat']
        basepos = robot.rgtarm[-1]['linkend']
        if armid == 'lft':
            baserot = robot.lftarm[-1]['rotmat']
            basepos = robot.lftarm[-1]['linkend']
        objrot = np.dot(objrelrot, baserot)
        objpos = basepos+objrelpos
        objmat4 = np.eye(4)
        objmat4[:3,:3] = objrot
        objmat4[:3,3] = objpos
        objtrimesh.apply_transform(objmat4)
        centers = trimesh.sample.sample_surface_even(objtrimesh, 20)
        for rowid in range(centers.shape[0]):
            center = centers[rowid][:]
            colsphere = CollisionSphere(center[0], center[1], center[2], 30)
            objcn.addSolid(colsphere)
        returnlist['object'] = objcn

        return returnlist

    def genfullactivebcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        Variation: both arms and hands are considered

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        # returnlist = {}
        #
        # # arm
        # link2r = robot.rgtarm[2]
        # divdir2r = robot.rgtarm[2]['rotmat'][:, 0]
        # link2l = robot.lftarm[2]
        # divdir2l = robot.lftarm[2]['rotmat'][:, 0]
        # armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
        #                                   [link2l, divdir2l]], radius=60)
        # link3r = robot.rgtarm[3]
        # spos = link3r['linkpos']
        # epos = link3r['linkend']
        # sposnew = spos
        # eposnew = (spos + epos) / 2.0
        # self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        # link3l = robot.lftarm[3]
        # spos = link3l['linkpos']
        # epos = link3l['linkend']
        # sposnew = spos
        # eposnew = (spos + epos) / 2.0
        # armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        # returnlist['arm'] = armbcn
        #
        # # rgt hand
        # link4r = robot.rgtarm[4]
        # divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        # link5r = robot.rgtarm[5]
        # divdir5r = robot.rgtarm[5]['rotmat'][:, 2]
        # link6r = robot.rgtarm[6]
        # divdir6r = robot.rgtarm[6]['rotmat'][:, 2]
        # rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]], radius=60)
        # returnlist['rgthnd'] = rgthandbcn
        #
        # # lft hand
        # link4l = robot.lftarm[4]
        # divdir4l = robot.lftarm[4]['rotmat'][:, 1]
        # link5l = robot.lftarm[5]
        # divdir5l = robot.lftarm[5]['rotmat'][:, 2]
        # link6l = robot.lftarm[6]
        # divdir6l = robot.lftarm[6]['rotmat'][:, 2]
        # lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l], [link6l, divdir6l]], radius=60)
        # returnlist['lfthnd'] = lfthandbcn

        # My modifications for the HRP5P

        returnlist = {}

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:, 0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:, 0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                          [link2l, divdir2l]], radius=100)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius=100)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius=100)
        returnlist['arm'] = armbcn

        # rgt hand
        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:, 0]
        link8r = robot.rgtarm[8]
        divdir8r = robot.rgtarm[8]['rotmat'][:, 2]
        link9r = robot.rgtarm[9]
        divdir9r = robot.rgtarm[9]['rotmat'][:, 2]
        rgthandbcn = self.__genbcn_manhattan([[link6r, divdir6r], [link8r, divdir8r], [link9r, divdir9r]], radius=100)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link6l = robot.lftarm[6]
        divdir6l = robot.lftarm[6]['rotmat'][:, 0]
        link8l = robot.lftarm[8]
        divdir8l = robot.lftarm[8]['rotmat'][:, 2]
        link9l = robot.lftarm[9]
        divdir9l = robot.lftarm[9]['rotmat'][:, 2]
        lfthandbcn = self.__genbcn_manhattan([[link6l, divdir6l], [link8l, divdir8l], [link9l, divdir9l]], radius=100)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def gensemiactivebcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        Variation: only arms are considered

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:, 0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:, 0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                          [link2l, divdir2l]], radius=100)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius=100)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius=100)
        returnlist['arm'] = armbcn

        # rgt hand  Modifications for HRP5
        # link4r = robot.rgtarm[4]
        # divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        # link5r = robot.rgtarm[5]
        # divdir5r = robot.rgtarm[5]['rotmat'][:, 2]
        # rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius=60)

        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:, 0]
        link8r = robot.rgtarm[8]
        divdir8r = robot.rgtarm[8]['rotmat'][:, 2]
        link9r = robot.rgtarm[9]
        divdir9r = robot.rgtarm[9]['rotmat'][:, 2]
        rgthandbcn = self.__genbcn_manhattan([[link6r, divdir6r], [link8r, divdir8r]], radius=100)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand Modifications for HRP5
        # link4l = robot.lftarm[4]
        # divdir4l = robot.lftarm[4]['rotmat'][:, 1]
        # link5l = robot.lftarm[5]
        # divdir5l = robot.lftarm[5]['rotmat'][:, 2]
        # lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius=60)

        link6l = robot.lftarm[6]
        divdir6l = robot.lftarm[6]['rotmat'][:, 0]
        link8l = robot.lftarm[8]
        divdir8l = robot.lftarm[8]['rotmat'][:, 2]
        link9l = robot.lftarm[9]
        divdir9l = robot.lftarm[9]['rotmat'][:, 2]
        lfthandbcn = self.__genbcn_manhattan([[link6l, divdir6l], [link8l, divdir8l]], radius=100)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def genholdbcndict(self, robot, armid='rgt'):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        # returnlist = {}
        #
        # # body
        # radius = 80
        # tmpcolnode = CollisionNode("cdbody")
        # spos = np.array([0.0, 0.0, 0.0])
        # epos = np.array([0.0, 0.0, robot.rgtarm[0]['linkend'][2]])
        # self.__genbcn_ext(tmpcolnode, spos, epos, radius=radius)
        # spos = robot.rgtarm[0]['linkend']
        # epos = robot.lftarm[0]['linkend']
        # self.__genbcn_ext(tmpcolnode, spos, epos, radius=radius)
        # spos = robot.rgtarm[0]['linkend'] + np.array([0.0, 0.0, 150.0])
        # epos = robot.lftarm[0]['linkend'] + np.array([0.0, 0.0, 150.0])
        # bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=radius)
        # returnlist['body'] = bodybcn
        #
        # # arm
        # link2r = robot.rgtarm[2]
        # divdir2r = robot.rgtarm[2]['rotmat'][:, 0]
        # link2l = robot.lftarm[2]
        # divdir2l = robot.lftarm[2]['rotmat'][:, 0]
        # armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
        #                                   [link2l, divdir2l]], radius=60)
        # link3r = robot.rgtarm[3]
        # spos = link3r['linkpos']
        # epos = link3r['linkend']
        # sposnew = spos
        # eposnew = (spos + epos) / 2.0
        # self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        # link3l = robot.lftarm[3]
        # spos = link3l['linkpos']
        # epos = link3l['linkend']
        # sposnew = spos
        # eposnew = (spos + epos) / 2.0
        # armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        # returnlist['arm'] = armbcn
        # #  Modified for HRP5P
        # # # rgt hand
        # # link4r = robot.rgtarm[4]
        # # divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        # # link5r = robot.rgtarm[5]
        # # divdir5r = robot.rgtarm[5]['rotmat'][:, 2]
        # # if armid == 'lft':
        # #     link6r = robot.rgtarm[6]
        # #     divdir6r = robot.rgtarm[6]['rotmat'][:, 2]
        # #     rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]],
        # #                                          radius=60)
        # # else:
        # #     rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius=60)
        # # returnlist['rgthnd'] = rgthandbcn
        #
        # # # lft hand
        # # link5l = robot.lftarm[5]
        # # divdir5l = robot.lftarm[5]['rotmat'][:, 0]
        # # link8l = robot.lftarm[8]
        # # divdir8l = robot.lftarm[8]['rotmat'][:, 2]
        # # if armid == 'rgt':
        # #     link9l = robot.lftarm[9]
        # #     divdir9l = robot.lftarm[9]['rotmat'][:, 2]
        # #     lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l], [link9l, divdir9l]],
        # #                                          radius=60)
        # # else:
        # #     lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l]], radius=60)
        # # returnlist['lfthnd'] = lfthandbcn
        #
        # # rgt hand
        # link5r = robot.rgtarm[5]
        # divdir5r = robot.rgtarm[5]['rotmat'][:, 0]
        # link8r = robot.rgtarm[8]
        # divdir8r = robot.rgtarm[8]['rotmat'][:, 2]
        # if armid == 'lft':
        #     link9r = robot.rgtarm[9]
        #     divdir9r = robot.rgtarm[9]['rotmat'][:, 2]
        #     rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r], [link9r, divdir9r]],
        #                                          radius=80)
        # else:
        #     rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r]], radius=60)
        # returnlist['rgthnd'] = rgthandbcn
        #
        # # lft hand
        # link5l = robot.lftarm[5]
        # divdir5l = robot.lftarm[5]['rotmat'][:, 0]
        # link8l = robot.lftarm[8]
        # divdir8l = robot.lftarm[8]['rotmat'][:, 2]
        # if armid == 'rgt':
        #     link9l = robot.lftarm[9]
        #     divdir9l = robot.lftarm[9]['rotmat'][:, 2]
        #     lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l], [link9l, divdir9l]],
        #                                          radius=80)
        # else:
        #     lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l]], radius=60)
        # returnlist['lfthnd'] = lfthandbcn

        returnlist = {}

        # body
        # link0r = robot.rgtarm[0]
        # divdir0r = robot.rgtarm[0]['rotmat'][:,2]
        # link0l = robot.lftarm[0]
        # divdir0l = robot.lftarm[0]['rotmat'][:,2]
        # leglink3r = robot.rgtleg[3]
        # legdivdir3r = robot.rgtleg[3]['rotmat'][:,2]
        # leglink3l = robot.lftleg[3]
        # legdivdir3l = robot.lftleg[3]['rotmat'][:,2]
        # bodybcn = self.__genbcn_manhattan([[link0r, divdir0r], [leglink3r, legdivdir3r],
        #                                    [leglink3l, legdivdir3l]], radius = 180)
        # returnlist['body'] = bodybcn

        # body
        radius = 180.0
        tmpcolnode = CollisionNode("cdbody")
        # body
        bstart = robot.rgtarm[0]['linkpos']
        bend = robot.rgtarm[0]['linkpos'] + robot.rgtarm[0]['rotmat'][:, 2] * 1000.0
        self.__genbcn_ext(tmpcolnode, bstart, bend, radius=radius)
        # head
        bstart = robot.rgtarm[0]['linkpos'] + robot.rgtarm[0]['rotmat'][:, 2] * 800.0
        bend = robot.rgtarm[0]['linkpos'] + robot.rgtarm[0]['rotmat'][:, 2] * 800.0 + robot.rgtarm[0]['rotmat'][:,
                                                                                      0] * 300.0
        self.__genbcn_ext(tmpcolnode, bstart, bend, radius=radius)
        # shoulders
        rshstart = robot.rgtarm[2]['linkpos']
        rshend = robot.rgtarm[2]['linkpos'] + robot.rgtarm[2]['rotmat'][:, 2] * 10.0
        self.__genbcn_ext(tmpcolnode, rshstart, rshend, radius=radius * 5.0 / 9.0)
        lshstart = robot.lftarm[2]['linkpos']
        lshend = robot.lftarm[2]['linkpos'] + robot.lftarm[2]['rotmat'][:, 2] * 10.0
        bodybcn = self.__genbcn_ext(tmpcolnode, lshstart, lshend, radius=radius * 5.0 / 9.0)
        returnlist['body'] = bodybcn

        if armid == "lft":

            # rgt hand
            link4r = robot.rgtarm[4]
            divdir4r = robot.rgtarm[4]['rotmat'][:, 2]
            link5r = robot.rgtarm[5]
            divdir5r = robot.rgtarm[5]['rotmat'][:, 0]
            link8r = robot.rgtarm[8]
            divdir8r = robot.rgtarm[8]['rotmat'][:, 2]
            link9r = robot.rgtarm[9]
            divdir9r = robot.rgtarm[9]['rotmat'][:, 2]
            rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r], [link9r, divdir9r]], radius=100)
            returnlist['rgthnd'] = rgthandbcn
        else:

            # lft hand
            link4l = robot.lftarm[4]
            divdir4l = robot.lftarm[4]['rotmat'][:, 2]
            link5l = robot.lftarm[5]
            divdir5l = robot.lftarm[5]['rotmat'][:, 0]
            link8l = robot.lftarm[8]
            divdir8l = robot.lftarm[8]['rotmat'][:, 2]
            link9l = robot.lftarm[9]
            divdir9l = robot.lftarm[9]['rotmat'][:, 2]
            lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l], [link9l, divdir9l]], radius=100)
            returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def gensglactivebcndict(self, robot, armid='rgt'):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        ignoredarmid = "lft"
        if armid == "lft":
            ignoredarmid = "rgt"

        returnlist = {}

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:, 0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:, 0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                          [link2l, divdir2l]], radius=60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        returnlist['arm'] = armbcn

        # # rgt hand  HRP5P
        # link4r = robot.rgtarm[4]
        # divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        # link5r = robot.rgtarm[5]
        # divdir5r = robot.rgtarm[5]['rotmat'][:, 2]
        # if ignoredarmid == 'lft':
        #     link6r = robot.rgtarm[6]
        #     divdir6r = robot.rgtarm[6]['rotmat'][:, 2]
        #     rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]],
        #                                          radius=60)
        # else:
        #     rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius=60)
        # returnlist['rgthnd'] = rgthandbcn
        #
        # # lft hand
        # link4l = robot.lftarm[4]
        # divdir4l = robot.lftarm[4]['rotmat'][:, 1]
        # link5l = robot.lftarm[5]
        # divdir5l = robot.lftarm[5]['rotmat'][:, 2]
        # if ignoredarmid == 'rgt':
        #     link6l = robot.lftarm[6]
        #     divdir6l = robot.lftarm[6]['rotmat'][:, 2]
        #     lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l], [link6l, divdir6l]],
        #                                          radius=60)
        # else:
        #     lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius=60)
        # returnlist['lfthnd'] = lfthandbcn



        # rgt hand
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:, 0]
        link8r = robot.rgtarm[8]
        divdir8r = robot.rgtarm[8]['rotmat'][:, 2]
        if ignoredarmid == 'lft':
            link9r = robot.rgtarm[9]
            divdir9r = robot.rgtarm[9]['rotmat'][:, 2]
            # rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r],[link9r, divdir9r]],
            #                                      radius=80)  # If i do not include 9 it works
            rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r]], radius=80)

            # offset = 70.0
            # radius = 30.0
            # stip = robot.rgtarm[-1]['linkend'] + robot.rgtarm[-1]['rotmat'][:, 1] * offset + robot.rgtarm[-1]['rotmat'][
            #                                                                                  :,
            #                                                                                  2] * 10
            # etip = robot.rgtarm[-1]['linkend'] + robot.rgtarm[-1]['rotmat'][:, 1] * offset - robot.rgtarm[-1]['rotmat'][
            #                                                                                  :,
            #                                                                                  2] * 10
            # rgthandbcn = self.__genbcn_ext(rgthandbcn, stip, etip, radius=radius)
            # stip = robot.rgtarm[-1]['linkend'] - robot.rgtarm[-1]['rotmat'][:, 1] * offset + robot.rgtarm[-1]['rotmat'][
            #                                                                                  :,
            #                                                                                  2] * 40
            # etip = robot.rgtarm[-1]['linkend'] - robot.rgtarm[-1]['rotmat'][:, 1] * offset - robot.rgtarm[-1]['rotmat'][
            #                                                                                  :,
            #                                                                                  2] * 40
            # rgthandbcn = self.__genbcn_ext(rgthandbcn, stip, etip, radius=radius)
        else:
            rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link8r, divdir8r]], radius=80)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:, 0]
        link8l = robot.lftarm[8]
        divdir8l = robot.lftarm[8]['rotmat'][:, 2]
        if ignoredarmid == 'rgt':
            link9l = robot.lftarm[9]
            divdir9l = robot.lftarm[9]['rotmat'][:, 2]
            # lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l],[link9l, divdir9l]],
            #                                      radius=80)
            lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l]], radius=80)

            # offset = 70.0
            # radius = 30.0
            # stip = robot.lftarm[-1]['linkend'] + robot.lftarm[-1]['rotmat'][:, 1] * offset + robot.lftarm[-1]['rotmat'][
            #                                                                                  :, 2] * 40
            # etip = robot.lftarm[-1]['linkend'] + robot.lftarm[-1]['rotmat'][:, 1] * offset - robot.lftarm[-1]['rotmat'][
            #                                                                                  :, 2] * 40
            # lfthandbcn = self.__genbcn_ext(lfthandbcn, stip, etip, radius=radius)
            # stip = robot.lftarm[-1]['linkend'] - robot.lftarm[-1]['rotmat'][:, 1] * offset + robot.lftarm[-1]['rotmat'][
            #                                                                                  :, 2] * 10
            # etip = robot.lftarm[-1]['linkend'] - robot.lftarm[-1]['rotmat'][:, 1] * offset - robot.lftarm[-1]['rotmat'][
            #                                                                                  :, 2] * 10
            # lfthandbcn = self.__genbcn_ext(lfthandbcn, stip, etip, radius=radius)
        else:
            lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link8l, divdir8l]], radius=80)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def gensemibcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        Variation: only arms are considered

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        radius = 10
        tmpcolnode = CollisionNode("cdbody")
        spos = np.array([0.0, 0.0, 0.0])
        epos = np.array([0.0, 0.0, robot.rgtarm[0]['linkend'][2]])
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=radius)
        spos = robot.rgtarm[0]['linkend']
        epos = robot.lftarm[0]['linkend']
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=radius)
        spos = robot.rgtarm[0]['linkend'] + np.array([0.0, 0.0, 150.0])
        epos = robot.lftarm[0]['linkend'] + np.array([0.0, 0.0, 150.0])
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=radius)
        returnlist['body'] = bodybcn

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:, 0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:, 0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                          [link2l, divdir2l]], radius = 60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = (spos + epos) / 2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        returnlist['arm'] = armbcn

        #Modify for HRP5
        # # rgt hand
        # link4r = robot.rgtarm[4]
        # divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        # link5r = robot.rgtarm[5]
        # divdir5r = robot.rgtarm[5]['rotmat'][:, 2]
        # rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius=60)
        # returnlist['rgthnd'] = rgthandbcn
        #
        # # lft hand
        # link4l = robot.lftarm[4]
        # divdir4l = robot.lftarm[4]['rotmat'][:, 1]
        # link5l = robot.lftarm[5]
        # divdir5l = robot.lftarm[5]['rotmat'][:, 2]
        # lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius=60)
        # returnlist['lfthnd'] = lfthandbcn

        # rgt hand
        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:, 0]
        link8r = robot.rgtarm[8]
        divdir8r = robot.rgtarm[8]['rotmat'][:, 2]

        rgthandbcn = self.__genbcn_manhattan([[link6r, divdir6r], [link8r, divdir8r]], radius = 60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link6l = robot.lftarm[5]
        divdir6l = robot.lftarm[5]['rotmat'][:, 0]
        link8l = robot.lftarm[8]
        divdir8l = robot.lftarm[8]['rotmat'][:, 2]

        lfthandbcn = self.__genbcn_manhattan([[link6l, divdir6l], [link8l, divdir8l]], radius = 60)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def showbcn(self, base, bcndict):
        """
        show bcnlist to base

        :param bcndict is in the form of
            {'body': colnode, 'rgtarm': colnode, 'lftarm': colnode,
            'rgthnd': colnode, 'lfthnd': colnode}
        :return: null

        author: weiwei
        date: 20170615
        """

        self.unshowbcn()
        self.__shownp = NodePath("collision balls")
        for key, bcn in bcndict.items():
            tst = self.__shownp.attachNewNode(bcn)
            tst.show()
        self.__shownp.reparentTo(base.render)

    def unshowbcn(self):
        """
        show bcnlist to base

        :param bcnlist is in the form of
            [bodybcn, rgtupperbcn, rgtlowerbcn,rgthandbcn,
                    lftupperbcn, lftlowerbcn, lfthandbcn]
        :return: null

        author: weiwei
        date: 20170615
        """

        if self.__shownp is not None:
            self.__shownp.removeNode()
        self.__shownp = None


if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    import hrp5p
    import hrp5pmesh
    from manipulation.grip.hrp5pf3 import hrp5pf3

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World()

    robot = hrp5p.Hrp5PRobot()
    robot.movearmfk([36.96323999482342, 47.304213446505955, -20.010314308801952, 1.8175863426595333, -136.53954611945306, -121.99779800649138, -19.45860390580239, 20.986187479278477, -76.30568492324078], 'rgt')
    robot.movearmfk([-9.05984698163083, 49.83255387665413, 63.760871945261144, 42.062123295513906, -139.40501925020988, 0.20750131802771543, -5.570314977613898, -36.9995646342732, 70.91226996828175], 'lft')
    robotball = Hrp5PBall()
    # bcndict = robotball.genbcndict(robot, objhold=[80,70,'rgt'])
    jawwidthrgt=89.9208948669732
    jawwidthlft=120.0
    bcndict = robotball.genbcndict(robot)
    robotball.showbcn(base, bcndict)

    lfthnd = hrp5pf3.Hrp5pf3(hndid='lft')
    rgthnd = hrp5pf3.Hrp5pf3(hndid='rgt')
    hrp5pmgen = hrp5pmesh.Hrp5PMesh(lfthand=lfthnd, rgthand=rgthnd)
    hrp5pmnp = hrp5pmgen.genmnp(robot, jawwidthrgt, jawwidthlft)
    hrp5pmnp.reparentTo(base.render)
    hrp5psnp = hrp5pmgen.gensnp(robot)
    hrp5psnp.reparentTo(base.render)

    base.run()

