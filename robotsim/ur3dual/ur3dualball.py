import numpy as np
import math
import utiltools.robotmath as rm
from panda3d.core import NodePath
from panda3d.core import CollisionNode
from panda3d.core import CollisionSphere

class Ur3DualBall(object):
    """
    generate ur3dual for quick collision detection
    """

    def __init__(self):
        """
        load models

        author: weiwei
        date: 20180130
        """

        # b indicates ball, cn indicates collision node
        self.__shownp = None

    def __genbcn_direct(self, linklist, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link
        the balls are generated along the direction of the links

        :param linkid:
        :param armname:
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
            for i in range(0, nball):
                pos = spos+linkvec*i*(balldist)
                # if i == nball-1:
                #     pos = spos+linkvec*(i-.4)*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def __genbcn_manhattan(self, linklist, mothercn = None, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link
        the balls are generated along the manhattan directions between joints

        :return:

        author: weiwei
        date: 20171101
        """

        radius = float(radius)
        balldist = radius

        if mothercn is None:
            mothercn = CollisionNode(name)
        for link, divdir in linklist:
            spos = link['linkpos']
            epos = link['linkend']
            # linkvec direction
            linkdir = np.dot((epos-spos), divdir)*divdir
            linkdirlength = np.linalg.norm(linkdir)
            if linkdirlength < 1e-4:
                continue
            linkdirnormalized = linkdir/linkdirlength
            nball = int(math.ceil(linkdirlength / balldist))
            for i in range(0, nball):
                pos = spos+linkdirnormalized*i*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                mothercn.addSolid(colsphere)
            # linkvec orthogonal direction
            linkorth = epos-spos-linkdir
            linkorthlength = np.linalg.norm(linkorth)
            if linkorthlength < 1e-4:
                continue
            linkorthnormalized = linkorth/linkorthlength
            nball = int(math.ceil(linkorthlength / balldist))
            for i in range(0, nball):
                pos = spos+linkdir+linkorthnormalized*i*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                mothercn.addSolid(colsphere)
        return mothercn

    def __genbcn_ext(self, mothercn, spos, epos, radius=70.0, name = "autogen"):
        """
        generate extra ball collision node for non-link decorations
        the balls follow a line connecting spos and epos

        :param mothercn: the generated cd balls will be added to the mothercn
        :param spos:
        :param epos:
        :param radius:
        :param name:
        :return:

        author: weiwei
        date: 20180420
        """

        balldist = radius

        linklength, linkvec = rm.unitvec_safe(epos - spos)
        if linklength < 1e-4:
            return None
        nball = int(math.ceil(linklength / balldist))
        ratio = np.linspace(0, linklength, nball, endpoint=True)
        spos = np.array(spos).reshape(-1, 1)
        linkvec = np.array(linkvec).reshape(-1, 1)
        ballpos = spos+linkvec*ratio
        for i in range(0, nball):
            pos = ballpos[:, i].reshape(-1, 1)
            colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
            mothercn.addSolid(colsphere)
        return mothercn

    def genfullbcndict(self, robot):
        """
        generate the ball collision nodes of a robot

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        tmpcolnode = CollisionNode("cdbody")
        ballradius = 80
        spos = robot.base['linkpos']
        epos = (robot.rgtarm[0]['linkend']+robot.lftarm[0]['linkend'])/2.0
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,ballradius,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-ballradius,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,5,0]))
        epos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,0,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius+20)
        spos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-5,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,0,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius+20)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,2*ballradius,150]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-2*ballradius,150]))
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        returnlist['body'] = bodybcn

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:,0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:,0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r], [link2l, divdir2l]], radius = 60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        returnlist['arm'] = armbcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:,1]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]], radius = 60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:,1]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        link6l = robot.lftarm[6]
        divdir6l = robot.lftarm[6]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l], [link6l, divdir6l]], radius = 60)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def genholdbcndict(self, robot, armname = 'rgt'):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        tmpcolnode = CollisionNode("cdbody")
        ballradius = 80
        spos = robot.base['linkpos']
        epos = (robot.rgtarm[0]['linkend']+robot.lftarm[0]['linkend'])/2.0
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,ballradius,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-ballradius,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,5,0]))
        epos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,0,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius+20)
        spos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-5,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,0,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius+20)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,2*ballradius,150]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-2*ballradius,150]))
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        returnlist['body'] = bodybcn

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:,0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:,0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r], [link2l, divdir2l]], radius = 60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        returnlist['arm'] = armbcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:,1]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        if armname == 'lft':
            link6r = robot.rgtarm[6]
            divdir6r = robot.rgtarm[6]['rotmat'][:,2]
            rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]], radius = 60)
        else:
            rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius = 60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:,1]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        if armname == 'rgt':
            link6l = robot.lftarm[6]
            divdir6l = robot.lftarm[6]['rotmat'][:,2]
            lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l], [link6l, divdir6l]], radius = 60)
        else:
            lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius = 60)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def gensemibcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        gripper balls are excluded

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        tmpcolnode = CollisionNode("cdbody")
        ballradius = 80
        spos = robot.base['linkpos']
        epos = (robot.rgtarm[0]['linkend']+robot.lftarm[0]['linkend'])/2.0
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,ballradius,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-ballradius,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,5,0]))
        epos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,0,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius+20)
        spos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-5,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,0,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius+20)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,2*ballradius,150]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([0,-2*ballradius,150]))
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        returnlist['body'] = bodybcn

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:,0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:,0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                           [link2l, divdir2l]], radius = 60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        returnlist['arm'] = armbcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius = 60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:, 1]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius = 60)
        returnlist['lfthnd'] = lfthandbcn

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

        returnlist = {}

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:,0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:,0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                           [link2l, divdir2l]], radius = 60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = (spos+epos)/2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = (spos+epos)/2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        returnlist['arm'] = armbcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:,1]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]], radius = 60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:,1]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        link6l = robot.lftarm[6]
        divdir6l = robot.lftarm[6]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l], [link6l, divdir6l]], radius = 60)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def genholdactivebcndict(self, robot, armname = 'rgt'):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        one of the grippers is ignored

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
                                          [link2l, divdir2l]], radius=60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius=60)
        returnlist['arm'] = armbcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:, 1]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:, 2]
        if armname == 'lft':
            link6r = robot.rgtarm[6]
            divdir6r = robot.rgtarm[6]['rotmat'][:, 2]
            rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r], [link6r, divdir6r]],
                                                 radius=60)
        else:
            rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius=60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:, 1]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:, 2]
        if armname == 'rgt':
            link6l = robot.lftarm[6]
            divdir6l = robot.lftarm[6]['rotmat'][:, 2]
            lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l], [link6l, divdir6l]],
                                                 radius=60)
        else:
            lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius=60)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def gensemiactivebcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        gripper balls are not included

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # arm
        link2r = robot.rgtarm[2]
        divdir2r = robot.rgtarm[2]['rotmat'][:,0]
        link2l = robot.lftarm[2]
        divdir2l = robot.lftarm[2]['rotmat'][:,0]
        armbcn = self.__genbcn_manhattan([[link2r, divdir2r],
                                           [link2l, divdir2l]], radius = 60)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = 60)
        returnlist['arm'] = armbcn

        # rgt hand
        link4r = robot.rgtarm[4]
        divdir4r = robot.rgtarm[4]['rotmat'][:,1]
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link4r, divdir4r], [link5r, divdir5r]], radius = 60)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link4l = robot.lftarm[4]
        divdir4l = robot.lftarm[4]['rotmat'][:,1]
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link4l, divdir4l], [link5l, divdir5l]], radius = 60)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def showcn(self, base, bcndict):
        """
        show bcnlist to base

        :param bcndict is in the form of
            {'body': colnode, 'rgtarm': colnode, 'lftarm': colnode,
            'rgthnd': colnode, 'lfthnd': colnode}
        :return: null

        author: weiwei
        date: 20170615
        """

        self.unshowcn()
        self.__shownp = NodePath("collision balls")
        for key, bcn in bcndict.items():
            tst = self.__shownp.attachNewNode(bcn)
            tst.show()
        self.__shownp.reparentTo(base.render)
        return self.__shownp

    def unshowcn(self):
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
    import manipulation.grip.robotiq85.rtq85 as rtq85
    from robotsim.ur3dual import ur3dual
    from robotsim.ur3dual import ur3dualmesh
    import utiltools.robotmath as rm

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World(camp = [3000,3000,3000], lookatp=[0,0,1300])

    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()

    ur3dualrobot = ur3dual.Ur3DualRobot(position=[0,500,0], rotmat=rm.rodrigues([0,0,1],70))
    # goalpos = np.array([200,120,1410])
    # goalrot = np.array([[1,0,0],[0,0,-1],[0,1,0]])
    # goal = ur3dualrobot.numik(goalpos, goalrot, armname = 'lft')
    # goal = np.array([-46.251876352032305, -41.418654079396184, 94.34173209615693, -15.917387039376576, 92.07172082393664, -25.134340575525133])
    goal = np.array([-209.13573775, -42.61307312, -283.86285256, 30.51538756, -231.85631837, -218.24860474])
    ur3dualrobot.movearmfk(goal, armname='rgt')
    # ur3dualrobot.gozeropose()
    ur3dualrobotball = Ur3DualBall()
    # bcndict = ur3dualrobotball.genfullbcndict(ur3dualrobot)
    # bcndict = ur3dualrobotball.gensemibcndict(ur3dualrobot)
    # bcndict = ur3dualrobotball.genholdbcndict(ur3dualrobot, armname='rgt')
    # bcndict = ur3dualrobotball.genfullactivebcndict(ur3dualrobot)
    # bcndict = ur3dualrobotball.gensemiactivebcndict(ur3dualrobot)
    bcndict = ur3dualrobotball.genholdactivebcndict(ur3dualrobot, armname='rgt')
    ur3dualrobotball.showcn(base, bcndict)
    ur3dualrobotmeshgen = ur3dualmesh.Ur3DualMesh(lfthand = lfthnd, rgthand = rgthnd)
    ur3dualrobotmnp = ur3dualrobotmeshgen.genmnp(ur3dualrobot, togglejntscoord=True)
    ur3dualrobotmnp.reparentTo(base.render)
    ur3dualrobotsnp = ur3dualrobotmeshgen.gensnp(ur3dualrobot)
    ur3dualrobotsnp.reparentTo(base.render)

    base.pggen.plotAxis(base.render)
    base.run()