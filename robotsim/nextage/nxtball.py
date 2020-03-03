import numpy as np
import math
import utiltools.robotmath as rm
from panda3d.core import NodePath
from panda3d.core import CollisionNode
from panda3d.core import CollisionSphere

class NxtBall(object):
    """
    generate nxtballs for quick collision detection

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self):
        """
        load models

        author: weiwei
        date: 20180110
        """

        # b indicates ball, cn indicates collision node
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
            linklength, linkvec = rm.unitvec_safe(epos-spos)
            nball = int(math.ceil(linklength / balldist))
            for i in range(1, nball):
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

        :param mothercn: the generated cd balls will be added to the tmpcolnode
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

        :param robot: see related definitions
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
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,ballradius,50]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,-ballradius,50]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([-100,ballradius,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([-100,-ballradius,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,0,150]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,0,150]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,ballradius,300]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,-ballradius,300]))
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        returnlist['body'] = bodybcn

        # arm
        armbcn = CollisionNode("cdarm")
        ballradius = 60
        link2r = robot.rgtarm[2]
        spos = link2r['linkpos']
        epos = link2r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link2l = robot.lftarm[2]
        spos = link2l['linkpos']
        epos = link2l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        returnlist['arm'] = armbcn

        # hands
        ballradius = 60
        # rgt hand
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link6r, divdir6r]], radius = ballradius)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        link6l = robot.lftarm[6]
        divdir6l = robot.lftarm[6]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link6l, divdir6l]], radius = ballradius)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def genholdbcndict(self, robot, armname = 'rgt'):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here

        :param robot: see related definitions
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
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,ballradius,50]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,-ballradius,50]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([-100,ballradius,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([-100,-ballradius,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,0,150]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,0,150]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,ballradius,300]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,-ballradius,300]))
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        returnlist['body'] = bodybcn

        # arm
        armbcn = CollisionNode("cdarm")
        ballradius = 60
        link2r = robot.rgtarm[2]
        spos = link2r['linkpos']
        epos = link2r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link2l = robot.lftarm[2]
        spos = link2l['linkpos']
        epos = link2l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        returnlist['arm'] = armbcn

        # hands
        ballradius = 60
        # rgt hand
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        if armname == 'lft':
            link6r = robot.rgtarm[6]
            divdir6r = robot.rgtarm[6]['rotmat'][:,2]
            rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link6r, divdir6r]], radius = ballradius)
        else:
            rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r]], radius = ballradius)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        if armname == 'rgt':
            link6l = robot.lftarm[6]
            divdir6l = robot.lftarm[6]['rotmat'][:,2]
            lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link6l, divdir6l]], radius = ballradius)
        else:
            lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l]], radius = ballradius)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def gensemibcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        Variation: only arms are considered

        :param robot: see related definitions
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
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,ballradius,50]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,-ballradius,50]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([-100,ballradius,0]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([-100,-ballradius,0]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,0,150]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,0,150]))
        self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        spos = robot.rgtarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,ballradius,300]))
        epos = robot.lftarm[0]['linkend'] + np.dot(robot.base['rotmat'], np.array([20,-ballradius,300]))
        bodybcn = self.__genbcn_ext(tmpcolnode, spos, epos, radius=ballradius)
        returnlist['body'] = bodybcn

        # arm
        armbcn = CollisionNode("cdarm")
        ballradius = 60
        link2r = robot.rgtarm[2]
        spos = link2r['linkpos']
        epos = link2r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/2.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link2l = robot.lftarm[2]
        spos = link2l['linkpos']
        epos = link2l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/2.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link3r = robot.rgtarm[3]
        spos = link3r['linkpos']
        epos = link3r['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        link3l = robot.lftarm[3]
        spos = link3l['linkpos']
        epos = link3l['linkend']
        sposnew = spos
        eposnew = spos+(epos-spos)/3.0
        armbcn = self.__genbcn_ext(armbcn, sposnew, eposnew, radius = ballradius)
        returnlist['arm'] = armbcn

        # hands
        ballradius = 60
        # rgt hand
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r]], radius = ballradius)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l]], radius = ballradius)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def genfullactivebcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        Variation: both arms and hands are considered

        :param robot: see related definitions
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20190401
        """

        return self.genfullbcndict(robot)

    def genholdactivebcndict(self, robot, armname = 'rgt'):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        one of the grippers is ignored

        :param robot: see related definitions
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20190401
        """

        return self.genholdbcndict(robot, armname=armname)

    def gensemiactivebcndict(self, robot):
        """
        generate the ball collision nodes of a robot
        only the active (movable) parts of the robot is considered here
        gripper balls are not included

        :param robot: see related definitions
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20190401
        """

        return self.gensemibcndict(robot)

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
    import nxt
    from manipulation.grip.robotiq85 import rtq85
    import nxt
    import nxtmesh

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World(camp=[3000,400,3000], lookatp = [0,0,1000])

    nxtrobot = nxt.NxtRobot()
    nxtrobot.movewaist(70)
    nxtb = NxtBall()
    # bcndict = nxtb.genfullbcndict(nxtrobot)
    # bcndict = nxtb.gensemibcndict(nxtrobot)
    bcndict = nxtb.genholdbcndict(nxtrobot, armname ="lft")
    nxtb.showcn(base, bcndict)
    rgthnd = rtq85.Rtq85(ftsensoroffset=0.0)
    lfthnd = rtq85.Rtq85(ftsensoroffset=0.0)
    nxtmeshgen = nxtmesh.NxtMesh(rgthand=rgthnd, lfthand = lfthnd)
    nxtmnp = nxtmeshgen.genmnp(nxtrobot)
    nxtmnp.reparentTo(base.render)

    base.run()

