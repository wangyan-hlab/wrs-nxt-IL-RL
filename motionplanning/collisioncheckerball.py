import copy
import json
import numpy as np
from panda3d.core import *

def getcdboxcn(pandanp, name = 'boxcd'):
    """

    :param obstacle:
    :return:

    author: weiwei
    date: 20180811
    """

    cnp = CollisionNode(name)
    bottomLeft, topRight = pandanp.getTightBounds()
    center = (bottomLeft+topRight)/2.0
    # enlarge the bounding box
    bottomLeft -= (bottomLeft-center).normalize()*15.0
    topRight += (topRight-center).normalize()*15.0
    cbn = CollisionBox(bottomLeft, topRight)
    cnp.addSolid(cbn)
    return cnp

class CollisionCheckerBall(object):
    """
    check the collision of a robot, using ball fitting

    """

    def __init__(self, robotball):
        """
        set up the collision checker

        :param robotball is an object of robotsim/robot.robotball

        author: weiwei
        date: 20170615
        """

        self.__robotball = robotball

    def isSelfCollided(self, robot):
        """
        check if the robot collides with its self

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170615
        """

        bcndict = self.__robotball.genfullbcndict(robot)

        if ('body' in bcndict) and (len(bcndict) > 1):
            bodynp = None
            othernp = NodePath("collision nodepath")
            for key, bcn in bcndict.items():
                if key is 'body':
                    bodynp = othernp.attachNewNode(bcn)
                else:
                    othernp.attachNewNode(bcn)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(bodynp, chan)
            ctrav.traverse(othernp)
            if chan.getNumEntries() > 0:
                print("collision between armhnd and body")
                return True
            bcndict.pop('body')

        if ('arm' in bcndict) and (len(bcndict) > 1):
            armnp = None
            othernp = NodePath("collision nodepath")
            for key, acn in bcndict.items():
                if key is 'arm':
                    armnp = othernp.attachNewNode(acn)
                else:
                    othernp.attachNewNode(acn)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(armnp, chan)
            ctrav.traverse(othernp)
            if chan.getNumEntries() > 0:
                print("collision between arm and hnd")
                return True
            bcndict.pop('arm')

        if ('rgthnd' in bcndict) and (len(bcndict) > 1):
            rgthandnp = None
            rgthndarmcnp = NodePath("collision nodepath")
            for key, bcn in bcndict.items():
                if key is 'rgthnd':
                    rgthandnp = rgthndarmcnp.attachNewNode(bcn)
                else:
                    rgthndarmcnp.attachNewNode(bcn)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(rgthandnp, chan)
            ctrav.traverse(rgthndarmcnp)
            if chan.getNumEntries() > 0:
                print("rgthnd-lfthnd collision")
                return True
            bcndict.pop('rgthnd')

        # if bcndict.has_key('lfthnd') and len(bcndict) > 1:
        #     lfthandnp = None
        #     lfthndarmcnp = NodePath("collision nodepath")
        #     for key, bcn in bcndict.items():
        #         if key is 'lfthnd':
        #             lfthandnp = lfthndarmcnp.attachNewNode(bcn)
        #         else:
        #             lfthndarmcnp.attachNewNode(bcn)
        #     ctrav = CollisionTraverser()
        #     chan = CollisionHandlerQueue()
        #     ctrav.addCollider(lfthandnp, chan)
        #     ctrav.traverse(lfthndarmcnp)
        #     if chan.getNumEntries() > 0:
        #         print "lfthnd-arm collision"
        #         return True
        #     bcndict.pop('lfthnd')

        return False

    def isSelfCollidedSEMI(self, robot):
        """
        check if the robot is self-collided
        gripper balls are exclulded

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170615
        """

        bcndict = self.__robotball.gensemibcndict(robot)

        if ('body' in bcndict) and (len(bcndict) > 1):
            bodynp = None
            othernp = NodePath("collision nodepath")
            for key, bcn in bcndict.items():
                if key is 'body':
                    bodynp = othernp.attachNewNode(bcn)
                else:
                    othernp.attachNewNode(bcn)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(bodynp, chan)
            ctrav.traverse(othernp)
            if chan.getNumEntries() > 0:
                print("collision between armhnd and body")
                return True
            bcndict.pop('body')

        if ('arm' in bcndict) and (len(bcndict) > 1):
            armnp = None
            othernp = NodePath("collision nodepath")
            for key, acn in bcndict.items():
                if key is 'arm':
                    armnp = othernp.attachNewNode(acn)
                else:
                    othernp.attachNewNode(acn)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(armnp, chan)
            ctrav.traverse(othernp)
            if chan.getNumEntries() > 0:
                print("collision between arm and hnd")
                return True
            bcndict.pop('arm')

        if ('rgthnd' in bcndict) and (len(bcndict) > 1):
            rgthandnp = None
            rgthndarmcnp = NodePath("collision nodepath")
            for key, bcn in bcndict.items():
                if key is 'rgthnd':
                    rgthandnp = rgthndarmcnp.attachNewNode(bcn)
                else:
                    rgthndarmcnp.attachNewNode(bcn)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(rgthandnp, chan)
            ctrav.traverse(rgthndarmcnp)
            if chan.getNumEntries() > 0:
                print("rgthnd-lfthnd collision")
                return True
            bcndict.pop('rgthnd')

        return False

    def isRobotObstacleCollided(self, robot, obstaclecmlist = [], holdarmname = "all"):
        """
        check if the robot collides with obstacles at key poses

        :param holdarmname: the armnames that will not be checked, by default both will be ignored
        :return:

        author: weiwei
        date: 20180811
        """

        # obj robot
        if holdarmname is None:
            bcndict = self.__robotball.genfullactivebcndict(robot)
        elif holdarmname is "all":
            bcndict = self.__robotball.gensemiactivebcndict(robot)
        elif holdarmname in ["rgt", "lft"]:
            bcndict = self.__robotball.genholdactivebcndict(robot, holdarmname)
        rocnp = NodePath("all cd nodepath")
        robotcn = CollisionNode("robot cd nodepath")
        for key, bcn in bcndict.items():
            for solid in bcn.getSolids():
                robotcn.addSolid(solid)
        robotcnp = rocnp.attachNewNode(robotcn)
        for obstaclecm in obstaclecmlist:
            obstaclecm.copycdnpTo(rocnp)
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(robotcnp, chan)
        ctrav.traverse(rocnp)
        if chan.getNumEntries() > 0:
            print("obstacle-robot collision at kp")
            return True
        return False

    def isObjectOthersCollided(self, objcm, robot, armid, obstaclecmlist = []):
        """

        :param object:
        :param obstaclecdnplist:
        :return:

        author: weiwei
        date: 20180811
        """

        oocnp = NodePath("collision nodepath")
        objcnp = objcm.copycdnpTo(oocnp)
        bcndict = self.__robotball.genholdbcndict(robot, armid)
        robotcn = CollisionNode("robot cd nodepath")
        for key, bcn in bcndict.items():
            for solid in bcn.getSolids():
                robotcn.addSolid(solid)
        oocnp.attachNewNode(robotcn)
        for obstaclecm in obstaclecmlist:
            obstaclecm.copycdnpTo(oocnp)
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(objcnp, chan)
        ctrav.traverse(oocnp)
        if chan.getNumEntries() > 0:
            print("object-others collision")
            # import robotsim.ur3dual.ur3dualball as ur3dualball
            # obstaclenp.reparentTo(base.render)
            # obstaclenp.show()
            # objcnp.reparentTo(base.render)
            # objcnp.show()
            # ur3dualrobotball = ur3dualball.Ur3DualBall()
            # ur3dualrobotball.showbcn(base, bcndict)
            # base.run()
            return True
        return False

    def isObjectsOthersCollided(self, objcmlist, robot, armid, obstaclecmlist = []):
        """

        :param object:
        :param obstaclecdnplist:
        :return:

        author: weiwei
        date: 20180811
        """

        oocnp0 = NodePath("collision nodepath")
        objcnplist = []
        for objcm in objcmlist:
            objcnplist.append(objcm.copycdnpTo(oocnp0))
        oocnp1 = NodePath("collision nodepath")
        bcndict = self.__robotball.genholdbcndict(robot, armid)
        robotcn = CollisionNode("robot cd nodepath")
        for key, bcn in bcndict.items():
            for solid in bcn.getSolids():
                robotcn.addSolid(solid)
        oocnp1.attachNewNode(robotcn)
        for obstaclecm in obstaclecmlist:
            obstaclecm.copycdnpTo(oocnp1)
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        for objcnp in objcnplist:
            ctrav.addCollider(objcnp, chan)
        ctrav.traverse(oocnp1)
        if chan.getNumEntries() > 0:
            print("objects-others collision")
            # import robotsim.ur3dual.ur3dualball as ur3dualball
            # obstaclenp.reparentTo(base.render)
            # obstaclenp.show()
            # objcnp.reparentTo(base.render)
            # objcnp.show()
            # ur3dualrobotball = ur3dualball.Ur3DualBall()
            # ur3dualrobotball.showbcn(base, bcndict)
            # base.run()
            return True
        return False

    def isCollided(self, robot, obstaclecmlist = [], holdarmname=None):
        """
        check the collision of robot-robot and robot-obstacle
        full self collision + full, semi, or hold robot-obstacle collision

        :param holdarmname: 'all' or 'lft' or 'rgt' or None, by default all
        :return:

        author: weiwei
        date: 20170615
        """

        bselfcollided = self.isSelfCollided(robot)
        if bselfcollided:
            return True
        else:
            return self.isRobotObstacleCollided(robot, obstaclecmlist, holdarmname=holdarmname)

    def isCollidedHO(self, robot, obstaclecmlist = []):
        """
        check the collision of robot-robot and robot-obstacle, variation
        semi self collision + full robot-obstacle collision

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170615
        """

        # iselfcollidedsemi is used to exclude gripper-gripper collision
        bselfcollided = self.isSelfCollidedSEMI(robot)
        if bselfcollided:
            return True
        else:
            # ho pose is always in the air, we dont exclude any hands
            return self.isRobotObstacleCollided(robot, obstaclecmlist, holdarmname=None)

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    # from direct.filter.CommonFilters import CommonFilters
    # from robotsim.ur5dual import ur5dual
    # from robotsim.ur5dual import ur5dualplot
    from robotsim.ur3dual import ur3dual
    from robotsim.ur3dual import ur3dualmesh
    from robotsim.ur3dual import ur3dualball
    # from robotsim.nextage import nxt
    # from robotsim.nextage import nxtplot
    # from manipulation.grip.robotiq85 import rtq85nm
    # from manipulation.grip.hrp5three import hrp5threenm

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    robotpose = [0.0, 90.0, -90.0, -45.0, -90.0, 0]
    goal = np.array([-209.13573775, -42.61307312, -283.86285256, 30.51538756, -231.85631837, -218.24860474])
    robot = ur3dual.Ur3DualRobot()
    robot.movearmfk(goal)
    robotmeshgen = ur3dualmesh.Ur3DualMesh()
    robotball = ur3dualball.Ur3DualBall()

    robotmnp = robotmeshgen.genmnp(robot)
    robotmnp.reparentTo(base.render)
    bcnlist = robotball.genfullbcndict(robot)
    robotball.showcn(base, bcnlist)

    cdchecker = CollisionCheckerBall(robotball)
    print(cdchecker.isSelfCollided(robot))
    base.run()