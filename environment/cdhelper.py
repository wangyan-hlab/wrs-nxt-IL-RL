"""
The cdchecker in obstacles is a AABB box checker.
It converts a collision model into a bulletmeshnode and checks the collision.

author: weiwei
date: 20190313
"""

from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletWorld
import environment.collisiondetection as cmcd

class BCMchecker(object):

    def __init__(self, base):
        self.__base = base
        self.__base.world = BulletWorld()
        bulletcolliderrender = self.__base.render.attachNewNode("bulletcollider")

        debugNode = BulletDebugNode('Debug')
        debugNode.showWireframe(True)
        debugNode.showConstraints(True)
        debugNode.showBoundingBoxes(False)
        debugNode.showNormals(False)
        self.__debugNP = bulletcolliderrender.attachNewNode(debugNode)
        self.__base.world.setDebugNode(self.__debugNP.node())

        self.worldrigidbodylist = []

    def isObjObjCollided(self, objcm0, objcm1):
        """
        check if two objects objcm0 as objcm1 are in collision with each other
        the two objects are in the form of collision model
        the AABB boxes will be used
        type "box" is required

        :param objcm0: the first object
        :param objcm1: the second object
        :return: boolean value showing if the two objects are in collision

        author: weiwei
        date: 20190313
        """

        objcm0boxbullnode = cmcd.genBulletCDBox(objcm0)
        objcm1boxbullnode = cmcd.genBulletCDBox(objcm1)
        result = self.__base.world.contactTestPair(objcm0boxbullnode, objcm1boxbullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def isObjObjListCollided(self, objcm0, objcmlist=[]):
        """
        check if objcm0 is in collision with a list of collisionmodels in objcmlist
        each obj is in the form of a collision model

        :param objcm0:
        :param obcmjlist: a list of collision models
        :return: boolean value showing if the object and the list are in collision

        author: weiwei
        date: 20190313
        """

        objcm0boxbullnode = cmcd.genBulletCDBox(objcm0)
        objcm1boxbullnode = cmcd.genBulletCDBoxes(objcmlist)
        result = self.__base.world.contactTestPair(objcm0boxbullnode, objcm1boxbullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def showObjObj(self, objcm0, objcm1):
        """
        show the AABB collision meshes of the given objects

        :param objcm0, objcm1

        author: weiwei
        date: 20190313
        :return:
        """

        def updateworld(world, task):
            world.doPhysics(globalClock.getDt())
            return task.cont
        base.taskMgr.add(updateworld, "updateworld", extraArgs=[self.__base.world], appendTask=True)

        objcm0boxbullnode = cmcd.genBulletCDBox(objcm0)
        objcm1boxbullnode = cmcd.genBulletCDBox(objcm1)
        cdnode0 = self.__base.world.attachRigidBody(objcm0boxbullnode)
        cdnode1 = self.__base.world.attachRigidBody(objcm1boxbullnode)
        self.worldrigidbodylist.append(cdnode0)
        self.worldrigidbodylist.append(cdnode1)
        self.__debugNP.show()

    def showObjList(self, objcmlist):
        """
        show the AABB collision meshes of the given objects

        :param objcm0, objcm1

        author: weiwei
        date: 20190313
        :return:
        """

        def updateworld(world, task):
            world.doPhysics(globalClock.getDt())
            return task.cont
        base.taskMgr.add(updateworld, "updateworld", extraArgs=[self.__base.world], appendTask=True)

        objcmboxbullnode = cmcd.genBulletCDBoxes(objcmlist)
        cdnode = self.__base.world.attachRigidBody(objcmboxbullnode)
        self.worldrigidbodylist.append(cdnode)
        self.__debugNP.show()

    def showObjObjList(self, objcm0, objcm1list):
        """
        show the AABB collision meshes of the given objects

        :param objcm0, objcm1

        author: weiwei
        date: 20190313
        :return:
        """

        def updateworld(world, task):
            world.doPhysics(globalClock.getDt())
            return task.cont
        base.taskMgr.add(updateworld, "updateworld", extraArgs=[self.__base.world], appendTask=True)

        objcm0boxbullnode = cmcd.genBulletCDBox(objcm0)
        objcm1boxbullnode = cmcd.genBulletCDBoxes(objcm1list)
        cdnode0 = self.__base.world.attachRigidBody(objcm0boxbullnode)
        cdnode1 = self.__base.world.attachRigidBody(objcm1boxbullnode)
        self.worldrigidbodylist.append(cdnode0)
        self.worldrigidbodylist.append(cdnode1)
        self.__debugNP.show()

    def unshow(self):
        """
        unshow everything

        author: weiwei
        date: 20180621
        :return:
        """

        base.taskMgr.remove("updateworld")
        for cdnode in self.worldrigidbodylist:
            cdnode.removeNode()
        self.__debugNP.hide()

if __name__=='__main__':
    import environment.ur3dbunri as ur3dbunri
    import pandaplotutils.pandactrl as pandactrl
    import manipulation.grip.robotiq85.rtq85nm as rtq85nm

    base = pandactrl.World(camp=[2700,300,2700], lookatp=[0,0,1000])
    env = ur3dbunri.Env()
    env.reparentTo(base.render)
    objcm = env.loadobj("bunnysim.stl")

    bcmc = BCMchecker(base)
    bcmc.showObjList(env.getstationaryobslist())
    base.run()