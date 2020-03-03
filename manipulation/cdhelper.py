"""
The cdhelper in manipulation is an exact mesh-mesh cd checker.
It converts the geom of a nodepath into bulletmeshnode and checks the collision.

author: weiwei
date: 20190313
"""

from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletWorld
import pandaplotutils.collisiondetection as cd
from panda3d.core import NodePath

class BNPchecker(object):

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

    def isObjObjCollided(self, obj0, obj1):
        """
        check if two objects obj0 as obj1 are in collision with each other
        the two objects are in the form of pandanodes

        :param obj0: the first object
        :param obj1: the second object
        :return: boolean value showing if the two objects are in collision

        author: weiwei
        date: 20180621
        """

        obj0bullnode = cd.genCollisionMeshMultiNp(obj0)
        obj1bullnode = cd.genCollisionMeshMultiNp(obj1)
        result = self.__base.world.contactTestPair(obj0bullnode, obj1bullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def isObjObjListCollided(self, obj0, objlist=[]):
        """
        check if an object obj0 is in collision with a list of objects in objlist
        each obj is in the form of a pandanode

        :param obj0: object
        :param objlist: a list of objects
        :return: boolean value showing if the object and the list is in collision

        author: weiwei
        date: 20180621
        """

        obj1 = NodePath("objobjlist")
        for obj in objlist:
            obj.reparentTo(obj1)
        result = self.isObjObjCollided(obj0, obj1)
        obj1.removeNode()
        return result

    def showObjObj(self, obj0, obj1):
        """
        show the collision meshes of the given objects

        :param obj0, obj1
        author: weiwei
        date: 20180621
        :return:
        """

        def updateworld(world, task):
            world.doPhysics(globalClock.getDt())
            return task.cont
        base.taskMgr.add(updateworld, "updateworld", extraArgs=[self.__base.world], appendTask=True)

        obj0bullnode = cd.genCollisionMeshMultiNp(obj0)
        obj1bullnode = cd.genCollisionMeshMultiNp(obj1)
        cdnode0 = self.__base.world.attachRigidBody(obj0bullnode)
        cdnode1 = self.__base.world.attachRigidBody(obj1bullnode)
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
    import os
    import time
    import database.dbaccess as db
    import pandaplotutils.pandactrl as pandactrl
    import manipulation.grip.robotiq85.rtq85nm as rtq85nm
    import manipulation.grip.freegrip as freegrip
    from panda3d.core import Vec3

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,100])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "grip/objects", "bunnysim.stl")
    objnp = base.pg.loadstlaspandanp_fn(objpath)
    objnp.setColor(.5,0,0,1)
    objnp.reparentTo(base.render)


    tblpath = os.path.join(this_dir, "env", "table.stl")
    tblnp = base.pg.loadstlaspandanp_fn(tblpath)
    tblnp.setColor(.5,.5,.5,1)
    tblnp.setPos(0,0,0)
    tblnp.reparentTo(base.render)

    handpkg = rtq85nm
    cdchecker = BNPchecker(base)

    gdb = db.GraspDB(database="ur3dualgrip")
    data = gdb.loadFreeAirGrip('bunnysim', 'rtq85')
    if data:
        freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth = data
        print(len(freegripid))
        for i, freegriprotmat in enumerate(freegriprotmats):
            # if i>12 and i <100:
            rtqhnd = rtq85nm.Rtq85NM(hndcolor=[1, 1, 1, .2])
            rtqhnd.setMat(pandanpmat4=freegriprotmat)
            rtqhnd.setJawwidth(freegripjawwidth[i])
            tic = time.time()
            cdchecker.isObjObjCollided(rtqhnd.handnp, tblnp)
            # cdchecker.showObjObj(rtqhnd.handnp, tblnp)
            # break
            toc = time.time()
            print(toc-tic)
            if cdchecker.isObjObjCollided(rtqhnd.handnp, tblnp):
                rtqhnd.setColor([1,0,1,.2])
            else:
                # if Vec3(freegriprotmat.getRow3(0)).angleDeg(Vec3(0,0,1)) < 30:
                #     cdchecker.showObjObj(rtqhnd.handnp, tblnp)
                rtqhnd.reparentTo(base.render)
    base.run()