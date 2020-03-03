from panda3d.core import NodePath
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionHandlerQueue
from panda3d.core import TransformState
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletBoxShape

def genBulletCDBoxes(obstaclecmlist, name = 'autogen'):
    """
    generate a bullet cd obj using the AABB boundaries stored in obstacle collision models

    :param obstaclecmlist: a list of collision models (cmshare doesnt work!)
    :return: bulletrigidbody

    author: weiwei
    date: 20190313, toyonaka
    """

    bulletboxesnode = BulletRigidBodyNode(name)
    for obstaclecm in obstaclecmlist:
        if obstaclecm.type is not "box":
            raise Exception("Wrong obstaclecm type! Box is required to genBulletCDBox.")
        cdsolid = obstaclecm.cdcn.getSolid(0)
        bulletboxshape = BulletBoxShape.makeFromSolid(cdsolid)
        bulletboxesnode.addShape(bulletboxshape,
                                 TransformState.makeMat3(obstaclecm.getMat().getUpper3()).
                                 setPos(obstaclecm.getPos()+cdsolid.getCenter()))
    return bulletboxesnode

def genBulletCDBox(obstaclecm, name = 'autogen'):
    """
    generate a bullet cd obj using the AABB boundary of a obstacle collision model

    :param obstaclecm: a collision model
    :return: bulletrigidbody

    author: weiwei
    date: 20190313, toyonaka
    """

    if obstaclecm.type is not "box":
        raise Exception("Wrong obstaclecm type! Box is required to genBulletCDBox.")
    bulletboxesnode = BulletRigidBodyNode(name)
    bulletboxshape = BulletBoxShape.makeFromSolid(obstaclecm.cdcn.getSolid(0))
    bulletboxesnode.addShape(bulletboxshape, TransformState.makeMat(obstaclecm.getMat()))
    return bulletboxesnode

def checkcmboxcd(objcm1, objcm2, toggleplot = False):
    """
    detect the collision between collision models

    WARNING toggleplot leads to memory problems, debug only

    :return: True or False

    author: weiwei, Toyonaka
    date: 20190312
    """

    oocnp = NodePath("collision nodepath")
    obj1cnp = objcm1.cdboxrpTo(oocnp)
    obj2cnp = objcm2.cdboxrpTo(oocnp)
    oocnp.reparentTo(base.render)
    if toggleplot:
        obj1cnp.show()
        obj2cnp.show()
    ctrav = CollisionTraverser()
    chan = CollisionHandlerQueue()
    ctrav.addCollider(obj1cnp, chan)
    ctrav.traverse(oocnp)
    if chan.getNumEntries() > 0:
        return True
    else:
        return False

def checkcmcd(objcm1, objcm2, toggleplot = False):
    """
    detect the collision between collision models

    :return: True or False

    author: weiwei, Toyonaka
    date: 20190312
    """

    oocnp = NodePath("collision nodepath")
    obj1cnp = objcm1.copycdnpTo(oocnp)
    obj2cnp = objcm2.copycdnpTo(oocnp)
    if toggleplot:
        oocnp.reparentTo(base.render)
        obj1cnp.show()
        obj2cnp.show()
    ctrav = CollisionTraverser()
    chan = CollisionHandlerQueue()
    ctrav.addCollider(obj1cnp, chan)
    ctrav.traverse(oocnp)
    if chan.getNumEntries() > 0:
        return True
    else:
        return False

def checkcmcdlist(objcm, objcmlist, toggleplot = False):
    """
    detect the collision between a collision model and a collision model list

    :return: True or False

    author: weiwei, Toyonaka
    date: 20190312
    """

    oocnp = NodePath("collision nodepath")
    objcnp = objcm.copycdnpTo(oocnp)
    objcnplist = []
    for objcm2 in objcmlist:
        objcnplist.append(objcm2.copycdnpTo(oocnp))
    if toggleplot:
        oocnp.reparentTo(base.render)
        objcnp.show()
        for obj2cnp in objcnplist:
            obj2cnp.show()
    ctrav = CollisionTraverser()
    chan = CollisionHandlerQueue()
    ctrav.addCollider(objcnp, chan)
    ctrav.traverse(oocnp)
    if chan.getNumEntries() > 0:
        return True
    else:
        return False

def checkcmcdlistlist(objcmlist0, objcmlist1, toggleplot = False):
    """
    detect the collision between two collision model lists

    :return: True or False

    author: weiwei, Toyonaka
    date: 20190422
    """

    oocnp0 = NodePath("collision nodepath")
    oocnp1 = NodePath("collision nodepath")
    obj0cnplist = []
    for objcm0 in objcmlist0:
        obj0cnplist.append(objcm0.copycdnpTo(oocnp0))
    obj1cnplist = []
    for objcm1 in objcmlist1:
        obj1cnplist.append(objcm1.copycdnpTo(oocnp1))
    if toggleplot:
        oocnp0.reparentTo(base.render)
        oocnp1.reparentTo(base.render)
        for obj0cnp in obj0cnplist:
            obj0cnp.show()
        for obj1cnp in obj1cnplist:
            obj1cnp.show()
    ctrav = CollisionTraverser()
    chan = CollisionHandlerQueue()
    for obj0cnp in obj0cnplist:
        ctrav.addCollider(obj0cnp, chan)
    ctrav.traverse(oocnp1)
    if chan.getNumEntries() > 0:
        return True
    else:
        return False

if __name__ == '__main__':
    import utiltools.robotmath as rm
    import numpy as np
    import environment.ur3dbunri as ur3dbunri
    import pandaplotutils.pandactrl as pc
    import robotsim.ur3dual.ur3dual as ur3dualsim
    import robotsim.ur3dual.ur3dualmesh as ur3dualsimmesh
    import manipulation.grip.robotiq85.rtq85nm as rtq85nm

    base = pc.World(camp=[2700,300,2700], lookatp=[0,0,1000])
    env = ur3dbunri.Env()
    env.reparentTo(base.render)
    objcm = env.loadobj("bunnysim.stl")

    objcm.setColor(.2,.5,0,1)
    objcm.setPos(400,-200,1200)
    objcm.reparentTo(base.render)
    objcm.showcn()
    obscmlist = env.getstationaryobslist()
    for obscm in obscmlist:
        obscm.showcn()

    objpos = np.array([400,-300,1200])
    objrot = rm.rodrigues([0,1,0], 45)
    objcm2 = env.loadobj("housing.stl")
    objcm2.setColor(1,.5,0,1)
    env.addchangableobs(base.render, objcm2, objpos, objrot)

    robotsim = ur3dualsim.Ur3DualRobot()
    rgthnd = rtq85nm.newHand(hndid = "rgt")
    lfthnd = rtq85nm.newHand(hndid = "lft")
    robotmeshgen = ur3dualsimmesh.Ur3DualMesh(rgthand = rgthnd, lfthand = lfthnd)
    robotmesh = robotmeshgen.genmnp(robotsim, toggleendcoord=False)
    robotmesh.reparentTo(base.render)

    genBulletCDBoxes(obscmlist)


    base.run()