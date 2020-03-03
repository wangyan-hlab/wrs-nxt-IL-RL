from panda3d.core import NodePath
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionHandlerQueue
from panda3d.core import BitMask32

def isCmCmCollided(objcm1, objcm2, toggleplot = False):
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

def isCmCmListCollided(objcm, objcmlist, toggleplot = False):
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

def isCmListCmListCollided(objcmlist0, objcmlist1, toggleplot = False):
    """
    detect the collision between two collision model lists

    :return: True or False

    author: weiwei, Toyonaka
    date: 20190422
    """

    oocnp = NodePath("collision nodepath")
    obj0cnplist = []
    for objcm0 in objcmlist0:
        obj0cnplist.append(objcm0.copycdnpTo(oocnp))
    obj1cnplist = []
    for objcm1 in objcmlist1:
        obj1cnplist.append(objcm1.copycdnpTo(oocnp))
    if toggleplot:
        oocnp.reparentTo(base.render)
        for obj0cnp in obj0cnplist:
            obj0cnp.show()
        for obj1cnp in obj1cnplist:
            obj1cnp.show()
    ctrav = CollisionTraverser()
    chan = CollisionHandlerQueue()
    for obj0cnp in obj0cnplist:
        obj0cnp.node().setFromCollideMask(BitMask32(0x1))
        obj0cnp.setCollideMask(BitMask32(0x2))
        ctrav.addCollider(obj0cnp, chan)
    ctrav.traverse(oocnp)
    if chan.getNumEntries() > 0:
        return True
    else:
        return False

if __name__ == '__main__':
    import utiltools.robotmath as rm
    import numpy as np
    import environment.bunrisettingfree as bunrisettingfree
    import pandaplotutils.pandactrl as pc
    import robotsim.ur3dual.ur3dual as ur3dualsim
    import robotsim.ur3dual.ur3dualmesh as ur3dualsimmesh
    import manipulation.grip.robotiq85.robotiq85 as rtq85

    base = pc.World(camp=[2700,300,2700], lookatp=[0,0,1000])
    env = bunrisettingfree.Env()
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

    hndfa = rtq85.Robotiq85Factory()
    rgthnd = hndfa.genHand()
    lfthnd = hndfa.genHand()
    robotsim = ur3dualsim.Ur3DualRobot(rgthnd = rgthnd, lfthnd = lfthnd)
    robotmeshgen = ur3dualsimmesh.Ur3DualMesh()
    robotmesh = robotmeshgen.genmnp(robotsim, toggleendcoord=False)
    robotmesh.reparentTo(base.render)

    print(isCmCmListCollided(objcm, obscmlist))

    base.run()