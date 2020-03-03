import trimesh
import trimesh.sample as sample
import copy
import math
import os
import numpy as np
import utiltools.robotmath as rm
from panda3d.core import CollisionNode
from panda3d.core import CollisionBox
from panda3d.core import CollisionSphere
from panda3d.core import TransparencyAttrib
from panda3d.core import NodePath
import environment.collisionmodelcollection as cmc

class CollisionModel(object):
    """
    load an object as a collision model
    both collison boxes and circular samples will be generated automatically

    the difference bewteen this one and share is
    this one generates a single type, either box or ball

    author: weiwei
    date: 20190312
    """

    def __init__(self, objinit, betransparency = True, type = "box", radius=None, name="defaultname"):
        """
        :param objinit: path type defined by os.path or trimesh or nodepath
        :param radius box expansion size for box type; ball radius for ball type (30 is good)
        """

        if type not in ["box", "ball"]:
            raise Exception("Wrong Collision Model type name.")

        self.__type = type
        if isinstance(objinit, str):
            self.__objpath = objinit
            self.__trimesh = trimesh.load_mesh(self.__objpath)
            self.__objnp = base.pg.loadstlaspandanp_fn(self.__objpath)
            self.__name = os.path.splitext(os.path.basename(self.__objpath))[0]
        elif isinstance(objinit, trimesh.Trimesh):
            self.__objpath = None
            self.__trimesh = objinit
            self.__objnp = base.pg.trimeshtonp(objinit)
            self.__name = name
        elif isinstance(objinit, NodePath):
            self.__objpath = None
            self.__trimesh = None
            self.__objnp = objinit
            self.__name = name
        if betransparency:
            self.__objnp.setTransparency(TransparencyAttrib.MDual)
        if type is "ball":
            if radius is None:
                radius = 15.0
            self.__cdcn = self.__gencdballcn(self.__trimesh, radius = radius)
        else:
            if radius is None:
                radius = 2.0
            self.__cdcn = self.__gencdboxcn(self.__objnp, radius = radius)
        self.__cdnp = self.__objnp.attachNewNode(self.__cdcn)

        self.__localframe = None

    @property
    def type(self):
        # read-only property
        return self.__type

    @property
    def name(self):
        # read-only property
        return self.__name

    @property
    def objpath(self):
        # read-only property
        return self.__objpath

    @property
    def objnp(self):
        # read-only property
        return self.__objnp

    @property
    def trimesh(self):
        # read-only property
        return self.__trimesh

    @property
    def com(self):
        # read-only property
        return self.__trimesh.center_mass

    @property
    def volume(self):
        # read-only property
        return self.__trimesh.volume

    @property
    def cdcn(self):
        # read-only property
        return self.__cdcn

    def __gencdboxcn(self, pandanp, name='boxcd', radius=15.0):
        """

        :param obstacle:
        :return:

        author: weiwei
        date: 20180811
        """

        cnp = CollisionNode(name)
        bottomLeft, topRight = pandanp.getTightBounds()
        center = (bottomLeft + topRight) / 2.0
        # enlarge the bounding box
        bottomLeft -= (bottomLeft - center).normalize() * radius
        topRight += (topRight - center).normalize() * radius
        cbn = CollisionBox(bottomLeft, topRight)
        cnp.addSolid(cbn)
        return cnp

    def __gencdballcn(self, trimeshmodel, name='ballcd', radius = 30):
        """

        :param obstacle:
        :return:

        author: weiwei
        date: 20180811
        """
        cnp = CollisionNode(name)
        nsample = int(math.ceil(trimeshmodel.area/(radius**2/3.0)))
        if nsample > 120:
            nsample = 120
        samples = sample.sample_surface_even(trimeshmodel, nsample)
        for sglsample in samples:
            cnp.addSolid(CollisionSphere(sglsample[0], sglsample[1], sglsample[2], radius=radius))
        return cnp

    def copycdnpTo(self, nodepath):
        """
        return a nodepath including the cdcn,
        the returned nodepath is attached to the given one

        :param nodepath: parent np
        :return:
        """
        returnnp = nodepath.attachNewNode(copy.deepcopy(self.__cdcn))
        returnnp.setMat(self.__objnp.getMat())
        return returnnp

    def setColor(self, r, g, b, a):
        self.__objnp.setColor(r, g, b, a)

    def clearColor(self):
        self.__objnp.clearColor()

    def getColor(self):
        return self.__objnp.getColor()

    def setTransparency(self, attribute):
        return self.__objnp.setTransparency(attribute)

    def setPos(self, x, y, z):
        self.__objnp.setPos(x, y, z)

    def getPos(self, rel = None):
        if rel is None:
            return self.__objnp.getPos()
        else:
            return self.__objnp.getPos(rel)

    def setMat(self, pandamat4):
        self.__objnp.setMat(pandamat4)

    def sethomomat(self, npmat4):
        """
        helper function that accepts numpy homo mat

        :param npmat4:
        :return:

        author: weiwei
        date: 20190628
        """
        self.__objnp.setMat(base.pg.np4ToMat4(npmat4))

    def setRPY(self, roll, pitch, yaw):
        """
        set the pose of the object using rpy

        :param roll: degree
        :param pitch: degree
        :param yaw: degree
        :return:

        author: weiwei
        date: 20190513
        """

        currentmat = self.__objnp.getMat()
        currentmatnp = base.pg.mat4ToNp(currentmat)
        newmatnp = rm.euler_matrix(roll, pitch, yaw, axes="sxyz")
        self.__objnp.setMat(base.pg.npToMat4(newmatnp, currentmatnp[:,3]))

    def getRPY(self):
        """
        get the pose of the object using rpy

        :return: [r, p, y] in degree

        author: weiwei
        date: 20190513
        """

        currentmat = self.__objnp.getMat()
        currentmatnp = base.pg.mat4ToNp(currentmat)
        rpy = rm.euler_from_matrix(currentmatnp[:3,:3], axes="sxyz")
        return np.array([rpy[0], rpy[1], rpy[2]])

    def getMat(self, rel=None):
        if rel is None:
            return self.__objnp.getMat()
        else:
            return self.__objnp.getMat(rel)

    def gethomomat(self, rel=None):
        pandamat4 = self.getMat(rel)
        return base.pg.mat4ToNp(pandamat4)

    def reparentTo(self, obj):
        if isinstance(obj, CollisionModel):
            self.__objnp.reparentTo(obj.objnp)
        elif isinstance(obj, cmc.CollisionModelCollection):
            obj.addcm(self)
        elif isinstance(obj, NodePath):
            self.__objnp.reparentTo(obj)
        else:
            print("NodePath.reparent_to() argument 1 must be environment.CollisionModel or panda3d.core.NodePath")

    def removeNode(self):
        self.__objnp.removeNode()

    def detachNode(self):
        self.__objnp.detachNode()

    def showcn(self):
        # reattach to bypass the failure of deepcopy
        self.__cdnp.removeNode()
        self.__cdnp = self.__objnp.attachNewNode(self.__cdcn)
        self.__cdnp.show()

    def showLocalFrame(self):
        self.__localframe = base.pggen.genAxis()
        self.__localframe.reparentTo(self.objnp)

    def unshowLocalFrame(self):
        if self.__localframe is not None:
            self.__localframe.removeNode()
            self.__localframe = None

    def unshowcn(self):
        self.__cdnp.hide()

if __name__=="__main__":
    import environment.pandacdhelper as cmcd
    import os
    import numpy as np
    import utiltools.robotmath as rm
    import pandaplotutils.pandactrl as pc

    base = pc.World(camp=[1000,300,1000], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "objects", "bunnysim.stl")
    bunnycm = CollisionModel(objpath, type="box")
    bunnycm.setColor(0.7, 0.7, 0.0, 1.0)
    bunnycm.reparentTo(base.render)
    rotmat = rm.rodrigues([1,0,0], 90)
    bunnycm.setMat(base.pg.npToMat4(rotmat))

    bunnycm1 = CollisionModel(objpath, type="box")
    bunnycm1.setColor(0.7, 0, 0.7, 1.0)
    bunnycm1.reparentTo(base.render)
    # rotmat = rm.rodrigues([0,0,1], 15)
    rotmat = rm.euler_matrix(0,0,15)
    bunnycm1.setMat(base.pg.npToMat4(rotmat, np.array([0,10,0])))

    bunnycm2 = CollisionModel(objpath, type="box")
    bunnycm2.setColor(0, 0.7, 0.7, 1.0)
    bunnycm2.reparentTo(base.render)
    rotmat = rm.rodrigues([1,0,0], -45)
    bunnycm2.setMat(base.pg.npToMat4(rotmat, np.array([0,200,0])))

    print(cmcd.isCmListCmListCollided([bunnycm, bunnycm1], [bunnycm2], toggleplot=True))
    base.run()
