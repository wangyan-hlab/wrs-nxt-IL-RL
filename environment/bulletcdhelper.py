from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape
from panda3d.core import TransformState, Vec3, GeomVertexRewriter
import copy
import _ctypes

class BCMchecker(object):
    """
    BBCMchecker bullet box collision model checker
    """

    def __init__(self, toggledebug = False):
        self.world = BulletWorld()
        self._toggledebug = toggledebug
        if self._toggledebug:
            bulletcolliderrender = base.render.attachNewNode("bulletboxcollider")
            debugNode = BulletDebugNode('Debug')
            debugNode.showWireframe(True)
            debugNode.showConstraints(True)
            debugNode.showBoundingBoxes(False)
            debugNode.showNormals(False)
            self._debugNP = bulletcolliderrender.attachNewNode(debugNode)
            self.world.setDebugNode(self._debugNP.node())

        self.worldrigidbodylist = []
        self._isupdateworldadded = False

    def _updateworld(self, world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont

    def isBoxBoxCollided(self, objcm0, objcm1):
        """
        check if two objects objcm0 as objcm1 are in collision with each other
        the two objects are in the form of collision model
        the AABB boxlist will be used
        type "box" is required

        :param objcm0: the first object
        :param objcm1: the second object
        :return: boolean value showing if the two objects are in collision

        author: weiwei
        date: 20190313
        """

        objcm0boxbullnode = genBulletCDBox(objcm0)
        objcm1boxbullnode = genBulletCDBox(objcm1)
        result = self.world.contactTestPair(objcm0boxbullnode, objcm1boxbullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def isBoxBoxListCollided(self, objcm0, objcmlist=[]):
        """
        check if objcm0 is in collision with a list of collisionmodels in objcmlist
        each obj is in the form of a collision model

        :param objcm0:
        :param obcmjlist: a list of collision models
        :return: boolean value showing if the object and the list are in collision

        author: weiwei
        date: 20190313
        """

        objcm0boxbullnode = genBulletCDBox(objcm0)
        objcm1boxbullnode = genBulletCDBoxList(objcmlist)
        result = self.world.contactTestPair(objcm0boxbullnode, objcm1boxbullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def isBoxListBoxListCollided(self, objcm0list=[], objcm1list=[]):
        """
        check if a list of collisionmodels in objcm0list is in collision with a list of collisionmodels in objcm1list
        each obj is in the form of a collision model

        :param objcm0list: a list of collision models
        :param obcmj1list: a list of collision models
        :return: boolean value showing if the object and the list are in collision

        author: weiwei
        date: 20190313
        """

        objcm0boxbullnode = genBulletCDBoxList(objcm0list)
        objcm1boxbullnode = genBulletCDBoxList(objcm1list)
        result = self.world.contactTestPair(objcm0boxbullnode, objcm1boxbullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def showBox(self, objcm):
        """
        show the AABB collision meshes of the given objects

        :param objcm

        author: weiwei
        date: 20190313
        :return:
        """

        if not self._toggledebug:
            print("Toggle debug on during defining the XCMchecker object to use showfunctions!")
            return

        if not self._isupdateworldadded:
            base.taskMgr.add(self._updateworld, "updateworld", extraArgs=[self.world], appendTask=True)

        objcmboxbullnode = genBulletCDBox(objcm)
        self.world.attach(objcmboxbullnode)
        self.worldrigidbodylist.append(objcmboxbullnode)
        self._debugNP.show()

    def showBoxList(self, objcmlist):
        """
        show the AABB collision meshes of the given objects

        :param objcm0, objcm1

        author: weiwei
        date: 20190313
        :return:
        """

        if not self._toggledebug:
            print("Toggle debug on during defining the XCMchecker object to use showfunctions!")
            return

        if not self._isupdateworldadded:
            base.taskMgr.add(self._updateworld, "updateworld", extraArgs=[self.world], appendTask=True)

        objcmboxbullnode = genBulletCDBoxList(objcmlist)
        self.world.attach(objcmboxbullnode)
        self.worldrigidbodylist.append(objcmboxbullnode)
        self._debugNP.show()

    def unshow(self):
        """
        unshow everything

        author: weiwei
        date: 20180621
        :return:
        """

        base.taskMgr.remove("updateworld")
        print(self.worldrigidbodylist)
        for cdnode in self.worldrigidbodylist:
            self.world.remove(cdnode)
        self.worldrigidbodylist = []
        self._debugNP.hide()

class MCMchecker(BCMchecker):
    """
    Mesh Collision Checker
    """

    def __init__(self, toggledebug = False):
        super().__init__(toggledebug)

    def isMeshMeshCollided(self, objcm0, objcm1):
        """
        check if two objects objcm0 and objcm1 are in collision with each other
        the two objects are in the form of collision model
        the bulletmeshes will be used

        :param objcm0: the first object
        :param objcm1: the second object
        :return: boolean value showing if the two objects are in collision

        author: weiwei
        date: 20190313
        """

        objcm0bullnode = genBulletCDMesh(objcm0)
        objcm1bullnode = genBulletCDMesh(objcm1)
        result = self.world.contactTestPair(objcm0bullnode, objcm1bullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def isMeshMeshListCollided(self, objcm0, objcm1list):
        """
        check if object objcm0 and objectlist objcm1list are in collision with each other
        the two objects are in the form of collision model
        the bulletmeshes will be used

        :param objcm0: the first collision model
        :param objcm1list: the second collision model list
        :return: boolean value showing if the object and object list are in collision

        author: weiwei
        date: 20190514
        """

        objcm0bullnode = genBulletCDMesh(objcm0)
        objcm1bullnode = genBulletCDMeshList(objcm1list)
        result = self.world.contactTestPair(objcm0bullnode, objcm1bullnode)
        if not result.getNumContacts():
            print("No collision.")
            return False
        else:
            print("Collision detected!!!")
            # print("Contact number =", result.getNumContacts())
            # contactpoints = result.getContacts()
            return True

    def isMeshListMeshListCollided(self, objcm0list, objcm1list):
        """
        check if two object lists objcm0list and objcm1list are in collision with each other
        the two objects are in the form of collision model
        the bulletmeshes will be used

        :param objcm0list: the first collision model list
        :param objcm1list: the second collision model list
        :return: boolean value showing if the two objects are in collision

        author: weiwei
        date: 20190514
        """

        objcm0bullnode = genBulletCDMeshList(objcm0list)
        objcm1bullnode = genBulletCDMeshList(objcm1list)
        result = self.world.contactTestPair(objcm0bullnode, objcm1bullnode)
        if not result.getNumContacts():
            return False
        else:
            return True

    def isRayHitMeshClosest(self, pfrom, pto, objcm):
        """

        :param pfrom:
        :param pto:
        :param objcm:
        :return:

        author: weiwei
        date: 20190805
        """

        geom = base.pg.packpandageom_fn(objcm.trimesh.vertices, objcm.trimesh.face_normals, objcm.trimesh.faces)
        targetobjmesh = BulletTriangleMesh()
        targetobjmesh.addGeom(geom)
        bullettmshape = BulletTriangleMeshShape(targetobjmesh, dynamic=True)
        bullettmshape.setMargin(1e-6)
        targetobjmeshnode = BulletRigidBodyNode('facet')
        targetobjmeshnode.addShape(bullettmshape)
        self.world.attach(targetobjmeshnode)
        result = self.world.rayTestClosest(base.pg.npToV3(pfrom), base.pg.npToV3(pto))
        self.world.removeRigidBody(targetobjmeshnode)
        if result.hasHit():
            return [result.getHitPos(), -result.getHitNormal()]
        else:
            return [None, None]

    def isRayHitMeshAll(self, pfrom, pto, objcm):
        """

        :param pfrom:
        :param pto:
        :param objcm:
        :return:

        author: weiwei
        date: 20190805
        """

        geom = base.pg.packpandageom_fn(objcm.trimesh.vertices, objcm.trimesh.face_normals, objcm.trimesh.faces)
        targetobjmesh = BulletTriangleMesh()
        targetobjmesh.addGeom(geom)
        bullettmshape = BulletTriangleMeshShape(targetobjmesh, dynamic=True)
        bullettmshape.setMargin(1e-6)
        targetobjmeshnode = BulletRigidBodyNode('facet')
        targetobjmeshnode.addShape(bullettmshape)
        self.world.attach(targetobjmeshnode)
        result = self.world.rayTestAll(base.pg.npToV3(pfrom), base.pg.npToV3(pto))
        self.world.removeRigidBody(targetobjmeshnode)
        if result.hasHits():
            allhits = []
            for hit in result.getHits():
                allhits.append([hit.getHitPos(), -hit.getHitNormal()])
            return allhits
        else:
            return []

    def showMesh(self, objcm):
        """
        show the collision meshes of the given objects

        :param objcm environment.collisionmodel

        author: weiwei
        date: 20190313
        :return:
        """

        if not self._toggledebug:
            print("Toggle debug on during defining the XCMchecker object to use showfunctions!")
            return

        if not self._isupdateworldadded:
            self.taskMgr.add(self._updateworld, "updateworld", extraArgs=[self.world], appendTask=True)

        objcmmeshbullnode = genBulletCDMesh(objcm)
        self.world.attach(objcmmeshbullnode)
        self.worldrigidbodylist.append(objcmmeshbullnode)
        self._debugNP.show()

    def showMeshList(self, objcmlist):
        """
        show the collision meshes of the given objects

        :param objcmlist environment.collisionmodel

        author: weiwei
        date: 20190313
        :return:
        """

        if not self._toggledebug:
            print("Toggle debug on during defining the XCMchecker object to use showfunctions!")
            return

        if not self._isupdateworldadded:
            base.taskMgr.add(self._updateworld, "updateworld", extraArgs=[self.world], appendTask=True)

        objcmmeshbullnode = genBulletCDMeshList(objcmlist)
        self.world.attach(objcmmeshbullnode)
        self.worldrigidbodylist.append(objcmmeshbullnode)
        self._debugNP.show()

def di(obj_id):
    """ Inverse of id() function. """
    return _ctypes.PyObj_FromPtr(obj_id)

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

    bulletboxnode = BulletRigidBodyNode(name)
    cdsolid = obstaclecm.cdcn.getSolid(0)
    bulletboxshape = BulletBoxShape.makeFromSolid(cdsolid)
    rotmat4_pd = obstaclecm.getMat(base.render)
    bulletboxnode.addShape(bulletboxshape,
                               TransformState.makeMat(rotmat4_pd).
                               setPos(rotmat4_pd.xformPoint(cdsolid.getCenter())))
    return bulletboxnode

def genBulletCDBoxList(obstaclecmlist, name = 'autogen'):
    """
    generate a bullet cd obj using the AABB boundaries stored in obstacle collision models

    :param obstaclecmlist: a list of collision models (cmshare doesnt work!)
    :return: bulletrigidbody

    author: weiwei
    date: 20190313, toyonaka
    """

    bulletboxlistnode = BulletRigidBodyNode(name)
    for obstaclecm in obstaclecmlist:
        if obstaclecm.type is not "box":
            raise Exception("Wrong obstaclecm type! Box is required to genBulletCDBox.")
        cdsolid = obstaclecm.cdcn.getSolid(0)
        bulletboxshape = BulletBoxShape.makeFromSolid(cdsolid)
        rotmatpd4 = obstaclecm.getMat(base.render)
        bulletboxlistnode.addShape(bulletboxshape,
                                 TransformState.makeMat(rotmatpd4).
                                 setPos(rotmatpd4.xformPoint(cdsolid.getCenter())))
    return bulletboxlistnode

def genBulletCDMesh(objcm, basenodepath=None, name='autogen'):
    """
    generate the collision mesh of a nodepath using nodepath
    this function suppose the nodepath has multiple models with many geomnodes

    use genCollisionMeshMultiNp instead of genCollisionMeshNp for generality

    :param nodepath: the panda3d nodepath of the object
    :param basenodepath: the nodepath to compute relative transform, identity if none
    :param name: the name of the rigidbody
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    gndcollection = objcm.objnp.findAllMatches("+GeomNode")
    geombullnode = BulletRigidBodyNode(name)
    for gnd in gndcollection:
        geom = copy.deepcopy(gnd.node().getGeom(0))
        # vdata = geom.modifyVertexData()
        # vertrewritter = GeomVertexRewriter(vdata, 'vertex')
        # while not vertrewritter.isAtEnd():
        #     v = vertrewritter.getData3f()
        #     vertrewritter.setData3f(v[0]-objcm.com[0], v[1]-objcm.com[1], v[2]-objcm.com[2])
        geomtf = gnd.getTransform(base.render)
        if basenodepath is not None:
            geomtf = gnd.getTransform(basenodepath)
        geombullmesh = BulletTriangleMesh()
        geombullmesh.addGeom(geom)
        bullettmshape = BulletTriangleMeshShape(geombullmesh, dynamic=True)
        bullettmshape.setMargin(-2)
        geombullnode.addShape(bullettmshape, geomtf)
        # rotmatpd4 = objcm.getMat(base.render)
        # geombullnode.addShape(bullettmshape,
        #                      TransformState.makeMat(rotmatpd4).
        #                      setPos(rotmatpd4.xformPoint(Vec3(objcm.com[0], objcm.com[1], objcm.com[2]))))
        from panda3d.core import Mat3, Mat4
        # geombullnode.setTransform(TransformState.makeMat(Mat4(Mat3.identMat(), rotmatpd4.xformPoint(Vec3(objcm.com[0], objcm.com[1], objcm.com[2])))))
        # print(objcm.com)
        # print(geomtf.getMat())
        # geombullnode.setTransform(TransformState.makeMat(geomtf.getMat()).setPos(geomtf.getMat().xformPoint(Vec3(objcm.com[0], objcm.com[1], objcm.com[2]))))
    return geombullnode

def genBulletCDMeshList(objcmlist, basenodepath=None, name='autogen'):
    """
    generate the collision mesh of a nodepath using nodepathlist
    this function suppose the nodepathlist is a list of models with many geomnodes
    "Multi" means each nodepath in the nodepath list may have multiple nps (parent-child relations)

    use genCollisionMeshMultiNp instead if the meshes have parent-child relations

    :param nodepathlist: panda3d nodepathlist
    :param basenodepath: the nodepath to compute relative transform, identity if none
    :param name: the name of the rigidbody
    :return: bulletrigidbody

    author: weiwei
    date: 20190514
    """

    geombullnode = BulletRigidBodyNode(name)
    for objcm in objcmlist:
        gndcollection = objcm.objnp.findAllMatches("+GeomNode")
        for gnd in gndcollection:
            geom = copy.deepcopy(gnd.node().getGeom(0))
            geomtf = gnd.getTransform(base.render)
            if basenodepath is not None:
                geomtf = gnd.getTransform(basenodepath)
            geombullmesh = BulletTriangleMesh()
            geombullmesh.addGeom(geom)
            bullettmshape = BulletTriangleMeshShape(geombullmesh, dynamic=True)
            bullettmshape.setMargin(0)
            geombullnode.addShape(bullettmshape, geomtf)
    return geombullnode

def genBullectCDMeshFromGeom(geom, name='autogen'):
    """
    generate the collision mesh of a nodepath using geom

    :param geom: the panda3d geom of the object
    :param basenodepath: the nodepath to compute relative transform
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    geomtf = TransformState.makeIdentity()
    geombullmesh = BulletTriangleMesh()
    geombullmesh.addGeom(geom)
    geombullnode = BulletRigidBodyNode(name)
    bullettmshape = BulletTriangleMeshShape(geombullmesh, dynamic=True)
    bullettmshape.setMargin(0)
    geombullnode.addShape(bullettmshape, geomtf)
    return geombullnode

def genBulleCDPlane(updirection = Vec3(0,0,1), offset = 0, name = 'autogen'):
    """
    generate a plane bulletrigidbody node

    :param updirection: the normal parameter of bulletplaneshape at panda3d
    :param offset: the d parameter of bulletplaneshape at panda3d
    :param name:
    :return: bulletrigidbody

    author: weiwei
    date: 20170202, tsukuba
    """

    bulletplnode = BulletRigidBodyNode(name)
    bulletplshape = BulletPlaneShape(updirection, offset)
    bulletplshape.setMargin(0)
    bulletplnode.addShape(bulletplshape)
    return bulletplnode

def bulletRayHit(pfrom, pto, geom):
    """
    NOTE: this function is quite slow
    find the nearest collision point between vec(pto-pfrom) and the mesh of nodepath

    :param pfrom: starting point of the ray, Point3
    :param pto: ending point of the ray, Point3
    :param geom: meshmodel, a panda3d datatype
    :return: None or Point3

    author: weiwei
    date: 20161201
    """

    bulletworld = BulletWorld()
    facetmesh = BulletTriangleMesh()
    facetmesh.addGeom(geom)
    facetmeshnode = BulletRigidBodyNode('facet')
    bullettmshape = BulletTriangleMeshShape(facetmesh, dynamic=True)
    bullettmshape.setMargin(1e-6)
    facetmeshnode.addShape(bullettmshape)
    bulletworld.attach(facetmeshnode)
    result = bulletworld.rayTestClosest(pfrom, pto)

    if result.hasHit():
        return result.getHitPos()
    else:
        return None

if __name__=='__main__':
    import environment.bunrisettingwrs as ur3dbunri
    import pandaplotutils.pandactrl as pandactrl

    pandactrl.World(camp=[2700,300,2700], lookatp=[0,0,1000])
    env = ur3dbunri.Env()
    env.reparentTo(base.render)
    objcm = env.loadobj("bunnysim.stl")

    bbcmc = BCMchecker(toggledebug=True)
    bbcmc.showBoxList(env.getstationaryobslist())
    base.run()