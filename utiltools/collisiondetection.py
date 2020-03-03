from panda3d.core import TransformState
from panda3d.core import Vec3
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape

def rayHit(pfrom, pto, geom):
    """
    NOTE: this function is quite slow
    find the nearest BulletCD point between vec(pto-pfrom) and the mesh of nodepath

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
    bullettmshape.setMargin(0)
    facetmeshnode.addShape(bullettmshape)
    bulletworld.attachRigidBody(facetmeshnode)
    result = bulletworld.rayTestClosest(pfrom, pto)

    if result.hasHit():
        return result.getHitPos()
    else:
        return None

def genBulletCDMeshNp(nodepath, basenodepath=None, name='autogen'):
    """
    generate the BulletCD mesh of a nodepath using nodepath
    this function suppose the nodepath is a single model with one geomnode

    :param nodepath: the panda3d nodepath of the object
    :param basenodepath: the nodepath to compute relative transform, identity if none
    :param name: the name of the rigidbody
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    geomnodepath = nodepath.find("**/+GeomNode")
    geombullnode = BulletRigidBodyNode(name)
    geom = geomnodepath.node().getGeom(0)
    geomtf = nodepath.getTransform(base.render)
    if basenodepath is not None:
        geomtf = nodepath.getTransform(basenodepath)
    geombullmesh = BulletTriangleMesh()
    geombullmesh.addGeom(geom)
    bullettmshape = BulletTriangleMeshShape(geombullmesh, dynamic=True)
    bullettmshape.setMargin(0)
    geombullnode.addShape(bullettmshape, geomtf)
    return geombullnode

def genBulletCDMeshMultiNp(nodepath, basenodepath=None, name='autogen'):
    """
    generate the BulletCD mesh of a nodepath using nodepath
    this function suppose the nodepath has multiple models with many geomnodes

    use genBulletCDMeshMultiNp instead of genBulletCDMeshNp for generality

    :param nodepath: the panda3d nodepath of the object
    :param basenodepath: the nodepath to compute relative transform, identity if none
    :param name: the name of the rigidbody
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    gndcollection = nodepath.findAllMatches("**/+GeomNode")
    geombullnode = BulletRigidBodyNode(name)
    for gnd in gndcollection:
        geom = gnd.node().getGeom(0)
        geomtf = gnd.getTransform(base.render)
        if basenodepath is not None:
            geomtf = gnd.getTransform(basenodepath)
        geombullmesh = BulletTriangleMesh()
        geombullmesh.addGeom(geom)
        bullettmshape = BulletTriangleMeshShape(geombullmesh, dynamic=True)
        bullettmshape.setMargin(0)
        geombullnode.addShape(bullettmshape, geomtf)
    return geombullnode

def genBulletCDMeshGeom(geom, name='autogen'):
    """
    generate the BulletCD mesh of a nodepath using geom

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

def genBulletCDPlane(updirection = Vec3(0,0,1), offset = 0, name = 'autogen'):
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
    bulletplshape = BulletPlaneShape(Vec3(0, 0, 1), offset)
    bulletplshape.setMargin(0)
    bulletplnode.addShape(bulletplshape)
    return bulletplnode

def genBulletCDBoxes(obstaclecmlist, name = 'autogen'):
    """
    generate a bullet cd obj using a list of AABB boxes generated from obstaclenplist

    :param obstaclenplist: the AABB box of each obstaclenp in the list will be computed and used for cd
    :return: bulletrigidbody

    author: weiwei
    date: 20180807, osaka
    """

    bulletboxesnode = BulletRigidBodyNode(name)
    for obstaclecm in obstaclecmlist:
        if obstaclecm.type is not "box":
            raise Exception("The type of obstaclecms must be box!")
        cdsolid = obstaclecm.cdcn.getSolid(0)
        bulletboxshape = BulletBoxShape.makeFromSolid(cdsolid)
        bulletboxesnode.addShape(bulletboxshape,
                                 TransformState.makeMat3(obstaclecm.getMat().getUpper3()).
                                 setPos(obstaclecm.getPos()+cdsolid.getCenter()))
    return bulletboxesnode

def genBulletCDBox(obstaclecm, name = 'autogen'):
    """
    generate a bullet cd obj using a list of AABB box generated from obstaclenp

    :param obstaclenp: the AABB box of the obstaclenp will be computed and used for cd
    :return: bulletrigidbody

    author: weiwei
    date: 20190126, osaka
    """

    bulletboxesnode = BulletRigidBodyNode(name)
    bulletboxshape = BulletBoxShape.makeFromSolid(obstaclecm.cdcn.getSolid(0))
    bulletboxesnode.addShape(bulletboxshape, TransformState.makeMat(obstaclecm.getMat()))
    return bulletboxesnode

if __name__=="main":
    import numpy as np
    import pandaplotutils.pandageom as pg
    import trimesh.primitives as tp
    import utiltools.robotmath as rm

    tribox = pg.trimeshToPanda(tp.Box(box_center=np.array([0,0,0]), box_transform=rm.rodrigues([0,0,1],30)))
