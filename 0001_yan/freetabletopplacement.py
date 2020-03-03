import os
import numpy as np
import database.dbaccess as db
import pandaplotutils.pandactrl as pandactrl
import manipulation.regrasp.freetabletopplacement as fttp
# import manipulation.grip.robotiq85.rtq85nm as rtq85nm
import manipulation.grip.schunk918.sck918 as sck918

if __name__ == '__main__':

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "housing.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "tool_motordriver.stl")
    # objpath = os.path.join(this_dir, "objects", "bunnysim.stl")
    # objpath = os.path.join(this_dir, "objects", "new_LSHAPE.stl")
    # objpath = os.path.join(this_dir, "objects", "candy.STL")
    objpath = os.path.join(this_dir, "objects", "twostairpeg_handle.STL")

    # handpkg = rtq85nm
    handpkg = sck918
    # use wrs database for ur3d
    # gdb = db.GraspDB(database="wrs")
    # use nxt database for nextage!
    gdb = db.GraspDB(database="nxt")
    tps = fttp.FreeTabletopPlacement(objpath, handpkg, gdb)

    # # plot obj and its convexhull
    # geom = pandageom.packpandageom(tps.objtrimesh.vertices,
    #                                tps.objtrimesh.face_normals,
    #                                tps.objtrimesh.faces)
    # node = GeomNode('obj')
    # node.addGeom(geom)
    # star = NodePath('obj')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,1,0,1))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    # geom = pandageom.packpandageom(tps.objtrimeshconv.vertices,
    #                                tps.objtrimeshconv.face_normals,
    #                                tps.objtrimeshconv.faces)
    # node = GeomNode('objconv')
    # node.addGeom(geom)
    # star = NodePath('objconv')
    # star.attachNewNode(node)
    # star.setColor(Vec4(0, 1, 0, .3))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    # pg.plotSphere(base.render, pos=tps.objcom, radius=10, rgba=[1,0,0,1])
    # def updateshow(task):
    #     # tps.ocfacetshow(base)
    #     tps.removebadfacetsshow(base, doverh=.1)
    #     return task.again
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    tps.genTpsandGrips(base, doverh=.15)
    tps.saveToDB()
    #
    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    #
    # tps.bulletworldhp.setDebugNode(debugNP.node())
    #
    # taskMgr.add(updateworld, "updateworld", extraArgs=[tps.bulletworldhp], appendTask=True)

    tps.showOnePlacementAndAssociatedGrips(base, 0)
    base.run()
