import os
import database.dbaccess as db
import pandaplotutils.pandactrl as pandactrl
import manipulation.grip.robotiq85.rtq85nm as rtq85nm
import manipulation.grip.freegrip as freegrip


if __name__=='__main__':

    from panda3d.core import *

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,100])
    this_dir, this_filename = os.path.split(__file__)
    print(this_dir)
    # objpath = os.path.join(this_dir, "objects", "housing.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "tool_motordriver.stl")
    # objpath = os.path.join(this_dir, "objects", "housingshaft.stl")
    objpath = os.path.join(this_dir, "objects", "bunnysim.stl")

    handpkg = rtq85nm
    freegriptst = freegrip.Freegrip(objpath, handpkg, readser=False, faceangle = .8, segangle= .9, hmax=1.0)

    freegriptst.segShow(base, togglesamples=False, togglenormals=False,
                        togglesamples_ref=False, togglenormals_ref=False,
                        togglesamples_refcls=False, togglenormals_refcls=False)

    # freegriptst.removeFgrpcc(base)
    # freegriptst.removeHndcc(base, discretesize = 16)
    freegriptst.planGrasps(base, discretesize=8)

    # use wrs database for ur3d
    gdb = db.GraspDB(database="wrs")
    # use nxt database for nextage!
    # gdb = db.GraspDB(database="nxt")
    freegriptst.saveToDB(gdb)
    #
    # def updateshow(task):
    #     # freegriptst.removeFgrpccShow(base)
    #     # freegriptst.removeFgrpccShowLeft(base)
    #     freegriptst.removeHndccShow(base)
    # #     # print task.delayTime
    # #     # if abs(task.delayTime-13) < 1:
    # #     #     task.delayTime -= 12.85
    #     return task.again
    #
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")
    # taskMgr.add(updateshow, "tickTask")
    # freegriptst.removeFgrpcc(base)
    # freegriptst.removeHndcc(base)

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont
    #
    # base.taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)
    #
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNode.showConstraints(True)
    # debugNode.showBoundingBoxes(False)
    # debugNode.showNormals(False)
    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    # freegriptst.bulletworld.setDebugNode(debugNP.node())
    # taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)

    # freegriptst.showAllGrips()

    data = gdb.loadFreeAirGrip('bunnysim', 'rtq85')
    if data:
        freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth = data
        print(len(freegripid))
        for i, freegriprotmat in enumerate(freegriprotmats):
            rtqhnd = rtq85nm.Rtq85NM(hndcolor=[1, 1, 1, .3])
            newpos = freegriprotmat.getRow3(3)-freegriprotmat.getRow3(2)*0.0
            freegriprotmat.setRow(3, newpos)
            rtqhnd.setMat(pandanpmat4=freegriprotmat)
            rtqhnd.setJawwidth(freegripjawwidth[i])
            pandamat4 = rtqhnd.handnp.getMat()
            # x = pandamat4.getRow3(0)
            # if (Vec3(1,0,0).angleDeg(x)-45) < 15:
            rtqhnd.reparentTo(base.render)
            # base.pggen.plotAxis(base.render, spos=rtqhnd.handnp.getPos(),pandamat4=rtqhnd.handnp.getMat())
            #     base.pggen.plotSphere(base.render, freegripcontacts[i][0], radius = 10, rgba = [1.0,0,0,1])
            #     base.pggen.plotArrow(base.render, spos = freegripcontacts[i][0], epos = freegripcontacts[i][0]+freegripnormals[i][0], thickness = 5, rgba = [1.0,0,0,1])
            #     base.pggen.plotSphere(base.render, freegripcontacts[i][1], radius = 10, rgba = [0,1,0,1])
            #     base.run()
    base.run()