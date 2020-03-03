#!/usr/bin/python

import os
import database.dbaccess as db
import manipulation.grip.robotiq85.rtq85nm as rtq85nm
import manipulation.regrasp.floatingposes as fps
import robotsim.ur3dual.ur3dual as ur3dualsim
import robotsim.nextage.nxt as nxtsim

if __name__=="__main__":

    # show in panda3d
    from panda3d.core import *
    import pandaplotutils.pandactrl as pandactrl

    base = pandactrl.World(lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "housing.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "tool_motordriver.stl")
    objpath = os.path.join(this_dir, "objects", "bunnysim.stl")

    handpkg = rtq85nm

    ## use wrs database for ur3d
    gdb = db.GraspDB(database="wrs")
    ## use nxt database for nextage!
    # gdb = db.GraspDB(database="nxt")

    fpose = fps.FloatingPoses(objpath, gdb, handpkg, base)

    ## usage:
    ## genFPGPandSaveToDB generate floating poses and grasps and saves them
    ## updatgeDBwithIK
    ## use the following grids for ur3d
    # grids = []
    # for x in range(400,401,100):
    #     for y in [0]:
    #         for z in range(1430,1431,400):
    #             grids.append([x,y,z])
    # fpose.genFPGPandSaveToDB(grids, angles=[0,180])
    # robsim = ur3dualsim.Ur3DualRobot()
    # fpose.updateDBwithIK(robot=robsim)
    ## use the following grids for nxt
    grids = []
    for x in range(300,301,100):
        for y in [0]:
            for z in range(1300,1301,400):
                grids.append([x,y,z])
    fpose.genFPGPandSaveToDB(grids, angles=[0,90,180,270])
    robsim = nxtsim.NxtRobot()
    fpose.updateDBwithIK(robot=robsim)

    base.run()
