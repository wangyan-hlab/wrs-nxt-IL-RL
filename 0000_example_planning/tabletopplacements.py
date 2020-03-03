import os
import database.dbaccess as db
import pandaplotutils.pandactrl as pandactrl
import manipulation.grip.robotiq85.rtq85nm as rtq85nm
import manipulation.regrasp.tabletopplacements as ttps
import robotsim.ur3dual.ur3dual as ur3dualsim
import robotsim.nextage.nxt as nxtsim

if __name__ == '__main__':

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "housing.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "tool_motordriver.stl")
    objpath = os.path.join(this_dir, "objects", "bunnysim.stl")

    # from manipulation.grip.hrp5three import hrp5threenm
    # handpkg = hrp5threenm
    handpkg = rtq85nm

    ## use wrs database for ur3d
    gdb = db.GraspDB(database="wrs")
    ## use nxt database for nextage!
    # gdb = db.GraspDB(database="nxt")

    tps = ttps.TabletopPlacements(objpath, handpkg)

    ## build grid space
    ## save to db
    ## use the following grids for ur3d
    # grids = []
    # for x in range(400,401,200):
    #     for y in range(-400,401,800):
    #         grids.append([x,y,1030])
    # tps.saveToDB(grids, gdb)
    ## use the following grids for nxt
    grids = []
    for x in range(400,401,200):
        for y in range(-270,271,540):
            grids.append([x,y,900])
    tps.saveToDB(grids, gdb)

    # add ik for a robot
    # robsim = ur3dualsim.Ur3DualRobot()
    # tps.updateDBwithIK(gdb, robsim, armname = "rgt")
    # tps.updateDBwithIK(gdb, robsim, armname = "lft")

    # add ik for a second robot
    robsim2 = nxtsim.NxtRobot()
    tps.updateDBwithIK(gdb, robsim2, armname = "rgt")
    tps.updateDBwithIK(gdb, robsim2, armname = "lft")

    tps.grpshow(base, gdb)
    base.run()