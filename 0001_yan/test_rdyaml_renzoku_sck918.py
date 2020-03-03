import numpy as np
import utils.robotmath as rm
import pandaplotutils.pandageom as pg
import robotsim.nextage.nxt as nxt
from panda3d.core import *
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.schunk918 import sck918
from robotsim.nextage import nxtmesh
from motionplanning.rrt import rrtconnect as rrtc
import motionplanning.ctcallback as ctcb
import motionplanning.collisioncheckerball as cdck
from robotsim.nextage import nxtball
import motionplanning.smoother as sm
from readfiles import readyaml
import time
import math
from direct.task import Task
import bldgfsettingnear
import robotcon.rpc.nxtrobot.nxtrobot_client as nxturx

if __name__ == "__main__":
    base = pandactrl.World(camp=[3500, -300, 1500], lookatp=[0, 0, 1000])
    env = bldgfsettingnear.Env(base)
    env.reparentTo(base.render)

    objpos_lft = []
    objrot_lft = []
    objpos_rgt = []
    objrot_rgt = []

    nxtrobot = nxt.NxtRobot()
    robotball = nxtball.NxtBall()
    lfthand = sck918.Sck918(ftsensoroffset=0)
    rgthand = sck918.Sck918(ftsensoroffset=0)
    robotmesh = nxtmesh.NxtMesh(lfthand=lfthand, rgthand=rgthand)
    nxtmesh = robotmesh.genmnp(nxtrobot, jawwidthrgt=50.0, jawwidthlft=50.0, togglejntscoord=False)
    # nxtmesh.reparentTo(base.render)
    armname = 'rgt'
    nxtball = nxtball.NxtBall()
    cdchecker = cdck.CollisionCheckerBall(nxtball)
    ctcallback = ctcb.CtCallback(base, nxtrobot, cdchecker, ctchecker=None, armname=armname)
    smoother = sm.Smoother()
    starttreesamplerate = 25
    endtreesamplerate = 30
    pandamat4 = Mat4()
    pandamat4.setRow(3, Vec3(0, 0, 250))

    nxtrobot.goinitpose()

    armjntsgoal6 = []
    # # this part can be packed as a function
    filename = "document/action0.yaml"
    num = readyaml.getlength(filename)

    for ts in range(num):
        [main_object, s_tran, s_rot, s_tran_sub, s_rot_sub] = \
            readyaml.readyaml(filename, ts)
        for i in range(3):
            s_tran[i] = s_tran[i] * 1000
            s_tran_sub[i] = s_tran_sub[i] * 1000
        # # cvtmatrix of main objects to world
        mat0 = []
        # # # # when the main object is the base
        if main_object == "HFMQA-SC-A150-B100-T25-X35-Y50-N12-L70-NA12":
            mat0 = np.array([[1, 0, 0, 500],
                             [0, 0, -1, 0],
                             [0, 1, 0, 50 + 973],
                             [0, 0, 0, 1]])
        # # # # when the main object is the shaft 1
        if main_object == "FXAC15-15-F22-MA12_1":
            mat0 = np.array([[0, 0, -1, 450],
                             [0, 1, 0, 34.9],
                             [1, 0, 0, 57.1 + 973],
                             [0, 0, 0, 1]])
        # # # # when the main object is the shaft 2
        if main_object == "FXAC15-15-F22-MA12_2":
            mat0 = np.array([[0, 0, -1, 449.8],
                             [0, 1, 0, 104.6],
                             [1, 0, 0, 56.7 + 973],
                             [0, 0, 0, 1]])
        # # cvtmatrix of sub-object to main object
        mat1 = np.array([[s_rot[0][0], s_rot[0][1], s_rot[0][2], s_tran[0]],
                         [s_rot[1][0], s_rot[1][1], s_rot[1][2], s_tran[1]],
                         [s_rot[2][0], s_rot[2][1], s_rot[2][2], s_tran[2]],
                         [0, 0, 0, 1]])
        # # cvtmatrix of joint5 to sub-object
        mat2 = np.array([[s_rot_sub[0][0], s_rot_sub[0][1], s_rot_sub[0][2], s_tran_sub[0]],
                         [s_rot_sub[1][0], s_rot_sub[1][1], s_rot_sub[1][2], s_tran_sub[1]],
                         [s_rot_sub[2][0], s_rot_sub[2][1], s_rot_sub[2][2], s_tran_sub[2]],
                         [0, 0, 0, 1]])
        # # change coordinate system to the new version
        mat3 = np.array([[0, 0, -1, 0],
                         [0, 1, 0, 0],
                         [1, 0, 0, 0],
                         [0, 0, 0, 1]])
        # # cvtmatrix of joint6 to joint5
        mat4 = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 145 - 45],
                         [0, 0, 0, 1]])

        mat = np.dot(np.dot(np.dot(np.dot(mat0, mat1), mat2), mat3), mat4)
        tran = np.array([mat[0][3], mat[1][3], mat[2][3]])
        objpos_rgt.append(tran)
        print("Pos: %s\n" % objpos_rgt[ts])
        rot = np.array([[mat[0][0], mat[0][1], mat[0][2]],
                        [mat[1][0], mat[1][1], mat[1][2]],
                        [mat[2][0], mat[2][1], mat[2][2]]])
        objrot_rgt.append(rot)
        print("Rot: %s\n" % objrot_rgt[ts])

        armjntsgoal6.append(nxtrobot.numik(objpos_rgt[ts], objrot_rgt[ts], armname))
        print("Joint angles: %s\n" % armjntsgoal6[ts])
        if armjntsgoal6[ts] is not None:
            nxtrobot.movearmfk(armjntsgoal6[ts], armname)
            nxtmesh = robotmesh.genmnp(nxtrobot, jawwidthrgt=50, jawwidthlft=0)
            # nxtmesh = nmg.gensnp(nxtrobot)
            # nxtmesh.reparentTo(base.render)

    # dcam = loader.loadShader('dcam.sha')
    # # render everything through this camera and shader
    # base.render.setShader(dcam)
    # # loadPrcFileData('', 'show-buffers 1')
    base.pggen.plotAxis(base.render)
    nxtrobot.goinitpose()

    # uncomment the following commands to actuate the robot
    # nxtu = nxturx.NxtRobot(host = "10.0.1.102:18300")
    # nxtu.goInitial()

    initjnts = np.array([-15.0, 0.0, -143.0, 0.0, 0.0, 0.0])
    armjntsgoal6.insert(0, initjnts)

    newpath = []
    for i in range(num):
        startjnts = armjntsgoal6[i]
        print("start = %s" % startjnts)
        goaljnts = armjntsgoal6[i + 1]
        print("goal = %s" % goaljnts)
        planner = rrtc.RRTConnect(start=startjnts, goal=goaljnts, ctcallback=ctcallback,
                                  starttreesamplerate=starttreesamplerate,
                                  endtreesamplerate=endtreesamplerate, expanddis=5,
                                  maxiter=2000, maxtime=100.0)
        tic = time.clock()
        [path, sampledpoints] = planner.planning(obstaclelist=[])
        print("path is %s" % path)
        toc = time.clock()
        print(toc - tic)
        newpath.append(smoother.pathsmoothing(path, planner))

    counter = [0]
    rbtmnp = [None, None]
    motioncounter = [0]


    def updateshow(rbtmnp, motioncounter, robot, path, armname, robotmesh, robotball, task):
        if motioncounter[0] < len(path):
            if rbtmnp[0] is not None:
                rbtmnp[0].detachNode()
                # rbtmnp[1].detachNode()
            pose = path[motioncounter[0]]
            robot.movearmfk(pose, armname)
            rbtmnp[0] = robotmesh.genmnp(robot, jawwidthrgt=50, jawwidthlft=50)
            bcndict = robotball.genfullactivebcndict(robot)
            # rbtmnp[1] = robotball.showcn(base, bcndict)
            rbtmnp[0].reparentTo(base.render)
            motioncounter[0] += 1
        else:
            motioncounter[0] = 0
            return task.done
        return task.again


    def updatesection(newpathnumikmsmpactive, motioncounter, robot, robotmesh, robotball, task):
        # if base.inputmgr.keyMap['space']:
        if counter[0] < len(newpath):
            path = newpath[counter[0]]
            base.inputmgr.keyMap['space'] = False
            # taskMgr.remove('updateshow')
            if not taskMgr.hasTaskNamed("updateshow"):
                taskMgr.doMethodLater(0.1, updateshow, "updateshow",
                                      extraArgs=[rbtmnp, motioncounter, robot, path, armname, robotmesh, robotball],
                                      appendTask=True)
                counter[0] += 1
                # check if the arm is moved
                rgtdiff = abs(np.array(path[0]) - np.array(path[-1]))
                if rgtdiff.max() > 1e-6:
                    path_rad = []
                    pathpt_rad = []
                    print("path is %s" % path)
                    for i in range(len(path)):
                        # print "jntslist_rgt[%d] is %s" % (i, jntslist_rgt[i])
                        for j in range(len(path[i])):
                            pathpt_rad.append(math.radians(path[i][j]))
                        path_rad.append([0, 0, 0] + pathpt_rad[len(path[i]) * i:len(path[i]) * (i + 1)] +
                                        [math.radians(15), 0, math.radians(-143), 0, 0, 0])
                    print("path_rad is %s" % path_rad)
                    # time = [0.8]
                    # nxtu.playPattern(path_rad, time*len(path_rad))
                print("You have pressed the space %d time(s)" % counter[0])
                if counter[0] == num:
                    print("WARNING! The motion section is completed!\n"
                          "Please execute goInitial()!")
        else:
            counter[0] = 0
        return task.again

    taskMgr = Task.TaskManager()
    taskMgr.add(updatesection, "updatesection",
                extraArgs=[newpath, motioncounter, nxtrobot, robotmesh, robotball],
                appendTask=True)

    base.run()
