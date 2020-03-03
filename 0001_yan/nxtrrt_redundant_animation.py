"""
This file shows how to perform motion planning.
You need to declare three objects to initialize a motion planner:
1. a collisioncheckerball object
2. a ctcallback object
3. a motion planner (could be rrt, ddrrt, rrtconnect(recommended), ddrrtconnect, etc.

You may also declare a smoother object to smooth the path using random cut afterwards.

author: weiwei, toyonaka
date: 20190402
"""

from manipulation.grip.robotiq85 import rtq85
from motionplanning import smoother as sm
from motionplanning import ctcallback as ctcb
from motionplanning import collisioncheckerball as cdck
from motionplanning.rrt import rrtconnect as rrtc
from robotsim.ur3dual import ur3dual
from robotsim.ur3dual import ur3dualmesh
from robotsim.ur3dual import ur3dualball
from robotsim.nextage import nxt
from robotsim.nextage import nxtmesh
from robotsim.nextage import nxtball
from pandaplotutils import pandactrl
import bldgfsettingnear
import numpy as np
import copy
import utils.robotmath as rm
import os
import environment.collisionmodel as cm

base = pandactrl.World(camp=[2700, 300, 2700], lookatp=[0, 0, 1000])

env = bldgfsettingnear.Env(base)
env.reparentTo(base.render)
obscmlist = env.getstationaryobslist()
# for obscm in obscmlist:
#     obscm.showcn()

# load obj collision model using env
objcm = env.loadobj("bunnysim.stl")

# another example -- load obj collision model independently
# this_dir, this_filename = os.path.split(__file__)
# objname = "tool_motordriver.stl"
# objpath = os.path.join(this_dir, "objects", objname)
# objcm = cm.CollisionModel(objpath)

robot = nxt.NxtRobot()
robotball = nxtball.NxtBall()
rgthnd = rtq85.Rtq85(jawwidth=85, ftsensoroffset=0)
lfthnd = rtq85.Rtq85(jawwidth=85, ftsensoroffset=0)
robotmesh = nxtmesh.NxtMesh(rgthand=rgthnd, lfthand=lfthnd)
robotnp = robotmesh.genmnp(robot, jawwidthrgt=85.0, jawwidthlft=0.0)
# robotnp.reparentTo(base.render)
armname = 'rgt'
smoother = sm.Smoother()

robot.goinitpose()
# robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
# robotnp.reparentTo(base.render)
# base.run()

starttreesamplerate = 25
endtreesamplerate = 30
objstpos = np.array([354.5617667638,-256.0889005661,1090.5])
objstrot = np.array([[1.0,0.0,0.0],
                    [0.0,1.0,0.0],
                    [0.0,0.0,1.0]]).T
objgpos = np.array([304.5617667638,-277.1026725769,1101.0])
objgrot = np.array([[1.0,0.0,0.0],
                   [0.0,6.12323426293e-17,-1.0],
                   [0.0,1.0,6.12323426293e-17]]).T
objcmcopys = copy.deepcopy(objcm)
objcmcopys.setColor(0.0,0.0,1.0,.3)
objcmcopys.setMat(base.pg.npToMat4(npmat3=objstrot,npvec3=objstpos))
objcmcopys.reparentTo(base.render)

startpos = np.array([354.5617667638,-56.0889005661,1150.5])
startrot = np.array([[1,0,0],
                    [0,-0.92388,-0.382683],
                    [0,0.382683,-0.92388]]).T
startrot = np.dot(rm.rodrigues([1,0,0],20),startrot)
startpos = startpos - 70.0*startrot[:,2]
goalpos = np.array([354.5617667638,-500,1050.5])
goalrot = np.dot(rm.rodrigues([1,0,0],-90),startrot)
goalpos = goalpos - 150.0*goalrot[:,2]
# goalpos2 = goalpos2 - 150.0*goalrot2[:,2]
startjnts = robot.numikr(startpos, startrot, armname=armname)
print(startjnts)
goaljnts = robot.numikr(goalpos, goalrot, armname=armname)
print(goaljnts)
base.pggen.plotAxis(base.render, spos=startpos, pandamat3=base.pg.npToMat3(startrot))
base.pggen.plotAxis(base.render, spos=goalpos, pandamat3=base.pg.npToMat3(goalrot))
cdchecker = cdck.CollisionCheckerBall(robotball)
ctcallback = ctcb.CtCallback(base, robot, cdchecker, ctchecker=None, armname=armname)
ctcallback.setwaist(usewaist=True)
startjntsflat = [startjnts[0]]+startjnts[1].tolist()
goaljntsflat = [goaljnts[0]]+goaljnts[1].tolist()
print(startjntsflat)
print(goaljntsflat)
planner = rrtc.RRTConnect(start=startjntsflat, goal=goaljntsflat, ctcallback=ctcallback,
                              starttreesamplerate=starttreesamplerate,
                              endtreesamplerate=endtreesamplerate, expanddis=5,
                              maxiter=2000, maxtime=100.0)
robot.movearmfkr(startjnts, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
# robotnp.reparentTo(base.render)
robot.movearmfkr(goaljnts, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
# robotnp.reparentTo(base.render)
robotball.showcn(base, robotball.genfullbcndict(robot))
robot.goinitpose()
[path, sampledpoints] = planner.planning(obscmlist)
path = smoother.pathsmoothing(path, planner)

def update(rbtmnp, motioncounter, robot, path, armname, robotmesh, robotball, task):
    if motioncounter[0] < len(path):
        if rbtmnp[0] is not None:
            rbtmnp[0].detachNode()
            rbtmnp[1].detachNode()
        pose = path[motioncounter[0]]
        posenonflat = [pose[0], np.array(pose[1:])]
        robot.movearmfkr(posenonflat, armname)
        rbtmnp[0] = robotmesh.genmnp(robot, 0, 0)
        bcndict = robotball.genfullactivebcndict(robot)
        rbtmnp[1] = robotball.showcn(base, bcndict)
        rbtmnp[0].reparentTo(base.render)
        motioncounter[0] += 1
    else:
        motioncounter[0] = 0
    return task.again

rbtmnp = [None, None]
motioncounter = [0]
taskMgr.doMethodLater(0.05, update, "update",
                      extraArgs=[rbtmnp, motioncounter, robot, path, armname, robotmesh, robotball],
                      appendTask=True)
base.run()

# for pose in path:
#     print(pose)
#     posenonflat = [pose[0], np.array(pose[1:])]
#     robot.movearmfkr(posenonflat, armname)
#     robotstick = robotmesh.gensnp(robot = robot)
#     robotstick.reparentTo(base.render)
# # base.pggen.plotAxis(base.render, spos=goalpos, pandamat3 = base.pg.npToMat3(goalrot))
#
# base.run()