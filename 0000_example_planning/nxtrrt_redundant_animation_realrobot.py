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
from robotsim.nextage import nxt
from robotsim.nextage import nxtmesh
from robotsim.nextage import nxtball
from pandaplotutils import pandactrl
import bldgfsettingnear
import numpy as np
import utiltools.robotmath as rm
import math
import robotcon.rpc.nxtrobot.nxtrobot_client as nxtc

base = pandactrl.World(camp=[2700, 300, 2700], lookatp=[0, 0, 1000])

env = bldgfsettingnear.Env(base)
env.reparentTo(base.render)
obscmlist = env.getstationaryobslist()
for obscm in obscmlist:
    obscm.showcn()

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
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)

armname = 'rgt'
smoother = sm.Smoother()

robot.goinitpose()

objstpos = np.array([504.5617667638,-0.0889005661,1150.5])
objstrot = np.array([[1.0,0.0,0.0],
                    [0.0,1.0,0.0],
                    [0.0,0.0,1.0]]).T
# load obj collision model using env
objcm = env.loadobj("bunnysim.stl")
objcm.setColor(0.0,.3,.5,.3)
objcm.setMat(base.pg.npToMat4(npmat3=objstrot,npvec3=objstpos))
objcm.reparentTo(base.render)

nxtreal = nxtc.NxtRobot(host = "10.0.1.102:18300")
nxtreal.checkEncoders()
nxtreal.goInitial()
startalljnts = nxtreal.getJointAngles()

robot.movealljnts(startalljnts)
startpos, startrot = robot.getee(armname="rgt")
startjnts = [startalljnts[0], np.array(startalljnts[3:9])]
goalpos = np.array([554.5617667638,200,1150.5])
goalrot = np.dot(rm.rodrigues([1,0,0],90),startrot)
goalpos = goalpos - 150.0*goalrot[:,2]
goaljnts = robot.numikr(goalpos, goalrot, armname=armname)
base.pggen.plotAxis(base.render, spos=startpos, pandamat3=base.pg.npToMat3(startrot))
base.pggen.plotAxis(base.render, spos=goalpos, pandamat3=base.pg.npToMat3(goalrot))
cdchecker = cdck.CollisionCheckerBall(robotball)
ctcallback = ctcb.CtCallback(base, robot, cdchecker, ctchecker=None, armname=armname)
ctcallback.setwaist(usewaist=True)
startjntsflat = [startjnts[0]]+startjnts[1].tolist()
goaljntsflat = [goaljnts[0]]+goaljnts[1].tolist()
print(startjntsflat)
print(goaljntsflat)

starttreesamplerate = 25
endtreesamplerate = 30
planner = rrtc.RRTConnect(start=startjntsflat, goal=goaljntsflat, ctcallback=ctcallback,
                              starttreesamplerate=starttreesamplerate,
                              endtreesamplerate=endtreesamplerate, expanddis=5,
                              maxiter=2000, maxtime=100.0)
robot.movearmfkr(startjnts, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
robotnp.reparentTo(base.render)
robot.movearmfkr(goaljnts, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
robotnp.reparentTo(base.render)
robotball.showcn(base, robotball.genfullbcndict(robot))
robot.goinitpose()
[path, sampledpoints] = planner.planning(obscmlist+[objcm])
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
    if base.inputmgr.keyMap['space'] is True:
        pattern = []
        for pose in path:
            anglesrad = []
            angles = [pose[0], startalljnts[1], startalljnts[2]]+pose[1:]+startalljnts[9:]
            for angle in angles:
                anglesrad.append(math.radians(angle))
            pattern.append(anglesrad)
        print(pattern)

        nxtreal.playPattern(pattern, [.3]*len(pattern))
    return task.again

rbtmnp = [None, None]
motioncounter = [0]
taskMgr.doMethodLater(0.05, update, "update",
                      extraArgs=[rbtmnp, motioncounter, robot, path, armname, robotmesh, robotball],
                      appendTask=True)
base.run()