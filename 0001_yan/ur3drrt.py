"""
This file shows how to perform motion planning.
You need to declare three objects to initialize a motion planner:
1. a collisioncheckerball object
2. a ctcallback object
3. a motion planner (could be rrt, ddrrt, rrtconnect(recommended), ddrrtconnect, etc.

You may also declare a smoother object to smooth the path using random cut afterwards.

author: weiwei, toyonaka
date: 20190312
"""

from manipulation.grip.robotiq85 import rtq85
from motionplanning import smoother as sm
from motionplanning import ctcallback as ctcb
from motionplanning import collisioncheckerball as cdck
from motionplanning.rrt import rrtconnect as rrtc
from robotsim.ur3dual import ur3dual
from robotsim.ur3dual import ur3dualmesh
from robotsim.ur3dual import ur3dualball
from pandaplotutils import pandactrl
import bunrisettingfree
import numpy as np
import copy
import utils.robotmath as rm

base = pandactrl.World(camp=[2700, 300, 2700], lookatp=[0, 0, 1000])

objname = "tool_motordriver.stl"
env = bunrisettingfree.Env(base)
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

robot = ur3dual.Ur3DualRobot()
robotball = ur3dualball.Ur3DualBall()
rgthnd = rtq85.Rtq85(jawwidth=85)
lfthnd = rtq85.Rtq85(jawwidth=85)
robotmesh = ur3dualmesh.Ur3DualMesh(rgthand=rgthnd, lfthand=lfthnd)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
# robotnp.reparentTo(base.render)
armname = 'rgt'
cdchecker = cdck.CollisionCheckerBall(robotball)
ctcallback = ctcb.CtCallback(base, robot, cdchecker, armname=armname)
smoother = sm.Smoother()

robot.goinitpose()
# robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
# robotnp.reparentTo(base.render)
# base.run()

starttreesamplerate = 25
endtreesamplerate = 30
objstpos = np.array([354.5617667638,-400.0889005661,1090.5])
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

startpos = np.array([354.5617667638,-256.0889005661,1060.5])
startrot = np.array([[1,0,0],
                     [0,-1,0],
                     [0,0,-1]]).T
goalpos3 = np.array([354.5617667638,-256.0889005661,1150.5])
goalrot3 = np.array([[1,0,0],
                    [0,-0.92388,-0.382683],
                    [0,0.382683,-0.92388]]).T
goalpos3 = goalpos3 - 70.0*goalrot3[:,2]
goalpos4 = np.array([454.5617667638,-100,1150.5])
goalrot4 = np.dot(rm.rodrigues([0,0,1],-120),goalrot3)
goalpos4 = goalpos4 - 250.0*goalrot4[:,2]
# goalpos2 = goalpos2 - 150.0*goalrot2[:,2]
start = robot.numik(startpos, startrot, armname)
goal3 = robot.numikmsc(goalpos3, goalrot3, msc=start, armname=armname)
print(goal3)
goal4 = robot.numikmsc(goalpos4, goalrot4, msc=start, armname=armname)
print(goal4)
planner = rrtc.RRTConnect(start=goal3, goal=goal4, ctcallback=ctcallback,
                              starttreesamplerate=starttreesamplerate,
                              endtreesamplerate=endtreesamplerate, expanddis=5,
                              maxiter=2000, maxtime=100.0)
robot.movearmfk(goal3, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
robotnp.reparentTo(base.render)
robot.movearmfk(goal4, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
robotnp.reparentTo(base.render)
# base.run()
robotball.showcn(base, robotball.genfullbcndict(robot))
robot.goinitpose()
[path, sampledpoints] = planner.planning(obscmlist)
path = smoother.pathsmoothing(path, planner)
print(path)
for pose in path:
    robot.movearmfk(pose, armname)
    robotstick = robotmesh.gensnp(robot = robot)
    robotstick.reparentTo(base.render)
# base.pggen.plotAxis(base.render, spos=goalpos, pandamat3 = base.pg.npToMat3(goalrot))

base.run()