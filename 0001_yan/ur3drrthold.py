"""
This file shows how to perform motion planning with object held in the hand.
Like ur3drrt, you will also declare three objects to initialize a motion planner:
1. a collisioncheckerball object
2. a ctcallback object
3. a motion planner (could be rrt, ddrrt, rrtconnect(recommended), ddrrtconnect, etc.

You may also additionally declare a smoother object to smooth the path using random cut afterwards.

author: weiwei, toyonaka
date: 20190312
"""

from environment import collisionmodel as cm
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
import os

base = pandactrl.World(camp=[2700, 300, 2700], lookatp=[0, 0, 1000])

env = bunrisettingfree.Env(base)
env.reparentTo(base.render)
obscmlist = env.getstationaryobslist()
for obscm in obscmlist:
    obscm.showcn()

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
objstpos = np.array([354.5617667638,-256.0889005661,1150.5])
objstrot = np.array([[1.0,0.0,0.0],
                    [0.0,1.0,0.0],
                    [0.0,0.0,1.0]]).T
objcmcopys = copy.deepcopy(objcm)
objcmcopys.setColor(0.0,0.0,1.0,.3)
objcmcopys.setMat(base.pg.npToMat4(npmat3=objstrot,npvec3=objstpos))
objcmcopys.reparentTo(base.render)
# objcmcopys.showboxcn()

startpos = np.array([354.5617667638,-256.0889005661,1150.5])
startrot = np.array([[1,0,0],
                    [0,-0.92388,-0.382683],
                    [0,0.382683,-0.92388]]).T
startpos = startpos - 70.0*startrot[:,2]
goalpos = np.array([354.5617667638,100,1250.5])
goalrot = np.array([[1,0,0],
                    [0,-0.92388,-0.382683],
                    [0,0.382683,-0.92388]]).T
goalpos = goalpos - 250.0*goalrot[:,2]

start = robot.numik(startpos, startrot, armname=armname)
print(start)
goal = robot.numikmsc(goalpos, goalrot, msc=start, armname=armname)
print(goal)
planner = rrtc.RRTConnect(start=start, goal=goal, ctcallback=ctcallback,
                              starttreesamplerate=starttreesamplerate,
                              endtreesamplerate=endtreesamplerate, expanddis=5,
                              maxiter=2000, maxtime=100.0)
robot.movearmfk(start, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
robotnp.reparentTo(base.render)
objrelstpos, objrelstrot = robot.getinhandpose(objstpos, objstrot, armname)

robot.movearmfk(goal, armname)
robotnp = robotmesh.genmnp(robot, jawwidthrgt = 85.0, jawwidthlft=0.0)
robotnp.reparentTo(base.render)
robot.goinitpose()

[path, sampledpoints] = planner.planninghold(objcm, [objrelstpos, objrelstrot], obscmlist)
path = smoother.pathsmoothing(path, planner)
print(path)
import random
for pose in path:
    robot.movearmfk(pose, armname)
    robotstick = robotmesh.gensnp(robot = robot)
    robotstick.reparentTo(base.render)
    objworldpos, objworldrot = robot.getworldpose(objrelstpos, objrelstrot, armname)
    temp = copy.deepcopy(objcm)
    temp.setColor(0.0,0.0,random.random(),.3)
    temp.setMat(base.pg.npToMat4(npmat3=objworldrot,npvec3=objworldpos))
    temp.reparentTo(base.render)
    temp.showcn()
# base.pggen.plotAxis(base.render, spos=goalpos, pandamat3 = base.pg.npToMat3(goalrot))

base.run()