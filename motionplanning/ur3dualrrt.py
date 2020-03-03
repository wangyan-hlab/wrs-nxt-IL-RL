import motionplanning.collisioncheckerball as cdck
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.robotiq85 import rtq85
from motionplanning.rrt import ddrrtconnect as ddrrtc
from motionplanning.rrt import rrtconnect as rrtc
from motionplanning.rrt import ddrrt as ddrrt
from robotsim.ur3dual import ur3dual
from robotsim.ur3dual import ur3dualmesh
from robotsim.ur3dual import ur3dualball

# the following one is an old callback function
# use the previous class instead
def iscollidedfunc(point, obstaclelist = [], robot = None, cdchecker = None):
    """
    check if a specific configuration is in collision

    :param point:
    :param robot the object defined in robotsim/robot
    :param cdchecker: a collisionchecker object
    :return:

    author: weiwei
    date: 20180109
    """

    robot.movearmfk(point)
    isselfcollided = cdchecker.isCollided(robot, obstaclelist)
    robot.goinitpose()

    if isselfcollided:
        return True
    else:
        return False

if __name__ == '__main__':
    # import robotcon.ur3dual as ur3urx
    import numpy as np
    import math3d
    import motionplanning.ctcallback as cdcb

    base = pandactrl.World(camp=[4000,0,4000])

    robot = ur3dual.Ur3DualRobot()
    robot.goinitpose()
    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()
    robotmesh = ur3dualmesh.Ur3DualMesh(rgthand=rgthnd, lfthand=lfthnd)
    # robotmesh.genmnp(robot).reparentTo(base.render)
    # base.run()
    robotball = ur3dualball.Ur3DualBall()
    # cdchecker = cdck.CollisionChecker(robotmesh)
    cdchecker = cdck.CollisionCheckerBall(robotball)
    cdcallback = cdcb.CDCallback(base, robot, cdchecker)

    # ur3u = ur3urx.Ur3DualUrx()
    # start = ur3u.getjnts('rgt')
    # goal = robot.initjnts[3:9]
    # start = ur3u.getjnts('rgt')
    goalpos = np.array([-300,-400,1800])
    goalrot = np.array([[0,1,0],[-1,0,0],[0,0,1]])
    # goalpos = np.array([300,0,1510])
    # goalrot = np.array([[-1,0,0],[0,0,1],[0,1,0]])
    # goalpos = np.array([300,-50,1070])
    # goalrot = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    goal = robot.numik(goalpos, goalrot, armid = 'rgt')
    print(goal)
    # start = [50.0,0.0,-143.0,0.0,0.0,0.0]
    # goal = [-15.0,0.0,-143.0,0.0,0.0,0.0]
    # plot init and goal
    # goal = robot.initjnts[3:9]

    # for i in range(1,100):
    #     print ur3u.recvft(armid = 'rgt')[0]

    armid = 'rgt'
    start = robot.initjnts[3:9]
    robot.movearmfk(armjnts = start, armid = armid)
    robotmesh.genmnp(robot).reparentTo(base.render)
    bcndict = robotball.genfullactivebcndict(robot)
    robotball.showbcn(base, bcndict)
    robot.movearmfk(armjnts = goal, armid = armid)
    robotmesh.genmnp(robot).reparentTo(base.render)
    bcndict = robotball.genfullactivebcndict(robot)
    robotball.showbcn(base, bcndict)
    # base.run()

    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, cdcallback=cdcallback,
                                  starttreesamplerate=30, endtreesamplerate=30,
                                  expanddis=40, maxiter=5000, maxtime = 150.0)

    # planner = ddrrt.DDRRT(start=start, goal=goal, cdcallback=cdcallback,
    #                               goalsamplerate=20,
    #                               expanddis=40, maxiter=5000, maxtime = 50.0)

    # planner = rrtc.RRTConnect(start=start, goal=goal, cdcallback=cdcallback,
    #                               starttreesamplerate=30, endtreesamplerate=30,
    #                               expanddis=20, maxiter=5000, maxtime = 150.0)

    import time
    tic = time.time()
    [path, sampledpoints] = planner.planning(obstaclelist = [])
    toc = time.time()
    print(toc-tic)
    import smoother as sm
    smoother = sm.Smoother()
    path = smoother.pathsmoothing(path, planner, 100)

    # test new movejntssgl
    # ur3u.movejntssgl_cont(path, armid='rgt')

    # # jposelist to tposelist
    # tposelist = []
    # for pose in path:
    #     robot.movearmfk(pose, armid=armid)
    #     tcppos, tcprot = robot.gettcp_robot()
    #     tcppos = tcppos/1000.0
    #     tpose = math3d.Transform(tcprot, tcppos).get_pose_vector()
    #     tposelist.append(tpose)
    # ur3u.movetposesgl_cont(tposelist, armid = armid, acc=1, vel=.5, radius=0.1)

    #
    # for pose in path:
    #     ur3u.movejntssgl_cont(pose, armid = 'rgt')
    #     robot.movearmfk(pose, armid = 'rgt')
    #     robotstick = robotmesh.gensnp(robot = robot)
    #     robotstick.reparentTo(base.render)
    # ur3u.movejntssgl(path[-1], armid='rgt')

    sampledpointsindex = [0]
    robotonscreen = [None]
    pathindex = [0]

    def updateshow(path, pathindex, sampledpoints, sampledpointsindex,
                   robot, robotmesh, task):
        if sampledpointsindex[0] <= len(sampledpoints) - 1:
            if robotonscreen[0] is not None:
                robotonscreen[0].removeNode()
            robot.movearmfk(sampledpoints[sampledpointsindex[0]][0], armid=armid)
            robotonscreen[0] = robotmesh.genmnp(robot)
            robotonscreen[0].reparentTo(base.render)
            robot.goinitpose()
            sampledpointsindex[0] += 1
            return task.again
        else:
            if robotonscreen[0] is not None:
                robotonscreen[0].removeNode()
            if pathindex[0] <= len(path) - 1:
                robot.movearmfk(path[pathindex[0]], armid=armid)
                robotonscreen[0] = robotmesh.genmnp(robot)
                robotonscreen[0].reparentTo(base.render)
                pathindex[0] += 1
            else:
                pathindex[0] = 0
            time.sleep(.07)
            return task.again

    taskMgr.add(updateshow, "updateshow",
                extraArgs=[path, pathindex, sampledpoints, sampledpointsindex,
                           robot, robotmesh],
                appendTask=True)

    # ur3u.closegripper(armid = 'lft')
    # ur3u.closegripper(armid = 'rgt')

    base.run()