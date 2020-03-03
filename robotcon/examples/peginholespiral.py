import robotcon.ur3dual as ur3urx
from robotsim.ur3dual import ur3dual
from robotsim.ur3dual import ur3dualmesh
from robotsim.ur3dual import ur3dualball
import motionplanning.collisioncheckerball as cdck
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from manipulation.grip.robotiq85 import rtq85
from motionplanning.rrt import ddrrtconnect as ddrrtc
import numpy as np
import robotcon.robotiq.programbuilder as pb

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
    import motionplanning.smoother as sm

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,1200])

    robot = ur3dual.Ur3DualRobot()
    robot.goinitpose()
    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()
    robotmesh = ur3dualmesh.Ur3DualMesh(rgthand=rgthnd, lfthand=lfthnd)
    robotball = ur3dualball.Ur3DualBall()
    ur3u = ur3urx.Ur3DualUrx()
    pgg = pandageom.PandaGeomGen()
    cdchecker = cdck.CollisionCheckerBall(robotball)

    start = ur3u.getjnts('rgt')
    goalpos = np.array([200,-150,1410])
    # goalrot = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    goalrot = np.array([[0,-1,0],[0,0,1],[-1,0,0]])
    goalrgt = robot.numik(goalpos, goalrot, armid = 'rgt')
    # plot init and goal
    # robot.movearmfk(armjnts = start, armid = 'rgt')
    # robotmesh.genmnp(robot).reparentTo(base.render)
    # robot.movearmfk(armjnts = goal, armid = 'rgt')
    # robotmesh.genmnp(robot).reparentTo(base.render)

    # smther = sm.Smoother()
    # jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
    #                [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
    #                [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
    #                [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
    #                [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
    #                [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']]]
    #
    # planner = ddrrtc.DDRRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #                               jointlimits = jointlimits, starttreesamplerate=30, expanddis = 5, robot = robot,
    #                               cdchecker = cdchecker)
    # [pathrgt, sampledpoints] = planner.planning(obstaclelist = [])
    # pathrgt = smther.pathsmoothing(pathrgt, planner)
    #
    # start = ur3u.getjnts('lft')
    goalpos = np.array([200,160,1410])
    goalrot = np.array([[1,0,0],[0,0,-1],[0,1,0]])
    goallft = robot.numik(goalpos, goalrot, armid = 'lft')
    # jointlimits = [[robot.lftarm[1]['rngmin'], robot.lftarm[1]['rngmax']],
    #                [robot.lftarm[2]['rngmin'], robot.lftarm[2]['rngmax']],
    #                [robot.lftarm[3]['rngmin'], robot.lftarm[3]['rngmax']],
    #                [robot.lftarm[4]['rngmin'], robot.lftarm[4]['rngmax']],
    #                [robot.lftarm[5]['rngmin'], robot.lftarm[5]['rngmax']],
    #                [robot.lftarm[6]['rngmin'], robot.lftarm[6]['rngmax']]]
    # planner = ddrrtc.DDRRTConnect(start=start, goal=goal, iscollidedfunc=iscollidedfunc,
    #                               jointlimits=jointlimits, starttreesamplerate=30, expanddis=5, robot=robot,
    #                               cdchecker=cdchecker)
    # [pathlft, sampledpoints] = planner.planning(obstaclelist=[])
    # pathlft = smther.pathsmoothing(pathlft, planner)

    # ur3u.movejntssgl_cont(pathrgt, armid='rgt')
    # ur3u.movejntssgl_cont(pathlft, armid = 'lft')

    ur3u.movejntssgl(goalrgt, armid='rgt')
    ur3u.movejntssgl(goallft, armid = 'lft')

    # get initial ft

    # fxr0, fyr0, fzr0, txr0, tyr0, tzr0 = ur3u.recvft(armid='rgt')
    # fxl0, fyl0, fzl0, txl0, tyl0, tzl0 = ur3u.recvft(armid='lft')

    # atp = ur3u.rgtarm.getactualtcppose()
    # atpinscript = "p[" + str(atp[0])[:5] + "," + str(atp[1])[:5] + "," + str(atp[2])[:5] + "," + \
    #               str(atp[3])[:5] + "," + str(atp[4])[:5] + "," + str(atp[5])[:5] + "]"
    # print atpinscript

    # spiral
    # pblder = pb.ProgramBuilder()
    # pblder.loadprog("peginholespiral_param.script")
    # prog = pblder.ret_program_to_run()
    # prog = prog.replace("parameter_startz", str(atp[2])[:5])
    # prog = prog.replace("parameter_stroke", "0.075")
    # id = prog.find("starting_pose=")
    # print prog[id-5:id+200]

    # impedance
    # pblder = pb.ProgramBuilder()
    # pblder.loadprog("peginholeimp.script")
    # prog = pblder.ret_program_to_run()
    # prog = prog.replace("parameter_massm", "10")
    # id = prog.find("massm =")
    # print prog[id-5:]

    # spiral impedance
    pblder = pb.ProgramBuilder()
    pblder.loadprog("peginholespiral_imp.script")
    prog = pblder.ret_program_to_run()
    prog = prog.replace("parameter_massm", "10")
    prog = prog.replace("parameter_stroke", "0.035")

    # pblder.loadprog("peginholespiral.script")
    # pblder.loadprog("tst.script")
    ur3u.rgtarm.send_program(prog)
    # wait until program runs
    while not ur3u.rgtarm.is_program_running():
        pass

    # ur3u.lftarm.send_program(prog)
    # # wait until program runs
    # while not ur3u.lftarm.is_program_running():
    #     pass

    # show data
    robotnp = []
    arrownp = []
    othernp = []
    def updateshow(ur3u, robot, robotmesh, robotnp, arrownp, othernp, task):
        for robotinnp in robotnp:
            robotinnp.removeNode()
        for arrow in arrownp:
            arrow.removeNode()
        for other in othernp:
            other.removeNode()

        rgtjoints = ur3u.getjnts(armid='rgt')
        robot.movearmfk(rgtjoints, armid='rgt')
        lftjoints = ur3u.getjnts(armid='lft')
        robot.movearmfk(lftjoints, armid='lft')
        robotmnp = robotmesh.genmnp(robot, toggleendcoord=False)
        robotmnp.reparentTo(base.render)
        robotnp.append(robotmnp)

        # plot rgtarm forces
        armid = 'rgt'
        tcppos, tcprot = robot.gettcp_robot(armid)
        pos = tcppos
        xvec = tcprot[:,0]
        yvec = tcprot[:,1]
        zvec = tcprot[:,2]
        fx, fy, fz, tx, ty, tz = ur3u.recvft(armid)
        # scale:
        fx = fx*10.
        fy = fy*10.
        fz = fz*10.
        # portion:wob
        tqscale = 10.0
        txportion = tx/tqscale if abs(tx/tqscale)<=1.0 else 1.0*np.sign(tx)
        typortion = ty/tqscale if abs(ty/tqscale)<=1.0 else 1.0*np.sign(ty)
        tzportion = tz/tqscale if abs(tz/tqscale)<=1.0 else 1.0*np.sign(tz)
        # fx = fx-fxr0
        # fy = fy-fyr0
        # fz = fz-fzr0
        othernp.append(pgg.plotText(content = "Right arm: "+str(fx/10.)+", "+str(fy/10.)+", "+str(fz/10.)+", "+
                               str(tx)+", "+str(ty)+", "+str(tz), rgba=[0,0,0,1], pos=[-.5,0,-.5]))
        arrownp.append(pgg.plotArrow(base.render, pos, pos+xvec*fx, thickness = 10, rgba = [.7,.2,.2,1]))
        arrownp.append(pgg.plotArrow(base.render, pos, pos+yvec*fy, thickness = 10, rgba = [.2,.7,.2,1]))
        arrownp.append(pgg.plotArrow(base.render, pos, pos+zvec*fz, thickness = 10, rgba = [.2,.2,.7,1]))
        arrownp.append(pgg.plotPortionCircArrow(base.render, xvec, txportion, pos, 100, 10, rgba = [.3,.0,.0,.7]))
        arrownp.append(pgg.plotPortionCircArrow(base.render, yvec, typortion, pos, 100, 10, rgba = [.0,.3,.0,.7]))
        arrownp.append(pgg.plotPortionCircArrow(base.render, zvec, tzportion, pos, 100, 10, rgba = [.0,.0,.3,.7]))
        # arrownp.append(pandageom.plotArrow(base.render, pos, pos+xvec*300, thickness = 5, rgba = [.7,.2,.2,1]))
        # arrownp.append(pandageom.plotArrow(base.render, pos, pos+yvec*300, thickness = 5, rgba = [.2,.7,.2,1]))
        # arrownp.append(pandageom.plotArrow(base.render, pos, pos+zvec*300, thickness = 5, rgba = [.2,.2,.7,1]))

        armid = 'lft'
        tcppos, tcprot = robot.gettcp_robot(armid)
        pos = tcppos
        xvec = tcprot[:,0]
        yvec = tcprot[:,1]
        zvec = tcprot[:,2]
        fx, fy, fz, tx, ty, tz = ur3u.recvft(armid)
        # scale:
        fx = fx*10.
        fy = fy*10.
        fz = fz*10.
        # portion:
        tqscale = 10.0
        txportion = tx/tqscale if abs(tx/tqscale)<=1.0 else 1.0*np.sign(tx)
        typortion = ty/tqscale if abs(ty/tqscale)<=1.0 else 1.0*np.sign(ty)
        tzportion = tz/tqscale if abs(tz/tqscale)<=1.0 else 1.0*np.sign(tz)
        # fx = fx-fxl0
        # fy = fy-fyl0
        # fz = fz-fzl0
        othernp.append(pgg.plotText(content = "Left arm: "+str(fx/10.)+", "+str(fy/10.)+", "+str(fz/10.)+", "+
                               str(tx)+", "+str(ty)+", "+str(tz), rgba=[0,0,0,1], pos=[-.5,0,-.55]))
        arrownp.append(pgg.plotArrow(base.render, pos, pos+xvec*fx, thickness = 10, rgba = [.7,.2,.2,1]))
        arrownp.append(pgg.plotArrow(base.render, pos, pos+yvec*fy, thickness = 10, rgba = [.2,.7,.2,1]))
        arrownp.append(pgg.plotArrow(base.render, pos, pos+zvec*fz, thickness = 10, rgba = [.2,.2,.7,1]))
        arrownp.append(pgg.plotPortionCircArrow(base.render, xvec, txportion, pos, 100, 10, rgba = [.3,.0,.0,.7]))
        arrownp.append(pgg.plotPortionCircArrow(base.render, yvec, typortion, pos, 100, 10, rgba = [.0,.3,.0,.7]))
        arrownp.append(pgg.plotPortionCircArrow(base.render, zvec, tzportion, pos, 100, 10, rgba = [.0,.0,.3,.7]))
        # arrownp.append(pandageom.plotArrow(base.render, pos, pos+xvec*300, thickness = 5, rgba = [.7,.2,.2,1]))
        # arrownp.append(pandageom.plotArrow(base.render, pos, pos+yvec*300, thickness = 5, rgba = [.2,.7,.2,1]))
        # arrownp.append(pandageom.plotArrow(base.render, pos, pos+zvec*300, thickness = 5, rgba = [.2,.2,.7,1]))

        if not ur3u.rgtarm.is_program_running():
            # print "open griper"
            # # task done
            # ur3u.opengripper('rgt')
            # goalpos = np.array([200,-120,1410])
            # goalrot = np.array([[1,0,0],[0,0,1],[0,-1,0]])
            # goal = robot.numik(goalpos, goalrot, armid = 'rgt')
            # ur3u.movejntssgl(goal, 'rgt')
            return task.done
        return task.again
    taskMgr.add(updateshow, 'updateshow', extraArgs = [ur3u, robot, robotmesh, robotnp, arrownp, othernp], appendTask = True)

    # ur3u.rgtarm.send_program("rq_linear_search('Z+',10,0.01,0.05)")
    # ur3u.rgtarm.send_program("rq_spiral_search(3,5,.3,False)")

    base.run()