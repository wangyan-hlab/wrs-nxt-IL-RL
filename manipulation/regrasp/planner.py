import sys
import copy
import numpy as np
import motionplanning.smoother as sm
import motionplanning.ctcallback as ctcb
import motionplanning.rrt.ddrrtconnect as ddrrtc
import manipulation.regrasp.sqdescriber2 as sqdes

def planRegrasp(robot, regrip, objcm, cdchecker, obstaclecmlist, id = 0, switch="OC", previous=[], end=False, togglemp=True):
    """
    plan the regrasp sequences

    :param objpath:
    :param robot:
    :param hand:
    :param dbase:
    :param obstaclecmlist:
    :param id = 0
    :param switch in "OC" open-close "CC" close-close
    :param previous: set it to [] if the motion is not a continuing one, or else, set it to [lastobjmat4, lastikr, lastjawwidth]
    :param end: set it to True if it is the last one
    :param togglemp denotes whether the motion between the keyposes are planned or not, True by default
    :return:

    author: weiwei
    date: 20180924
    """

    while True:
        print("new search")
        regrip.updateshortestpath()
        [objms, numikrms, jawwidth, pathnidlist, originalpathnidlist] = \
            sqdes.getMotionSequence(regrip, id = id, choice = regrip.choice, type=switch, previous=previous)
        if objms is None:
            return [None, None, None, None]
        bcdfree = True
        for i in range(len(numikrms)):
            rgtarmjnts = numikrms[i][1].tolist()
            lftarmjnts = numikrms[i][2].tolist()
            robot.movealljnts([numikrms[i][0], 0, 0]+rgtarmjnts+lftarmjnts)
            # skip the exact handover pose and only detect the cd between armhnd and body
            if pathnidlist[i].startswith('ho') and pathnidlist[i+1].startswith('ho'):
                abcd = cdchecker.isCollidedHO(robot, obstaclecmlist)
                if abcd:
                    regrip.removeBadNodes([pathnidlist[i][:-1]])
                    print("Abcd collided at ho pose")
                    bcdfree = False
                    # robotmesh.genmnp(robot).reparentTo(base.render)
                    # robotball.showbcn(base, robotball.genbcndict(robot))
                    # base.run()
                    break
            else:
                # NOTE: we ignore both arms here for conciseness
                # This might be a potential bug
                if cdchecker.isCollided(robot, obstaclecmlist, holdarmname="all"):
                    regrip.removeBadNodes([pathnidlist[i][:-1]])
                    print("Robot collided at non-ho pose")
                    bcdfree = False
                    break
        robot.goinitpose()
        if bcdfree:
            objmsmp = []
            numikrmsmp = []
            jawwidthmp = []
            print(pathnidlist)
            if not togglemp:
                for i, numikrm in enumerate(numikrms):
                    if i>0:
                        startid = pathnidlist[i-1]
                        endid = pathnidlist[i]
                        if (not end) and (endid is 'end'):
                            continue
                        if (len(previous) > 0) and (startid is 'begin'):
                            continue
                        numikrmsmp.append([numikrms[i-1], numikrms[i]])
                        objmsmp.append([objms[i-1], objms[i]])
                        jawwidthmp.append([jawwidth[i-1], jawwidth[i]])
                return objmsmp, numikrmsmp, jawwidthmp, originalpathnidlist

            # INNERLOOP motion planning
            smoother = sm.Smoother()
            ctcallback = ctcb.CtCallback(base, robot, cdchecker)
            breakflag = False
            for i, numikrm in enumerate(numikrms):
                if i > 0:
                    # determine which arm to plan
                    # assume right
                    # assume redundant planning
                    robot.goinitpose()
                    startid = pathnidlist[i-1]
                    endid = pathnidlist[i]
                    objmat = base.pg.mat4ToNp(objms[i-1])
                    objrot = objmat[:3,:3]
                    objpos = objmat[:3,3]
                    if (not end) and (endid is 'end'):
                        continue
                    if (len(previous)>0) and (startid is 'begin'):
                        continue
                    if (startid[-1] == "o" and endid[-1] == "c") or (startid[-1] == "c" and endid[-1] == "o"):
                        # open and close gripper
                        print("O/C hands, simply include ", pathnidlist[i - 1], " and ", pathnidlist[i])
                        numikrmsmp.append([numikrms[i-1], numikrms[i]])
                        objmsmp.append([objms[i-1], objms[i]])
                        jawwidthmp.append([jawwidth[i-1], jawwidth[i]])
                        continue
                    if (startid[:-1] == endid[:-1]):
                        if (startid[-1] != "i") and (endid[-1] != "i"):
                            # linear interpolation
                            tempnumikrmsmp = []
                            tempjawwidthmp = []
                            tempobjmsmp = []
                            temparmname = "rgt"
                            startjntags = numikrms[i-1][1].tolist()
                            goaljntags = numikrms[i][1].tolist()
                            if "lft" in startid:
                                temparmname = "lft"
                                startjntags = numikrms[i-1][2].tolist()
                                goaljntags = numikrms[i][2].tolist()

                            [interplatedjnts, interplatedobjposes] = \
                                ctcallback.isLMAvailableJNTwithObj(startjntags, goaljntags,
                                                                   [objpos, objrot],  armname = temparmname, type = startid[-1])
                            if len(interplatedjnts) == 0:
                                print("Failed to interpolate motion primitive! restarting...")
                                # always a single hand
                                regrip.removeBadNodes([pathnidlist[i-1][:-1]])
                                breakflag = True
                                break
                            print("Motion primitives, interplate ", pathnidlist[i - 1], " and ", pathnidlist[i])
                            for eachitem in interplatedjnts:
                                if temparmname == "rgt":
                                    tempnumikrmsmp.append([numikrms[i-1][0], np.array(eachitem), numikrms[i-1][2]])
                                else:
                                    tempnumikrmsmp.append([numikrms[i-1][0], numikrms[i-1][1], np.array(eachitem)])
                                tempjawwidthmp.append(jawwidth[i-1])
                            for eachitem in interplatedobjposes:
                                tempobjmsmp.append(base.pg.npToMat4(eachitem[1], eachitem[0]))
                            numikrmsmp.append(tempnumikrmsmp)
                            jawwidthmp.append(tempjawwidthmp)
                            objmsmp.append(tempobjmsmp)
                            # update the keypose to avoid non-continuous linear motion: numikrms and objms
                            if temparmname == "rgt":
                                numikrms[i][1] = tempnumikrmsmp[-1][1]
                            elif temparmname == "lft":
                                numikrms[i][2] = tempnumikrmsmp[-1][2]
                            objms[i] = tempobjmsmp[-1]
                            continue
                    # init robot pose
                    rgtarmjnts = numikrms[i-1][1].tolist()
                    lftarmjnts = numikrms[i-1][2].tolist()
                    robot.movealljnts([numikrms[i-1][0], 0, 0]+rgtarmjnts+lftarmjnts)
                    # assume rgt
                    armname = 'rgt'
                    start = numikrms[i-1][1].tolist()
                    goal = numikrms[i][1].tolist()
                    startjawwidth = jawwidth[i-1][0]
                    if "lft" in endid:
                        armname = 'lft'
                        start = numikrms[i-1][2].tolist()
                        goal = numikrms[i][2].tolist()
                        startjawwidth = jawwidth[i-1][1]
                    starttreesamplerate = 25
                    endtreesamplerate = 30
                    print(armname)
                    print(startjawwidth)
                    ctcallback.setarmname(armname)
                    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, ctcallback=ctcallback,
                                                  starttreesamplerate=starttreesamplerate,
                                                  endtreesamplerate=endtreesamplerate, expanddis=5,
                                                  maxiter=200, maxtime=10.0)
                    tempnumikrmsmp = []
                    tempjawwidthmp = []
                    tempobjmsmp = []
                    if (endid[-1] == "c") or (endid[-1] == "w"):
                        # if the arm is holding an object
                        print("Planning hold motion between ", pathnidlist[i-1], " and ", pathnidlist[i])
                        relpos, relrot = robot.getinhandpose(objpos, objrot, armname)
                        # try:
                        #     [path, sampledpoints] = planner.planninghold(objcm, [relpos, relrot], obstaclecmlist)
                        # except Exception as e:
                        #     print(e)
                        #     print("Motion planning with hold failed! restarting...")
                        #     regrip.removeBadNodes([pathnidlist[i-1][:-1]])
                        #     regrip.removeBadNodes([pathnidlist[i][:-1]])
                        #     breakflag = True
                        #     break
                        path, sampledpoints = planner.planninghold(objcm, [relpos, relrot], obstaclecmlist)
                        if path is False:
                            print("Motion planning with hold failed! restarting...")
                            # regrip.removeBadNodes([pathnidlist[i-1][:-1]])
                            regrip.removeBadNodes([pathnidlist[i][:-1]])
                            breakflag = True
                            break
                        path = smoother.pathsmoothinghold(path, planner, objcm, [relpos, relrot], 30)
                        npath = len(path)
                        for j in range(npath):
                            if armname == 'rgt':
                                tempnumikrmsmp.append([0.0, np.array(path[j]), numikrms[i-1][2]])
                            else:
                                tempnumikrmsmp.append([0.0, numikrms[i-1][1], np.array(path[j])])
                            robot.movearmfk(np.array(path[j]), armname = armname)
                            tempjawwidthmp.append(jawwidth[i-1])
                            objpos, objrot = robot.getworldpose(relpos, relrot, armname)
                            tempobjmsmp.append(base.pg.npToMat4(objrot, objpos))
                    else:
                        # if the arm is not holding an object, the object will be treated as an obstacle
                        print("Planning motion ", pathnidlist[i-1], " and ", pathnidlist[i])
                        objcmcopy = copy.deepcopy(objcm)
                        objcmcopy.setMat(objms[i-1])
                        obstaclecmlistnew = obstaclecmlist + [objcmcopy]
                        # try:
                        #     [path, sampledpoints] = planner.planning(obstaclecmlistnew)
                        # except Exception as e:
                        #     print(e)
                        #     print("Motion planning failed! restarting...")
                        #     regrip.removeBadNodes([pathnidlist[i-1][:-1]])
                        #     regrip.removeBadNodes([pathnidlist[i][:-1]])
                        #     # if pathnidlist[i][:2]=="ho" and pathnidlist[i][-1] == "x":
                        #     #     import robotsim.ur3dual.ur3dualmesh as ur3dualsimmesh
                        #     #     import robotsim.ur3dual.ur3dualball as ur3dualball
                        #     #     import manipulation.grip.robotiq85.rtq85nm as rtq85nm
                        #     #     rgthnd = rtq85nm.newHand(hndid = "rgt")
                        #     #     lfthnd = rtq85nm.newHand(hndid = "lft")
                        #     #     robotmeshgen = ur3dualsimmesh.Ur3DualMesh(rgthand = rgthnd, lfthand = lfthnd)
                        #     #     # robot.movearmfk(start, armname)
                        #     #     # robotmeshgen.genmnp(robot).reparentTo(base.render)
                        #     #     robot.movearmfk(goal, armname)
                        #     #     robotmeshgen.genmnp(robot).reparentTo(base.render)
                        #     #     ur3dball = ur3dualball.Ur3DualBall()
                        #     #     bcndict = ur3dball.genfullactivebcndict(robot)
                        #     #     ur3dball.showbcn(base, bcndict)
                        #     #     objcmcopy.reparentTo(base.render)
                        #     #     objcmcopy.show()
                        #     #     base.run()
                        #     breakflag = True
                        #     break
                        path, sampledpoints = planner.planning(obstaclecmlistnew)
                        if path is False:
                            print("Motion planning failed! restarting...")
                            # regrip.removeBadNodes([pathnidlist[i-1][:-1]])
                            # regrip.removeBadNodes([pathnidlist[i][:-1]])
                            if pathnidlist[i-1] == "begin":
                                regrip.removeBadNodes([pathnidlist[i][:-1]])
                                breakflag = True
                                break
                            if pathnidlist[1] == "end":
                                regrip.removeBadNodes([pathnidlist[i-1][:-1]])
                                breakflag = True
                                break
                            node0 = pathnidlist[i-1][:-1]
                            node1 = pathnidlist[i][:-1]
                            regrip.removeBadEdge(node0, node1)
                            breakflag = True
                            break
                        path = smoother.pathsmoothing(path, planner, 30)
                        npath = len(path)
                        for j in range(npath):
                            if armname == 'rgt':
                                tempnumikrmsmp.append([0.0, np.array(path[j]), numikrms[i-1][2]])
                            else:
                                tempnumikrmsmp.append([0.0, numikrms[i-1][1], np.array(path[j])])
                            tempjawwidthmp.append(jawwidth[i-1])
                            tempobjmsmp.append(objms[i-1])
                    numikrmsmp.append(tempnumikrmsmp)
                    jawwidthmp.append(tempjawwidthmp)
                    objmsmp.append(tempobjmsmp)
            print(i, len(numikrms)-1)
            if breakflag is False:
                # successfully finished!
                return [objmsmp, numikrmsmp, jawwidthmp, originalpathnidlist]
            else:
                # remov node and start new search
                continue