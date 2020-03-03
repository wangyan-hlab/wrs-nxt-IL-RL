import math
import copy
import numpy as np
import pyception as ep
import utiltools.robotmath as rm

def eubik(pos, armid="rgt"):
    """
    compute the euristic waist rotation
    ew = euristic waist

    :param pos: object position
    :return: waistangle in degree

    author: weiwei
    date: 20170410
    """

    anglecomponent1 = 0
    try:
        anglecomponent1 = math.asin(80/np.linalg.norm(pos[0:2]))
    except:
        pass
    waistangle = (math.atan2(pos[1], pos[0]) + anglecomponent1)*180/math.pi
    if armid=="lft":
        waistangle = 180.0-2*math.atan2(pos[0], pos[1])*180.0/math.pi-waistangle
    return waistangle

def jacobian(robot, armid="rgt"):
    """
    compute the jacobian matrix of the last 6 dof for rgt or lft arm

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :return: armjac a 6-by-6 ndarray

    author: weiwei
    date: 20161202
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = robot.rgtarm
    if armid == "lft":
        armlj = robot.lftarm

    armjac = np.zeros((6,len(robot.targetjoints)))

    counter = 0
    for i in robot.targetjoints:
        a = np.dot(armlj[i]["rotmat"], armlj[i]["rotax"])
        armjac[:, counter] = np.append(np.cross(a, armlj[robot.targetjoints[-1]]["linkpos"]-armlj[i]["linkpos"]), a)
        counter += 1

    return armjac

def manipulability(robot, armid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft arm (lst 6-dof)

    :param robot: see the hrp5.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(robot, armid)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))

def tcperror(robot, tgtpos, tgtrot, armid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param robot: see the hrp5robot.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :param tgtpos: the position of the goal
    :param tgtrot: the rotation of the goal
    :return: a 1-by-6 vector where the first three indicates the displacement in pos,
                the second three indictes the displacement in rot

    author: weiwei
    date: 20161205
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = robot.rgtarm
    if armid == "lft":
        armlj = robot.lftarm

    deltapos = tgtpos - armlj[robot.targetjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, armlj[robot.targetjoints[-1]]["rotmat"].transpose())

    anglesum = np.trace(deltarot)
    if anglesum is 3:
        deltaw = np.array([0,0,0])
    else:
        nominator = anglesum - 1
        if nominator > 2:
            nominator = 2
        if nominator < -2:
            nominator = -2
        theta = math.acos(nominator / 2.0)
        if theta == 0:
            deltaw = np.array([0, 0, 0])
        else:
            deltaw = (theta / (2 * math.sin(theta))) * (np.array([deltarot[2, 1] - deltarot[1, 2], \
                                                                  deltarot[0, 2] - deltarot[2, 0], \
                                                                  deltarot[1, 0] - deltarot[0, 1]]))
    return np.append(deltapos, deltaw)

def numik(robot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik numerically for the specified armid

    :param robot: see hrp5robot.Hrp5Robot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armid: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-6 numpy ndarray

    TODO: change starting joints to initjoints to make the results unique

    author: weiwei
    date: 20161205
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    # stablizer
    steplength = 5
    steplengthinc = 10
    armjntssave = robot.getarmjnts(armid)
    armjntsiter = copy.deepcopy(armjntssave)
    # armjntsiter = robot.getinitarmjnts(armid)#Rerun DB!!!

    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        armjac = jacobian(robot, armid)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(robot, tgtpos, tgtrot, armid)
            dq = steplength*(np.linalg.lstsq(armjac, err))[0]
        else:
            print( "The Jacobian Matrix of the specified arm is at singularity")
            # robot.movearmfk(armjntssave, armid)
            # return None
            break
        # print(np.linalg.norm(err))
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm <1e-6 :
            # print('goal reached', armjntsiter)
            # print("number of iteration ", i)
            # if robot.chkrng(armjntsiter, armid):
            armjntsreturn = robot.getarmjnts(armid)
            robot.movearmfk(armjntssave, armid)
            return armjntsreturn
            # else:
            #     # import hrp5nplot
            #     # import manipulation.grip.hrp5three.hrp5three as handpkg
            #     # # hrp5nplot.plotstick(base.render, robot)
            #     # hrp5nmnp = hrp5nplot.genmnp(robot, handpkg)
            #     # hrp5nmnp.reparentTo(base.render)
            #     # print("out of range")
            #     # robot.movearmfk(armjntssave, armid)
            #     # return None
            #     break
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm-errnormlast) < 1e-6 :
                # nlocalencountered += 1
                # print("local minima at iteration", i)
                # print("n local encountered", nlocalencountered)
                # steplength = 3
                # steplengthinc = 7
                # if nlocalencountered > 1:
                #     break
                break
            else:
                if steplength < 50:
                    steplength = steplength+steplengthinc
            armjntsiter += dq
            armjntsiter = rm.cvtRngPM180(armjntsiter)
            # print(armjntsiter)
            # for i in range(armjntsiter.shape[0]):
            #     armjntsiter[i] = armjntsiter[i]%360
            # print(armjntsiter)
            # the robot may encounter overrange errors in the first few iterations
            # use i<50 to avoid these errors
            # if robot.chkrng(armjntsiter, armid) or i < 30:
            #     # print(armjntsiter)
            #     robot.movearmfk(armjntsiter, armid)
            #     # import hrp5plot
            #     # hrp5plot.plotstick(base.render, hrp5robot)
            #     # hrp5mnp = hrp5plot.genHrp5mnp(hrp5robot)
            #     # hrp5mnp.reparentTo(base.render)
            #     # nxtmnp = nxtplot.genNxtmnp(nxtrobot)
            #     # nxtmnp.reparentTo(base.render)
            #     # import hrp5nplot
            #     # import manipulation.grip.hrp5three.hrp5three as handpkg
            #     # # hrp5nplot.plotstick(base.render, robot)
            #     # hrp5nmnp = hrp5nplot.genHrp5Nmnp_nm(robot, handpkg)
            #     # hrp5nmnp.reparentTo(base.render)
            # else:
            #     # import hrp5plot
            #     # hrp5plot.plotstick(base.render, hrp5robot)
            #     robot.movearmfk(armjntssave, armid)
            #     return None
            bdragged, jntangles = robot.chkrngdrag(armjntsiter, armid)
            armjntsiter[:] = jntangles[:]
            # print(jntangles)
            robot.movearmfk(jntangles, armid)
        errnormlast = errnorm
        # print(errnorm)
    robot.movearmfk(armjntssave, armid)
    return None

def numikr(robot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)

    :param hrp5robot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armid:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20161216, sapporo
    """

    # anglewai means the init angle of waist
    anglewai = robot.getjntwaist()
    anglewa = eubik(tgtpos, armid)
    robot.movewaist(anglewa)
    armjnts = numik(robot, tgtpos, tgtrot, armid)
    robot.movewaist(anglewai)
    if armjnts is None:
        return None
    else:
        return [anglewa, armjnts]

    # numikresult = numik(robot, tgtpos, tgtrot, armid)
    # if numikresult is not None:
    #     return [0.0, numikresult]
    # else:
    #     return None

# if __name__=="__main__":
#     pos = [300,300,0]
#     print(eubik(pos))
#
#     try:
#         print(math.asin(145/np.linalg.norm(pos[0:1])))
#     except:
#         print("nontriangle")

def numikmsc(robot, tgtpos, tgtrot,msc, armid="rgt"):
    """
    solve the ik numerically for the specified armid with manually specified starting configuration (msc)

    :param robot: see hrp5robot.Hrp5Robot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armid: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-9 numpy ndarray

    author: weiwei
    date: 20180808, osaka
    """

    if armid != "rgt" and armid != "lft":
        raise ep.ValueError

        # stablizer
    steplength = 5
    steplengthinc = 10
    armjntssave = robot.getarmjnts(armid)
    armjntsiter = copy.deepcopy(msc)
    robot.movearmfk(armjntsiter, armid)
    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        armjac = jacobian(robot, armid)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(robot, tgtpos, tgtrot, armid)
            dq = steplength * (np.linalg.lstsq(armjac, err))[0]
        else:
            print( "The Jacobian Matrix of the specified arm is at singularity")
            break
        # print(np.linalg.norm(err))
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm < 1e-4:
            # print('goal reached', armjntsiter)
            # print("number of iteration ", i)
            # if hrp5nrobot.chkrng(armjntsiter, armid):
            armjntsreturn = robot.getarmjnts(armid)
            robot.movearmfk(armjntssave, armid)
            return armjntsreturn
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm - errnormlast) < 1e-4:
                nlocalencountered += 1
                print( "local minima at iteration", i)
                print( "n local encountered", nlocalencountered)
                steplength = 3
                steplengthinc = 7
                if nlocalencountered > 2:
                    break
            else:
                if steplengthinc == 10 and steplength < 50:
                    steplength = steplength + steplengthinc
            armjntsiter += dq
            # armjntsiter = rm.cvtRngPM180(armjntsiter)
            armjntsiter = rm.cvtRngPM360(armjntsiter)
            bdragged, jntangles = robot.chkrngdrag(armjntsiter, armid)
            armjntsiter[:] = jntangles[:]
            robot.movearmfk(jntangles, armid)
            # print(jntangles)
            # ur5sglstick = ur5sglmeshgen.gensnp(ur5sglrobot)
            # ur5sglstick.reparentTo(base.render)
        errnormlast = errnorm
    #     print(errnorm)
    # ur5sglmnp = ur5sglmeshgen.genmnp(ur5sglrobot)
    # ur5sglmnp.reparentTo(base.render)
    robot.movearmfk(armjntssave, armid)
    return None


# def numikmsc(robot, tgtpos, tgtrot, msc, armid="rgt"):
#     """
#     solve the ik numerically for the specified armid with manually specified starting configuration (msc)
#
#     :param robot: see hrp5robot.Hrp5Robot class
#     :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
#     :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
#     :param armid: a string "rgt" or "lft" indicating the arm that will be solved
#     :return: armjnts: a 1-by-9 numpy ndarray
#
#     author: weiwei
#     date: 20180808, osaka
#     """
#
#     if armid!="rgt" and armid!="lft":
#         raise ep.ValueError
#
#     # stablizer
#     steplength = 5
#     steplengthinc = 10
#     armjntssave = robot.getarmjnts(armid)
#     armjntsiter = copy.deepcopy(msc)
#     robot.movearmfk(armjntsiter, armid)
#     errnormlast = 0.0
#     nlocalencountered = 0
#
#
#     for i in range(100):
#         armjac = jacobian(robot, armid)
#         if np.linalg.matrix_rank(armjac) == 6:
#             err = tcperror(robot, tgtpos, tgtrot, armid)
#             dq = steplength * (np.linalg.lstsq(armjac, err))[0]
#         else:
#             print("The Jacobian Matrix of the specified arm is at singularity")
#             break
#         # print(np.linalg.norm(err))
#         # if np.linalg.norm(err)<1e-4:
#         errnorm = np.linalg.norm(err)
#         if errnorm < 1e-6:  #1e-6
#             # print('goal reached', armjntsiter)
#             # print("number of iteration ", i)
#             # if hrp5nrobot.chkrng(armjntsiter, armid):
#             armjntsreturn = robot.getarmjnts(armid)
#             robot.movearmfk(armjntssave, armid)
#             return armjntsreturn
#         else:
#             # todo dq definition
#             # judge local minima
#             if abs(errnorm - errnormlast) < 1e-6:  #1e-6
#                 # nlocalencountered += 1
#                 # print("local minima at iteration", i)
#                 # print("n local encountered", nlocalencountered)
#                 # print("armjntstest",robot.getarmjnts(armid), armid)
#                 # steplength = 3
#                 # steplengthinc = 7
#                 # if nlocalencountered > 2:
#                 break
#             else:
#                 # if steplengthinc == 10 and steplength < 50:
#                 #     steplength = steplength + steplengthinc
#
#                 if steplength < 50:
#                     steplength = steplength + steplengthinc
#             armjntsiter += dq
#             armjntsiter = rm.cvtRngPM180(armjntsiter)
#
#             # armjntsiter += dq
#             # # armjntsiter = rm.cvtRngPM180(armjntsiter)
#             # armjntsiter = rm.cvtRngPM360(armjntsiter)
#             bdragged, jntangles = robot.chkrngdrag(armjntsiter, armid)
#             armjntsiter[:] = jntangles[:]
#             robot.movearmfk(jntangles, armid)
#             # print(jntangles)
#             # ur5sglstick = ur5sglmeshgen.gensnp(ur5sglrobot)
#             # ur5sglstick.reparentTo(base.render)
#         errnormlast = errnorm
#     #     print(errnorm)
#     # ur5sglmnp = ur5sglmeshgen.genmnp(ur5sglrobot)
#     # ur5sglmnp.reparentTo(base.render)
#     robot.movearmfk(armjntssave, armid)
#     return None

def numikrmsc(robot, tgtpos, tgtrot, msc, armid="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)
    with manually specified starting configuration (msc)

    :param ur3dualrobot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armid:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20180808, osaka
    """

    # the following code is deprecated
    # anglewai means the init angle of waist
    # anglewai = robot.getjntwaist()
    # anglewa = eubik(tgtpos, armid)
    # robot.movewaist(anglewa)
    # armjnts = numik(robot, tgtpos, tgtrot, armid)
    # robot.movewaist(anglewai)
    # if armjnts is None:
    #     return None
    # else:
    #     return [anglewa, armjnts]
    anglewai = robot.getjntwaist()
    print( anglewai, "anglewai")
    anglewa = msc[0]
    print( anglewa , "anglewa")
    robot.movewaist(anglewa)

    numikresult = numikmsc(robot, tgtpos, tgtrot, msc[1], armid) #Orig solution
    robot.movewaist(anglewai)

    if numikresult is not None:
        return [anglewa, numikresult]
    else:
        return None


#BACKUP numikrmsc
    # def numikrmsc(robot, tgtpos, tgtrot, msc, armid="rgt"):
    #     """
    #     solve the ik of the specified arm, waist is included (r means redundant)
    #     with manually specified starting configuration (msc)
    #
    #     :param ur3dualrobot:
    #     :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    #     :param tgtrot:
    #     :param armid:
    #     :return: [waist, shoulder, 1-by-6 armjnts]
    #
    #     author: weiwei
    #     date: 20180808, osaka
    #     """
    #
    #     # the following code is deprecated
    #     # anglewai means the init angle of waist
    #     # anglewai = robot.getjntwaist()
    #     # anglewa = eubik(tgtpos, armid)
    #     # robot.movewaist(anglewa)
    #     # armjnts = numik(robot, tgtpos, tgtrot, armid)
    #     # robot.movewaist(anglewai)
    #     # if armjnts is None:
    #     #     return None
    #     # else:
    #     #     return [anglewa, armjnts]
    #
    #     numikresult = numikmsc(robot, tgtpos, tgtrot, msc, armid)  # Orig solution
    #     if numikresult is not None:
    #         return [0.0, numikresult]
    #     else:
    #         return None
    #     # My solution
    #     # anglewai = robot.getjntwaist()
    #     # anglewa = eubik(tgtpos, armid)
    #     # robot.movewaist(anglewa)
    #     # # armjnts = numik(ur3dualrobot, tgtpos, tgtrot, armid)
    #     # numikresult = numikmsc(robot, tgtpos, tgtrot, msc, armid)
    #     # robot.movewaist(anglewai)
    #     # if numikresult is None:
    #     #     return None
    #     # else:
    #     #     return [anglewa, numikresult]


def numikreltcp(ur3dualrobot, deltax, deltay, deltaz, armid = 'rgt'):
    """
    add deltax, deltay, deltaz to the tcp
    tcp is link[-1].linkpos
    since the function is relative, moving link[-1].linkend is the same

    :param deltax: float
    :param deltay:
    :param deltaz:
    :return:

    author: weiwei
    date: 20170412
    """

    armlj = ur3dualrobot.rgtarm
    if armid == "lft":
        armlj = ur3dualrobot.lftarm

    tgtpos = armlj[-1]['linkend']
    tgtrot = armlj[-1]['rotmat']
    newtgtpos = tgtpos + np.array([deltax, deltay, deltaz])
    return numik(ur3dualrobot, newtgtpos, tgtrot, armid)

def anaik(robot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik analytically for the specified armid

    :param robot: see the ur3dual.Ur3SglRobot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armid: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-6 numpy ndarray

    author: weiwei
    date: 20180203
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    # stablizer
    steplength = 5
    steplengthinc = 10
    armjntssave = robot.getarmjnts(armid)
    armjntsiter = robot.getinitarmjnts(armid)
    robot.movearmfk(armjntsiter, armid)
    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        armjac = jacobian(robot, armid)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(robot, tgtpos, tgtrot, armid)
            dq = steplength * (np.linalg.lstsq(armjac, err))[0]
        else:
            print( "The Jacobian Matrix of the specified arm is at singularity")
            break
        # print(np.linalg.norm(err))
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm < 1e-6:
            # print('goal reached', armjntsiter)
            # print("number of iteration ", i)
            # if hrp5nrobot.chkrng(armjntsiter, armid):
            armjntsreturn = robot.getarmjnts(armid)
            robot.movearmfk(armjntssave, armid)
            return armjntsreturn
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm - errnormlast) < 1e-6:
                nlocalencountered += 1
                print( "local minima at iteration", i)
                print( "n local encountered", nlocalencountered)
                steplength = 3
                steplengthinc = 7
                if nlocalencountered > 2:
                    break
            else:
                if steplengthinc == 10 and steplength < 50:
                    steplength = steplength + steplengthinc
            armjntsiter += dq
            # armjntsiter = rm.cvtRngPM180(armjntsiter)
            armjntsiter = rm.cvtRngPM360(armjntsiter)
            bdragged, jntangles = robot.chkrngdrag(armjntsiter, armid)
            armjntsiter[:] = jntangles[:]
            robot.movearmfk(jntangles, armid)
            # print(jntangles)
            # ur5sglstick = ur5sglmeshgen.gensnp(ur5sglrobot)
            # ur5sglstick.reparentTo(base.render)
        errnormlast = errnorm
    #     print(errnorm)
    # ur5sglmnp = ur5sglmeshgen.genmnp(ur5sglrobot)
    # ur5sglmnp.reparentTo(base.render)
    robot.movearmfk(armjntssave, armid)
    return None


if __name__=="__main__":
    pos = [300,300,0]
    print( eubik(pos))

    try:
        print( math.asin(145/np.linalg.norm(pos[0:1])))
    except:
        print( "nontriangle")