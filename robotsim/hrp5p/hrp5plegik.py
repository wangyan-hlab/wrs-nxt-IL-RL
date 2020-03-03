import math
import copy
import numpy as np
import pyception as ep
import utiltools.robotmath as rm

def jacobian(robot, legid="rgt"):
    """
    compute the jacobian matrix of legs

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param legid: a string indicating "rgt" or "lft"
    :return: legjac a 6-by-6 ndarray

    author: weiwei
    date: 20180616
    """

    if legid!="rgt" and legid!="lft":
        raise ep.ValueError

    leglj = robot.rgtleg
    if legid == "lft":
        leglj = robot.lftleg

    legjac = np.zeros((6,len(robot.targetlegjoints)))

    counter = 0
    for i in robot.targetlegjoints:
        a = np.dot(leglj[i]["rotmat"], leglj[i]["rotax"])
        legjac[:, counter] = np.append(np.cross(a, leglj[robot.targetlegjoints[-1]]["linkpos"]-leglj[i]["linkpos"]), a)
        counter += 1

    return legjac

def manipulability(robot, legid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft leg (lst 6-dof)

    :param robot: see the hrp5.Hrp5Robot class
    :param legid: a string indicating "rgt" or "lft"
    :return:

    author: weiwei
    date: 20180616
    """

    legjac = jacobian(robot, legid)
    return math.sqrt(np.linalg.det(np.dot(legjac, legjac.transpose())))

def tcperror(robot, tgtpos, tgtrot, legid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param robot: see the hrp5robot.Hrp5Robot class
    :param legid: a string indicating "rgt" or "lft"
    :param tgtpos: the position of the goal
    :param tgtrot: the rotation of the goal
    :return: a 1-by-6 vector where the first three indicates the displacement in pos,
                the second three indictes the displacement in rot

    author: weiwei
    date: 20180616
    """

    if legid!="rgt" and legid!="lft":
        raise ep.ValueError

    leglj = robot.rgtleg
    if legid == "lft":
        leglj = robot.lftleg

    deltapos = tgtpos - leglj[robot.targetlegjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, leglj[robot.targetlegjoints[-1]]["rotmat"].transpose())

    anglesum = np.trace(deltarot)
    if abs(anglesum-3) < 1e-4:
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

def numik(robot, tgtpos, tgtrot, legid="rgt"):
    """
    solve the ik numerically for the specified legid

    :param robot: see hrp5robot.Hrp5Robot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param legid: a string "rgt" or "lft" indicating the leg that will be solved
    :return: legjnts: a 1-by-6 numpy ndarray

    TODO: change starting joints to initjoints to make the results unique

    author: weiwei
    date: 20180616
    """

    if legid!="rgt" and legid!="lft":
        raise ep.ValueError

    # stablizer
    steplength = 3
    steplengthinc = 7
    legjntssave = robot.getlegjnts(legid)
    legjntsiter = robot.getinitlegjnts(legid)
    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        legjac = jacobian(robot, legid)
        if np.linalg.matrix_rank(legjac) == 6:
            err = tcperror(robot, tgtpos, tgtrot, legid)
            # print("tcperr", err)
            dq = steplength*(np.linalg.lstsq(legjac, err))[0]
            # print(dq, "dq")
            # print(steplength, "steplenght")
            # print(np.linalg.lstsq(legjac, err)[0], "linalg in hrp5plegik")

            # print("dq", dq)
        else:
            print( "The Jacobian Matrix of the specified leg is at singularity")
            # robot.movelegfk(legjntssave, legid)
            # return None
            break
        # print(np.linalg.norm(err))
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm <1e-6:
            # print('goal reached', legjntsiter)
            # print("number of iteration ", i)
            # if robot.chkrng(legjntsiter, legid):
            legjntsreturn = robot.getlegjnts(legid)
            robot.movelegfk(legjntssave, legid)
            return legjntsreturn
            # else:
            #     # import hrp5nplot
            #     # import manipulation.grip.hrp5three.hrp5three as handpkg
            #     # # hrp5nplot.plotstick(base.render, robot)
            #     # hrp5nmnp = hrp5nplot.genmnp(robot, handpkg)
            #     # hrp5nmnp.reparentTo(base.render)
            #     # print("out of range")
            #     # robot.movelegfk(legjntssave, legid)
            #     # return None
            #     break
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm-errnormlast) < 1e-6:
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
            legjntsiter += dq
            legjntsiter = rm.cvtRngPM180(legjntsiter)
            bdragged, jntangles = robot.chklegrngdrag(legjntsiter, legid)
            legjntsiter[:] = jntangles[:]
            robot.movelegfk(jntangles, legid)
            # print(jntangles)
            # import manipulation.grip.hrp5pf3.hrp5pf3 as hrp5pf3
            # import hrp5pmesh
            # lfthnd = hrp5pf3.Hrp5pf3(hndid='lft')
            # rgthnd = hrp5pf3.Hrp5pf3(hndid='rgt')
            # hrp5pmgen = hrp5pmesh.Hrp5PMesh(lfthand=lfthnd, rgthand=rgthnd)
            # hrp5pmnp = hrp5pmgen.genmnp(robot, togglejntscoord=False)
            # hrp5pmnp.reparentTo(base.render)
        errnormlast = errnorm
        # print(errnorm)
    robot.movelegfk(legjntssave, legid)
    return None