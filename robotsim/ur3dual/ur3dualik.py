import math
import copy
import numpy as np
import utiltools.robotmath as rm

def eubik(pos, armname="rgt"):
    """
    compute the euristic waist rotation
    ew = euristic waist

    :param pos: object position
    :return: waistangle in degree

    author: weiwei
    date: 20170410
    """

    # waist is always 0
    waistangle = 0
    return waistangle

def jacobian(robot, armname="rgt"):
    """
    compute the jacobian matrix of the targetjoints of rgt or lft arm

    :param robot: see the robot class
    :param armname: a string indicating "rgt" or "lft"
    :return: armjac a 6-by-x matrix

    author: weiwei
    date: 20161202
    """

    if armname!="rgt" and armname!="lft":
        raise ValueError

    armlj = robot.rgtarm
    if armname == "lft":
        armlj = robot.lftarm

    armjac = np.zeros((6,len(robot.targetjoints)))
    counter = 0
    for i in robot.targetjoints:
        a = np.dot(armlj[i]["rotmat"], armlj[i]["rotax"])
        armjac[:, counter] = np.append(np.cross(a, armlj[robot.targetjoints[-1]]["linkpos"]-armlj[i]["linkpos"]), a)
        counter += 1

    return armjac


def manipulability(ur3dualrobot, armname="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param armname: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(ur3dualrobot, armname)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))

def tcperror(robot, tgtpos, tgtrot, armname="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param robot: see the robot class
    :param armname: a string indicating "rgt" or "lft"
    :param tgtpos: the position of the goal
    :param tgtrot: the rotation of the goal
    :return: a 1-by-6 vector where the first three indicates the displacement in pos,
                the second three indictes the displacement in rot

    author: weiwei
    date: 20180827
    """

    if armname!="rgt" and armname!="lft":
        raise ValueError

    armlj = robot.rgtarm
    if armname == "lft":
        armlj = robot.lftarm

    deltapos = tgtpos - armlj[robot.targetjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, armlj[robot.targetjoints[-1]]["rotmat"].transpose())

    if np.allclose(deltarot, np.eye(3)):
        deltaw = np.array([0.0,0.0,0.0])
    elif np.allclose(deltarot, np.array([1.0, -1.0, -1.0])):
        deltaw = np.array(math.pi, 0.0, 0.0)
    elif np.allclose(deltarot, np.array([-1.0, 1.0, -1.0])):
        deltaw = np.array(0.0, math.pi, 0.0)
    elif np.allclose(deltarot, np.array([-1.0, -1.0, 1.0])):
        deltaw = np.array(0.0, 0.0, math.pi)
    else:
        tempvec = np.array([deltarot[2, 1] - deltarot[1, 2], deltarot[0, 2] - deltarot[2, 0], deltarot[1, 0] - deltarot[0, 1]])
        tempveclength = np.linalg.norm(tempvec)
        deltaw = math.atan2(tempveclength, np.trace(deltarot)-1.0)/tempveclength*tempvec
    return np.append(deltapos, deltaw)

def numik(robot, tgtpos, tgtrot, armname="rgt"):
    """
    solve the ik numerically for the specified armname

    :param robot: see the ur3dual.Ur3SglRobot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armname: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-6 numpy ndarray

    author: weiwei
    date: 20180203
    """

    if armname!="rgt" and armname!="lft":
        raise ValueError

    deltapos = tgtpos - robot.getarm(armname)[1]["linkpos"]
    if np.linalg.norm(deltapos) > 800.0:
        return None

    # stablizer
    steplength = 5
    steplengthinc = 10
    armjntssave = robot.getarmjnts(armname)
    armjntsiter = robot.getinitarmjnts(armname)
    robot.movearmfk(armjntsiter, armname)
    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        armjac = jacobian(robot, armname)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(robot, tgtpos, tgtrot, armname)
            dq = steplength * (np.linalg.lstsq(armjac, err, rcond=None))[0]
        else:
            print("The Jacobian Matrix of the specified arm is at singularity")
            break
        # print np.linalg.norm(err)
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm < 1e-6:
            # print 'goal reached', armjntsiter
            # print "number of iteration ", i
            # if hrp5nrobot.chkrng(armjntsiter, armname):
            armjntsreturn = robot.getarmjnts(armname)
            robot.movearmfk(armjntssave, armname)
            return armjntsreturn
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm - errnormlast) < 1e-6:
                nlocalencountered += 1
                # print "local minima at iteration", i
                # print "n local encountered", nlocalencountered
                steplength = 3
                steplengthinc = 7
                if nlocalencountered > 2:
                    break
            else:
                if steplength < 50:
                    steplength = steplength + steplengthinc
            armjntsiter += dq
            # armjntsiter = rm.cvtRngPM180(armjntsiter)
            armjntsiter = rm.cvtRngPM360(armjntsiter)
            bdragged, jntangles = robot.chkrngdrag(armjntsiter, armname)
            armjntsiter[:] = jntangles[:]
            robot.movearmfk(jntangles, armname)
            # print jntangles
            # ur5sglstick = ur5sglmeshgen.gensnp(ur5sglrobot)
            # ur5sglstick.reparentTo(base.render)
        errnormlast = errnorm
    #     print errnorm
    # ur5sglmnp = ur5sglmeshgen.genmnp(ur5sglrobot)
    # ur5sglmnp.reparentTo(base.render)
    robot.movearmfk(armjntssave, armname)
    return None

def numikr(robot, tgtpos, tgtrot, armname="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)

    :param robot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armname:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20161216, sapporo
    """

    # the following code is deprecated
    # # anglewai means the init angle of waist
    # anglewai = robot.getjntwaist()
    # anglewa = eubik(tgtpos, armname)
    # robot.movewaist(anglewa)
    # armjnts = numik(ur3dualrobot, tgtpos, tgtrot, armname)
    # robot.movewaist(anglewai)
    # if armjnts is None:
    #     return None
    # else:
    #     return [anglewa, armjnts]

    # numikr is the same as numik
    numikresult = numik(robot, tgtpos, tgtrot, armname)
    if numikresult is not None:
        return [0.0, numikresult]
    else:
        return None

def numikmsc(robot, tgtpos, tgtrot, msc, armname="rgt"):
    """
    solve the ik numerically for the specified armname with manually specified starting configuration (msc)

    :param robot: see the robot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armname: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-6 numpy ndarray

    author: weiwei
    date: 20180808, osaka
    """

    if armname!="rgt" and armname!="lft":
        raise ValueError

    # stablizer
    steplength = 5
    steplengthinc = 10
    armjntssave = robot.getarmjnts(armname)
    armjntsiter = copy.deepcopy(msc)
    robot.movearmfk(armjntsiter, armname)
    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        armjac = jacobian(robot, armname)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(robot, tgtpos, tgtrot, armname)
            dq = steplength * (np.linalg.lstsq(armjac, err, rcond=None))[0]
        else:
            print("The Jacobian Matrix of the specified arm is at singularity")
            break
        # print np.linalg.norm(err)
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm < 1e-6:
            # print 'goal reached', armjntsiter
            # print "number of iteration ", i
            # if hrp5nrobot.chkrng(armjntsiter, armname):
            armjntsreturn = robot.getarmjnts(armname)
            robot.movearmfk(armjntssave, armname)
            return armjntsreturn
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm - errnormlast) < 1e-6:
                nlocalencountered += 1
                print("local minima at iteration", i)
                print("n local encountered", nlocalencountered)
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
            bdragged, jntangles = robot.chkrngdrag(armjntsiter, armname)
            armjntsiter[:] = jntangles[:]
            robot.movearmfk(jntangles, armname)
            # print jntangles
            # ur5sglstick = ur5sglmeshgen.gensnp(ur5sglrobot)
            # ur5sglstick.reparentTo(base.render)
        errnormlast = errnorm
    #     print errnorm
    # ur5sglmnp = ur5sglmeshgen.genmnp(ur5sglrobot)
    # ur5sglmnp.reparentTo(base.render)
    robot.movearmfk(armjntssave, armname)
    return None

def numikrmsc(robot, tgtpos, tgtrot, msc, armname="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)
    with manually specified starting configuration (msc)

    :param robot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armname:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20180808, osaka
    """

    # the following code is deprecated
    # # anglewai means the init angle of waist
    # anglewai = ur3dualrobot.getjntwaist()
    # anglewa = eubik(tgtpos, armname)
    # ur3dualrobot.movewaist(anglewa)
    # armjnts = numik(ur3dualrobot, tgtpos, tgtrot, armname)
    # ur3dualrobot.movewaist(anglewai)
    # if armjnts is None:
    #     return None
    # else:
    #     return [anglewa, armjnts]

    # numikr is the same as numik
    numikresult = numikmsc(robot, tgtpos, tgtrot, msc, armname)
    if numikresult is not None:
        return [0.0, numikresult]
    else:
        return None

def numikreltcp(robot, deltax, deltay, deltaz, armname = 'rgt'):
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

    armlj = robot.rgtarm
    if armname == "lft":
        armlj = robot.lftarm

    tgtpos = armlj[-1]['linkend']
    tgtrot = armlj[-1]['rotmat']
    newtgtpos = tgtpos + np.array([deltax, deltay, deltaz])
    return numik(robot, newtgtpos, tgtrot, armname)

if __name__=="__main__":
    pos = [300,300,0]
    print(eubik(pos))

    try:
        print(math.asin(145/np.linalg.norm(pos[0:1])))
    except:
        print("nontriangle")