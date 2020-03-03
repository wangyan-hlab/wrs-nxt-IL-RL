import os
from panda3d.core import *
import pandaplotutils.pandactrl as pc
import pandaplotutils.pandageom as pg
import bldgfsettingnear
import environment.bulletcdhelper as bcdh
import motionplanning.rrt.ddrrtconnect as ddrrtc
import motionplanning.smoother2 as sm
import motionplanning.ctcallback as ctcb
import numpy as np
import utiltools.robotmath as rm
import math
import copy
import time


class CtCallback(object):
    def __init__(self):
        self.__jointlimits = [[-135.0, -45.0], [-1.0, 1.0], [30.0, 140.0],
                              [-2.0, 2.0], [0.0, 13.0], [-2.0, 2.0]]
        pass

    @property
    def jointlimits(self):
        return self.__jointlimits

    def iscollided(self, xyzrpy, objcm, obstaclecmlist):
        """

        :param object:
        :param obstaclecdnplist:
        :return:

        author: weiwei, hao
        date: 20190429
        """
        objcm.setPos(xyzrpy[0],xyzrpy[1],xyzrpy[2])
        objcm.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5])

        checker = bcdh.MCMchecker()
        result = checker.isMeshMeshListCollided(objcm, obstaclecmlist)
        # if type(contactpoints) is tuple:
            # print(contactpoints[0])
            # a = contactpoints[0].getManifoldPoint()
            # print(a)
            # b = a.getDistance()
            # print(b)

        # else:
        #     print("listlistlist")
        # print(len(contactpoints))
        # print(type(contactpoints))
        # for i in range(len(contactpoints)):
        #     print(di(contactpoints[i]))

        return result


def readdemopose(filename, basepos):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "r")
    # if f.mode == "r":
    #     contents = f.read()
    #     print(contents)
    f1 = f.readlines()
    poselist = []
    for string in f1:
        strlist = string.strip("[").strip(" ").replace("]", "").rstrip(" ")
        s = " ".join(strlist.split()).replace(" ", ",")
        # print(s)
        list1 = eval(s)
        poselist.append([list1[0], list1[1], list1[2]])
    length = len(f1)

    demopathlist = []
    for n in range(int(length/4)):
        demopos = [np.array([poselist[n*4][0],poselist[n*4][1],poselist[n*4][2]]),
                   np.array([[poselist[n*4+1][0],poselist[n*4+1][1],poselist[n*4+1][2]],
                             [poselist[n*4+2][0],poselist[n*4+2][1],poselist[n*4+2][2]],
                             [poselist[n*4+3][0],poselist[n*4+3][1],poselist[n*4+3][2]]])]
        demopathlist.append(demopos)
    demoposelist = []
    for i in range(len(demopathlist)):
        relpos = demopathlist[i][0]
        relrot = demopathlist[i][1]
        relmat = base.pg.npToMat4(relrot, relpos)
        posL = relmat * basepos
        virtualgoalpos, virtualgoalrot = setPosRPY(objcm, posL)
        demopose = np.array([0, virtualgoalpos[1], virtualgoalpos[2],
                             virtualgoalrot[0], 0, 0], dtype=float)
        demoposelist.append(demopose)
    return demoposelist

def writefile(list, filename):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "w")
    for poses in list:
        # print(poses)
        f.write(str(poses)+'\n')
    f.close()

def setPosRPY(objcm, Mat4):
    RPY = rm.euler_from_matrix(pg.npToMat3(np.array([[Mat4[0][0], Mat4[0][1], Mat4[0][2]],
                                                     [Mat4[1][0], Mat4[1][1], Mat4[1][2]],
                                                     [Mat4[2][0], Mat4[2][1], Mat4[2][2]]])))
    Pos = np.array([Mat4[3][0], Mat4[3][1], Mat4[3][2]])
    objcm.setPos(Pos[0], Pos[1], Pos[2])
    objcm.setRPY(RPY[0], RPY[1], RPY[2])

    return Pos, RPY

def motionplanning(start, goal, fixedobj, maniobj, expdis):
    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, ctcallback=ctcallback,
                                  starttreesamplerate=30, endtreesamplerate=30,
                                  expanddis=expdis, maxiter=1000, maxtime=800)
    obscmlist = [fixedobj]
    starttime = time.perf_counter()
    [path, sampledpoints] = planner.planning2(maniobj, obscmlist)
    if path is False:
        pass
    else:
        smoother = sm.Smoother(objcm)
        path = smoother.pathsmoothing(path, planner)
    endtime = time.perf_counter()
    runtime = endtime - starttime
    print("Run time =", runtime)

    return path, runtime


if __name__=="__main__":
    base = pc.World(camp=[1200, 0, 0], lookatp=[0, 0, 0], up=[0, 0, 1], fov=40, w=1920, h=1080)
    env = bldgfsettingnear.Env()
    # self.env.reparentTo(base.render)
    # objname = "new_LSHAPE.stl"
    objname = "candy.STL"
    # objname = "tenon.STL"
    objcm = env.loadobj(objname)
    # groove = env.loadobj("new_GROOVE.stl")
    # groove = env.loadobj("GROOVEnew.STL")
    # groove = env.loadobj("base2_new.STL")
    # groove = env.loadobj("tenonhole.STL")
    groove = env.loadobj("tenonhole_new_115.STL")
    posG = base.pg.npToMat4(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                            np.array([0, 0, 0]))
    setPosRPY(groove, posG)
    groove.setColor(0, 0, 1, 0.4)
    groove.reparentTo(base.render)

    relpos1 = np.array([0, -87.49,   40.36])
    relrot1 = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
    relmat1 = base.pg.npToMat4(relrot1, relpos1)
    posL1 = relmat1 * posG
    virtualgoalpos1, virtualgoalrot1 = setPosRPY(objcm, posL1)

    ctcallback = CtCallback()
    # start = np.array([virtualgoalpos1[0], virtualgoalpos1[1], virtualgoalpos1[2],
    #                   virtualgoalrot1[0], virtualgoalrot1[1], virtualgoalrot1[2]], dtype=float)
    start = np.array([0, -95, 55, -20, 0, 0], dtype=float)
    goal = np.array([0, -82.5, -2.5, 0, 0, 0], dtype=float)
    print("bbb", ctcallback.iscollided(start, objcm, [groove]))
    print("aaa", ctcallback.iscollided(goal, objcm, [groove]))

    demopathlist = [
        [np.array([ -5.73826664, -79.25563482,  14.92496054]), np.array([[ 0.9981996 ,  0.0533105 , -0.0274875 ],
        [-0.0416214 ,  0.94563808,  0.32254643],
        [ 0.04318831, -0.3208217 ,  0.94615453]])],
        [np.array([ -5.80018528, -78.35461208,  14.89169411]), np.array([[ 0.99820279,  0.05226622, -0.02931364],
        [-0.04039872,  0.9482276 ,  0.31501193],
        [ 0.04426054, -0.31326155,  0.94863494]])],
        [np.array([ -5.53747054, -77.27624206,  13.92599495]), np.array([[ 0.99876397,  0.03807851, -0.03194821],
        [-0.02626875,  0.95001581,  0.31109483],
        [ 0.04219744, -0.30987102,  0.94984174]])],

        [np.array([ -3.11935112, -76.35935833,   8.58060048]), np.array([[ 9.99597583e-01,  6.17025122e-03, -2.76873378e-02],
        [-9.30837334e-05,  9.76764322e-01,  2.14316150e-01],
        [ 2.83664259e-02, -2.14227336e-01,  9.76371877e-01]])],
        [np.array([ -3.0759386 , -75.45105842,   9.44781914]), np.array([[ 0.99965226,  0.00321951, -0.0261727 ],
        [ 0.00232887,  0.97786218,  0.20923676],
        [ 0.02626687, -0.20922492,  0.97751475]])],
        [np.array([ -2.9077887 , -74.48581032,   9.41741975]), np.array([[ 9.99676333e-01,  6.18829202e-03, -2.46724752e-02],
        [-7.97622781e-04,  9.77105055e-01,  2.12756374e-01],
        [ 2.54242102e-02, -2.12667790e-01,  9.76793673e-01]])],
        [np.array([ -2.76688882, -73.44726693,   9.22560804]), np.array([[ 9.99741124e-01,  5.40731047e-03, -2.20995138e-02],
        [-5.41198733e-04,  9.76723537e-01,  2.14501519e-01],
        [ 2.27450382e-02, -2.14434014e-01,  9.76473620e-01]])],
        [np.array([ -2.60037906, -72.31603003,   9.45824538]), np.array([[ 0.99978227,  0.00907408, -0.01878822],
        [-0.00470301,  0.97531228,  0.22078011],
        [ 0.02032787, -0.22064367,  0.97514265]])],
        [np.array([ -1.54096425, -70.13011602,   8.54937833]), np.array([[ 0.99987456,  0.00680028, -0.01430236],
        [-0.00355047,  0.97638197,  0.21602244],
        [ 0.01543363, -0.21594453,  0.97628363]])],
        [np.array([ -0.93197257, -69.78465376,   7.95621096]), np.array([[ 9.99964061e-01,  1.85683689e-04, -8.47094424e-03],
        [ 1.49969138e-03,  9.80096973e-01,  1.98513907e-01],
        [ 8.33918090e-03, -1.98519494e-01,  9.80061425e-01]])],
        [np.array([  0.74487922, -69.72855114,   5.85952825]), np.array([[ 0.99995613,  0.00305078, -0.00886608],
        [-0.00156433,  0.98661524,  0.16305846],
        [ 0.00924492, -0.16303738,  0.98657658]])],
        [np.array([  1.01042125, -69.37454747,   4.84276747]), np.array([[ 0.99998092,  0.00385806, -0.0048102 ],
        [-0.00313187,  0.98975198,  0.14276298],
        [ 0.00531165, -0.14274518,  0.98974521]])],
        [np.array([  0.86478442, -69.72282409,   3.14961593]), np.array([[ 0.99995396,  0.00826717, -0.00486443],
        [-0.00768089,  0.99388018,  0.11019706],
        [ 0.00574571, -0.11015458,  0.99389778]])],
        [np.array([  0.63555423, -71.21651216,   1.5015602 ]), np.array([[ 0.99991912,  0.00898789, -0.00900115],
        [-0.00828383,  0.99711775,  0.07541556],
        [ 0.00965298, -0.07533489,  0.99711156]])],
        [np.array([  0.64673777, -70.90885034,   0.44502828]), np.array([[ 0.99988347,  0.01164553, -0.00986759],
        [-0.01097423,  0.99778893,  0.0655498 ],
        [ 0.0106091 , -0.06543388,  0.99780049]])],
    ]
    demopathlistxyzRPY = []
    for i in range(len(demopathlist)):
        relpos = demopathlist[i][0]
        relrot = demopathlist[i][1]
        relmat = base.pg.npToMat4(relrot, relpos)
        posL = relmat * posG
        virtualgoalpos, virtualgoalrot = setPosRPY(objcm, posL)
        demopose = np.array([0, virtualgoalpos[1], virtualgoalpos[2],
                             virtualgoalrot[0], 0, 0], dtype=float)
        demopathlistxyzRPY.append(demopose)
    print(demopathlistxyzRPY)
    # read rearranged demo poses
    demopathlist1 = readdemopose("Rearrangedtenonpath(1st deriv).txt", posG)
    demopathlist2 = readdemopose("Rearrangedtenonpath(2nd deriv).txt", posG)
    print("Demopathlist 1 is", demopathlist1)
    print("Demopathlist 2 is", demopathlist2)
    testpathlist = []
    pathlist = []
    poseidlist = []

    for pose in demopathlist1:  ## Don't forget to change the output file!
    # for pose in demopathlist2:
        for j in range(len(demopathlistxyzRPY)):
            if (np.around(demopathlistxyzRPY[j], decimals=2) == np.around(pose, decimals=2)).all():
                poseid = j
                poseidlist.append(poseid)
    print('poseidlist is', poseidlist)
    dlist = []
    wholepath = []
    print('length of demopathlistxyzRPY is', len(demopathlistxyzRPY))

    # time_start = time.time()
    # print(time_start)
    plantime = 0
    time_total_s = time.perf_counter()
    for number in range(len(demopathlistxyzRPY)):
        ddd = copy.deepcopy(demopathlistxyzRPY)
        newposeidlist = []
        wholepath = []
        for n in range(number+1):
            newposeidlist.append(poseidlist[n])
        for m in range(len(demopathlistxyzRPY)):
            if m not in newposeidlist:
                ddd[m] = False
        for n in range(len(demopathlistxyzRPY)):
            dlist = [x for x in ddd if x is not False]

        print("%d th pose is inserted" % (number+1))
        print("Inserted poses are", dlist)
        # for pose in dlist:
        #     pose[2] += 1
        dlist.insert(0, start)
        dlist.append(goal)
        testpathlist = dlist

        for i in range(len(testpathlist) - 1):
            print("Now the testpathlist is", testpathlist)

            for pos in testpathlist:
                tmpobjcm = copy.deepcopy(objcm)
                tmpobjcm.setPos(pos[0], pos[1], pos[2])
                tmpobjcm.setRPY(pos[3], pos[4], pos[5])
                tmpobjcm.setColor(1, 0, 0, 1)
                # tmpobjcm.reparentTo(base.render)

            path, runtime = motionplanning(start=testpathlist[i], goal=testpathlist[i+1], fixedobj=groove, maniobj=objcm, expdis=1)
            if path is False:
                print("No path found for this part!!!")
                break
            else:
                if ctcallback.iscollided(testpathlist[i+1], objcm, [groove]):
                    print(ctcallback.iscollided(testpathlist[i+1], objcm, [groove]))
                    newp = np.array([path[-2][0], path[-2][1], path[-2][2], path[-2][3], path[-2][4], path[-2][5]])
                    for index, value in enumerate(demopathlistxyzRPY):
                        if (demopathlistxyzRPY[index] == testpathlist[i+1]).all():
                            demopathlistxyzRPY[index] = newp
                    del path[-1]
                    testpathlist[i + 1] = newp
                print("The path of this part is", path)
                plantime += runtime
                for point in path:
                    wholepath.append(point)
        print("The whole path is", wholepath)
        if wholepath:
            last = np.array([wholepath[-1][0], wholepath[-1][1], wholepath[-1][2], wholepath[-1][3], wholepath[-1][4],
                             wholepath[-1][5]], dtype=float)
            print(last)
            if (last == goal).all():
                break
    time_total_e = time.perf_counter()
    # plantime = time_e - time_s
    print("Planning time is %f s" % plantime)
    print("Total time cost is %f s" % (time_total_e - time_total_s))
    # time_end = time.time()
    # print(time_end)
    # time = time_end - time_start
    # print("Planning time is %d s" % time)
    # writefile(wholepath, "Planned_tenonpath_deriv1.txt")
    # writefile(wholepath, "Planned_tenonpath_deriv2.txt")

    num = 0
    for pos in wholepath:
        num += 0.05
        tmpobjcm = copy.deepcopy(objcm)
        tmpobjcm.setPos(pos[0], pos[1], pos[2])
        tmpobjcm.setRPY(pos[3], pos[4], pos[5])
        tmpobjcm.setColor(1, 1, 0, 1-num*0.07)
        tmpobjcm.reparentTo(base.render)

    base.run()
