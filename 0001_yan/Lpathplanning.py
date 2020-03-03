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
        demopose = np.array([virtualgoalpos[0], 0, virtualgoalpos[2],
                             0, virtualgoalrot[1], 0], dtype=float)
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
                                  expanddis=expdis, maxiter=500, maxtime=400)
    obscmlist = [fixedobj]
    starttime = time.perf_counter()
    [path, sampledpoints] = planner.planning2(maniobj, obscmlist)
    if path is False:
        pass
    else:
        smoother = sm.Smoother(objcm)
        # smoother = sm.Smoother()
        path = smoother.pathsmoothing(path, planner)
    endtime = time.perf_counter()
    runtime = endtime - starttime
    print("Run time =", runtime)

    return path, runtime


if __name__=="__main__":
    base = pc.World(camp=[0, 800, 0], lookatp=[0, 0, 0], up=[0, 0, 1], fov=40, w=1920, h=1080)
    env = bldgfsettingnear.Env()
    # self.env.reparentTo(base.render)
    objname = "new_LSHAPE.stl"
    objcm = env.loadobj(objname)
    groove = env.loadobj("new_GROOVE.stl")
    # groove = env.loadobj("GROOVEnew.STL")
    # groove = env.loadobj("groove.STL")
    posG = base.pg.npToMat4(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                            np.array([0, 0, 0]))
    setPosRPY(groove, posG)
    groove.setColor(0, 0, 1, 0.4)
    groove.reparentTo(base.render)

    relpos1 = np.array([-96.31549327,  0,  60])
    relrot1 = np.array([[ 0.99985765, -0.01595814, -0.00546159],
       [ 0.01591153,  0.99983752, -0.00847297],
       [ 0.00559591,  0.00838475,  0.99994921]])
    relmat1 = base.pg.npToMat4(relrot1, relpos1)
    posL1 = relmat1 * posG
    virtualgoalpos1, virtualgoalrot1 = setPosRPY(objcm, posL1)

    ctcallback = CtCallback()
    start = np.array([-96, 0, 60, 0, 15, 0], dtype=float)
    goal = np.array([-45.2, 0, 30.2, 0, 0, 0], dtype=float)
    # goal = np.array([-66, 0, 30.2, 0, 0, 0], dtype=float)

    demolist = [
       [ np.array([-56.34711289,  -2.3277788 ,  41.486317  ]), np.array([[ 0.957233  , -0.02124666,  0.28853689],
       [ 0.0281112 ,  0.99941128, -0.01966753],
       [-0.28794916,  0.02693747,  0.95726672]])],
       [ np.array([-55.29537773,  -2.51049493,  40.57262921]), np.array([[ 0.95681391, -0.01526481,  0.2902998 ],
       [ 0.02533061,  0.99920006, -0.03094758],
       [-0.2895952 ,  0.03696457,  0.95643522]])],
       [ np.array([-56.28811304,  -3.55672667,  40.3790682 ]), np.array([[ 0.96447225, -0.01227223,  0.2638989 ],
       [ 0.02384718,  0.99888664, -0.04070254],
       [-0.26310561,  0.04554969,  0.96369113]])],
       [ np.array([-55.37825773,  -1.68474561,  39.82546385]), np.array([[ 0.96550984, -0.01636351,  0.25985156],
       [ 0.02114006,  0.99965474, -0.01559763],
       [-0.25950664,  0.02055303,  0.96552264]])],
       [ np.array([-54.50492747,  -1.62321191,  39.95768752]), np.array([[ 0.96517502, -0.02170076,  0.26070336],
       [ 0.02545853,  0.99961491, -0.0110453 ],
       [-0.26036325,  0.01729778,  0.96535576]])],
       [ np.array([-54.99177087,  -1.22233754,  39.41459372]), np.array([[ 0.96905854, -0.01542571,  0.24634838],
       [ 0.01894627,  0.99974934, -0.01192694],
       [-0.24610265,  0.01622536,  0.96910796]])],
       [ np.array([-54.73268736,  -0.90911092,  39.26989627]), np.array([[ 0.96947976, -0.01360526,  0.24479381],
       [ 0.01481142,  0.99988564, -0.0030869 ],
       [-0.24472382,  0.00661853,  0.96957025]])],
       [ np.array([-56.56095666,  -0.96448707,  38.51783869]), np.array([[ 0.97539864, -0.01540023,  0.21990957],
       [ 0.0175    ,  0.99981796, -0.00760326],
       [-0.21975248,  0.01126463,  0.97549068]])],
       [ np.array([-57.41808535,  -1.07939   ,  38.53897999]), np.array([[ 0.97963831, -0.00684274,  0.20065332],
       [ 0.00829576,  0.99994509, -0.00640148],
       [-0.20059851,  0.00793565,  0.97964136]])],
       [ np.array([-59.44218587,  -1.35169849,  37.79899559]), np.array([[ 0.98397869, -0.00628695,  0.17817539],
       [ 0.00843703,  0.99990049, -0.01131218],
       [-0.17808655,  0.01263419,  0.98393369]])],
       [ np.array([-60.95621057,  -0.5953807 ,  37.39275103]), np.array([[ 0.98607025,  0.0026192 ,  0.16630839],
       [-0.00112822,  0.99995842, -0.00905904],
       [-0.1663252 ,  0.00874515,  0.98603221]])],
       [ np.array([-62.79357403,  -1.05823876,  36.48028262]), np.array([[ 0.99027563,  0.00118221,  0.13911419],
       [ 0.00111986,  0.99986365, -0.01646864],
       [-0.13911473,  0.01646428,  0.99013946]])],
       [ np.array([-64.80450837,  -0.23494515,  34.82992182]), np.array([[ 0.99401502, -0.00158832,  0.10923094],
       [ 0.00259749,  0.99995524, -0.00909716],
       [-0.10921163,  0.00932654,  0.9939748 ]])],
       [ np.array([-66.22917913,   0.1118955 ,  34.28614898]), np.array([[ 0.99455811, -0.0040383 ,  0.1041043 ],
       [ 0.00425792,  0.99998906, -0.00188753],
       [-0.10409555,  0.0023206 ,  0.9945646 ]])],
       [ np.array([-67.10650641,  -0.1193292 ,  33.93423328]), np.array([[ 0.99609031, -0.01031331,  0.08773627],
       [ 0.01003265,  0.99994311,  0.00363922],
       [-0.08776881, -0.00274479,  0.99613706]])],
       [ np.array([-68.61183264,  -0.21232315,  32.84707963]), np.array([[ 0.99864274, -0.00538155,  0.05180529],
       [ 0.00527699,  0.99998384,  0.00215483],
       [-0.05181606, -0.00187851,  0.9986549 ]])],
       [ np.array([-69.54832495,  -0.32143422,  31.95026492]), np.array([[ 0.9991783 , -0.0107607 ,  0.03907496],
       [ 0.01088172,  0.99993666, -0.0028859 ],
       [-0.03904143,  0.00330862,  0.99923216]])],
       [ np.array([-70.96072921,  -0.62346984,  30.96529228]), np.array([[ 0.99990782, -0.01045392,  0.00865956],
       [ 0.01051936,  0.99991613, -0.0075462 ],
       [-0.00857995,  0.00763665,  0.999934  ]])],
       [ np.array([-72.35619886,  -0.56641044,  30.18126581]), np.array([[ 0.99985765, -0.01595814, -0.00546159],
       [ 0.01591153,  0.99983752, -0.00847297],
       [ 0.00559591,  0.00838475,  0.99994921]])]
        ]

    demopathlist = []
    for num in range(len(demolist) - 1):
        pose = [np.array([(demolist[num][0][0] + demolist[num + 1][0][0]) / 2,
                          (demolist[num][0][1] + demolist[num + 1][0][1]) / 2,
                          (demolist[num][0][2] + demolist[num + 1][0][2]) / 2]),
                np.array([[(demolist[num][1][0][0] + demolist[num + 1][1][0][0]) / 2,
                           (demolist[num][1][0][1] + demolist[num + 1][1][0][1]) / 2,
                           (demolist[num][1][0][2] + demolist[num + 1][1][0][2]) / 2],
                          [(demolist[num][1][1][0] + demolist[num + 1][1][1][0]) / 2,
                           (demolist[num][1][1][1] + demolist[num + 1][1][1][1]) / 2,
                           (demolist[num][1][1][2] + demolist[num + 1][1][1][2]) / 2],
                          [(demolist[num][1][2][0] + demolist[num + 1][1][2][0]) / 2,
                           (demolist[num][1][2][1] + demolist[num + 1][1][2][1]) / 2,
                           (demolist[num][1][2][2] + demolist[num + 1][1][2][2]) / 2]
                          ])]

        demopathlist.append(pose)
    demopathlistxyzRPY = []
    for i in range(len(demopathlist)):
        relpos = demopathlist[i][0]
        relrot = demopathlist[i][1]
        relmat = base.pg.npToMat4(relrot, relpos)
        posL = relmat * posG
        virtualgoalpos, virtualgoalrot = setPosRPY(objcm, posL)
        demopose = np.array([virtualgoalpos[0], 0, virtualgoalpos[2],
                             0, virtualgoalrot[1], 0], dtype=float)
        demopathlistxyzRPY.append(demopose)
    print(demopathlistxyzRPY)
    # read rearranged demo poses
    demopathlist1 = readdemopose("RearrangedLpath(1st deriv).txt", posG)
    demopathlist2 = readdemopose("RearrangedLpath(2nd deriv).txt", posG)
    print("Demopathlist 1 is", demopathlist1)
    print("Demopathlist 2 is", demopathlist2)
    testpathlist = []
    pathlist = []
    poseidlist = []

    # for pose in demopathlist1:  ## Don't forget to change the output file!
    for pose in demopathlist2:
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
        for pose in dlist:
            pose[2] += 1
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
    print("Planning time is %f s" % plantime)
    print("Total time cost is %f s" % (time_total_e - time_total_s))
    # writefile(wholepath, "Planned_Lpath_deriv1.txt")
    # writefile(wholepath, "Planned_Lpath_deriv2.txt")

    num = 0
    for pos in wholepath:
        num += 0.05
        tmpobjcm = copy.deepcopy(objcm)
        tmpobjcm.setPos(pos[0], pos[1], pos[2])
        tmpobjcm.setRPY(pos[3], pos[4], pos[5])
        tmpobjcm.setColor(1, 1, 0, 1-num*0.07)
        tmpobjcm.reparentTo(base.render)

    base.run()
