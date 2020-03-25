from __future__ import absolute_import, division, print_function
import pandas as pd
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
from tensorflow.keras.models import load_model
import pickle

def resize_panda3dwin(w, h):
    props = WindowProperties()
    props.setSize(w, h)
    base.win.requestProperties(props)
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
def get_value_high(df, index):
    return [round(df.at[index, 'sc_px'], 3), round(df.at[index, 'sc_py'], 3), round(df.at[index, 'sc_pz'], 3),
            round(df.at[index, 'sc_prx'], 3),round(df.at[index, 'sc_pry'], 3), round(df.at[index, 'sc_prz'], 3),
            round(df.at[index, 'sh_px'], 3), round(df.at[index, 'sh_py'], 3), round(df.at[index, 'sh_pz'], 3),
            round(df.at[index, 'sh_prx'], 3), round(df.at[index, 'sh_pry'], 3), round(df.at[index, 'sh_prz'], 3)]
def get_value_low(df, index):
    return [round(df.at[index, 'sc_px'], 3), round(df.at[index, 'sc_py'], 3), round(df.at[index, 'sc_pz'], 3),
            round(df.at[index, 'sc_prx'], 3),round(df.at[index, 'sc_pry'], 3), round(df.at[index, 'sc_prz'], 3),
            round(df.at[index, 'sl_px'], 3), round(df.at[index, 'sl_py'], 3), round(df.at[index, 'sl_pz'], 3),
            round(df.at[index, 'sl_prx'], 3), round(df.at[index, 'sl_pry'], 3), round(df.at[index, 'sl_prz'], 3)]
def norm_high(var):
    x = {'sc_px': [var[0]], 'sc_py': [var[1]], 'sc_pz': [var[2]], 'sc_prx': [var[3]], 'sc_pry': [var[4]], 'sc_prz': [var[5]],
         'sh_px': [var[6]], 'sh_py': var[[7]], 'sh_pz': [var[8]], 'sh_prx': [var[9]], 'sh_pry': [var[10]], 'sh_prz': [var[11]]}
    df = pd.DataFrame(data=x)
    with open(os.path.join(this_dir, "nnmodel", "high_train_data_mean_std.pickle"), "rb") as f:
        train_mean, train_std = pickle.load(f)
    return (df - train_mean) / train_std
def norm_low(var):
    x = {'sc_px': [var[0]], 'sc_py': [var[1]], 'sc_pz': [var[2]], 'sc_prx': [var[3]], 'sc_pry': [var[4]], 'sc_prz': [var[5]],
         'sl_px': [var[6]], 'sl_py': var[[7]], 'sl_pz': [var[8]], 'sl_prx': [var[9]], 'sl_pry': [var[10]], 'sl_prz': [var[11]]}
    df = pd.DataFrame(data=x)
    with open(os.path.join(this_dir, "nnmodel", "low_train_data_mean_std.pickle"), "rb") as f:
        train_mean, train_std = pickle.load(f)
    return (df - train_mean) / train_std
def predict_high(sc, sh):
    high_model = load_model(os.path.join(this_dir, "nnmodel", "high_model.h5"))
    normed_sh = norm_high(np.concatenate((sc, sh), axis=0))
    sl = high_model.predict([get_value_high(normed_sh, 0)])
    subgoal = sl[0]
    return subgoal
def predict_low(sc, sl):
    low_model = load_model(os.path.join(this_dir, "nnmodel", "low_model.h5"))
    normed_sl = norm_low(np.concatenate((sc, sl), axis=0))
    a = high_model.predict([get_value_low(normed_sl, 0)])
    action = a[0]
    return action
if __name__=="__main__":
    this_dir, this_filename = os.path.split(__file__)

    base = pc.World(camp=[700, 1000, -300], lookatp=[700, 0, -300], up=[0, 0, 1], fov=40, w=1920, h=1080)
    env = bldgfsettingnear.Env()
    # self.env.reparentTo(base.render)
    objname = "new_LSHAPE.stl"
    objcm = env.loadobj(objname)
    groove = env.loadobj("new_GROOVE.stl")

    # read demonstration path file
    state = pd.read_csv(os.path.join(this_dir, "document", "State.csv"),
                        names=['x', 'y', 'z', 'R', 'P', 'Y'], na_values="?", comment='\t', sep=",",
                        skipinitialspace=True)
    demopath = []
    for index in range(23*int(len(state)/100), 24*int(len(state)/100)):
        demopath.append([state.at[index, 'x'], state.at[index, 'y'], state.at[index, 'z'],
                         state.at[index, 'R'], state.at[index, 'P'], state.at[index, 'Y']])
    finalgoalL = demopath[-1]
    print(finalgoalL)
    relpos1 = np.array([45 + 0.5, 0, -30 - 0.5])
    relrot1 = rm.euler_matrix(0, 0, 0)
    relmat1 = base.pg.npToMat4(relrot1, relpos1)
    posG = relmat1 * base.pg.npToMat4(rm.euler_matrix(finalgoalL[3], finalgoalL[4], finalgoalL[5]),
                                      np.array([finalgoalL[0], finalgoalL[1], finalgoalL[2]]))
    setPosRPY(groove, posG)
    groove.setColor(0, 0, 1, 0.4)
    groove.reparentTo(base.render)

    # startL = np.array([608, 72, -232, -1, 20, -4], dtype=float)
    # finalgoalL = np.array([660, 65, -280, 0, 0, -4], dtype=float)
    # # ------------------------------------------------------------------------------------------------------------------
    #
    # low_model = load_model(os.path.join(this_dir, "nnmodel", "low_model.h5"))
    # high_model = load_model(os.path.join(this_dir, "nnmodel", "high_model.h5"))
    #
    # subgoal = predict_high(startL, finalgoalL)
    # print("subgoal is", subgoal)
    #
    # action = predict_low(startL, subgoal)
    # print("action is", action)
    #
    # # ------------------------------------------------------------------------------------------------------------------
    # wholepath = []
    # for pose in [startL, subgoal, finalgoalL]:
    #     wholepath.append(pose)

    # draw the demonstration path
    num = 0
    for pos in demopath:
        # num += 0.05
        tmpobjcm = copy.deepcopy(objcm)
        tmpobjcm.setPos(pos[0], pos[1], pos[2])
        tmpobjcm.setRPY(pos[3], pos[4], pos[5])
        tmpobjcm.setColor(1, 1, 0, 1-num*0.07)
        tmpobjcm.reparentTo(base.render)

    resize_panda3dwin(w=1024, h=768)
    base.run()
