import os
import utiltools.robotmath as rm
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import numpy as np

def readtest(filename):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "r")
    f1 = f.readlines()
    poselist = []
    for string in f1:
        strlist = string.strip("[").strip(" ").replace("]", "").rstrip(" ")
        # s = " ".join(strlist.split()).replace(" ", ",")
        s = " ".join(strlist.split())
        # print(s)
        list1 = eval(s)
        poselist.append([list1[0], list1[1], list1[2], list1[3], list1[4], list1[5]])
    return poselist

if __name__ == '__main__':
    base = pc.World(camp=[0, 800, 0], lookatp=[0, 0, 0], up=[0, 0, 1], fov=40, w=1920, h=1080)
    l = readtest("Planned_Lpath_deriv2.txt")
    print(type(l), l)
    pos = np.array([l[1][0], l[1][1], l[1][2],])
    rot = rm.euler_matrix(l[1][3], l[1][4], l[1][5])
    m = rm.homobuild(pos, rot)
    print(type(m), m)
    # print(base.pg.mat3ToNp(rm.euler_matrix(l[1][3], l[1][4], l[1][5])))